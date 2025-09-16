package org.example.nareugobackend.api.service.auth;

import static org.example.nareugobackend.common.constant.ConstantUtil.BEARER_PREFIX;
import static org.example.nareugobackend.common.constant.ConstantUtil.BIRTHDAY_DAY_START_INDEX;
import static org.example.nareugobackend.common.constant.ConstantUtil.BIRTHDAY_LENGTH;
import static org.example.nareugobackend.common.constant.ConstantUtil.BIRTHDAY_MONTH_END_INDEX;
import static org.example.nareugobackend.common.constant.ConstantUtil.NAVER_USER_INFO_URL;
import static org.example.nareugobackend.common.constant.ConstantUtil.TOKEN_HEADER;

import java.util.Map;
import java.util.Optional;
import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.service.auth.info.KakaoUserInfo;
import org.example.nareugobackend.api.service.auth.info.NaverUserInfo;
import org.example.nareugobackend.api.service.auth.info.SocialUserInfo;
import org.example.nareugobackend.api.service.auth.request.NaverUserCreateServiceRequest;
import org.example.nareugobackend.common.jwt.AccountContext;
import org.springframework.security.oauth2.client.userinfo.DefaultOAuth2UserService;
import org.springframework.security.oauth2.client.userinfo.OAuth2UserRequest;
import org.springframework.security.oauth2.core.OAuth2AuthenticationException;
import org.springframework.security.oauth2.core.OAuth2Error;
import org.springframework.security.oauth2.core.user.OAuth2User;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.web.client.RestClient;

@Service
@Transactional
@RequiredArgsConstructor
public class CustomOAuth2UserService extends DefaultOAuth2UserService {

  private final RestClient restClient;

  @Override
  public OAuth2User loadUser(OAuth2UserRequest userRequest) throws OAuth2AuthenticationException {
    OAuth2User oAuth2User = super.loadUser(userRequest);
    String registrationId = userRequest.getClientRegistration().getRegistrationId();

    SocialUserInfo socialUserInfo = extractUserInfo(registrationId, userRequest, oAuth2User);

    return AccountContext.fromOAuth2User(oAuth2User, socialUserInfo, registrationId);
  }

  private SocialUserInfo extractUserInfo(String registrationId, OAuth2UserRequest userRequest, OAuth2User oAuth2User) {
    return switch (registrationId.toLowerCase()) {
      case "kakao" -> createKakaoUserInfo(oAuth2User);
      case "naver" -> getNaverUserInfo(userRequest);
      default -> throw new OAuth2AuthenticationException(
        new OAuth2Error("unsupported_provider"),
        "지원하지 않는 소셜 로그인 제공자입니다: " + registrationId
      );
    };
  }

//  private KakaoUserInfo createKakaoUserInfo(OAuth2User oAuth2User) {
//    var attributes = oAuth2User.getAttributes();
//    var kakaoAccount = getMapValue(attributes, "kakao_account");
//
//    String birthdate = formatBirthdate(
//        getStringValue(kakaoAccount, "birthyear"),
//        getStringValue(kakaoAccount, "birthday")
//    );
//
//    return new KakaoUserInfo(
//      getStringValue(attributes, "id"),                    // sub (provider ID)
//      getStringValue(kakaoAccount, "name"),                // name (실제 사용자 이름)
//      getStringValue(kakaoAccount, "gender"),              // gender (kakao_account 안에 있음)
//      getStringValue(kakaoAccount, "email"),               // email
//      birthdate,                                           // birthdate (년도+월일 조합)
//      getStringValue(kakaoAccount, "phone_number")         // phone_number
//    );
//  }

  private KakaoUserInfo createKakaoUserInfo(OAuth2User oAuth2User) {
    var attributes = oAuth2User.getAttributes();
    var kakaoAccount = getMapValue(attributes, "kakao_account");
    var profile = getMapValue(kakaoAccount, "profile");

    // 사용자 ID
    String id = getStringValue(attributes, "id");

    // 사용자 이름 (nickname)
    String name = getStringValue(profile, "nickname");
    if (name == null) {
      name = "알 수 없음"; // 기본값 설정
      System.out.println("Kakao profile.nickname 정보가 없습니다.");
    }

    // 성별
    String gender = getStringValue(kakaoAccount, "gender");
    if (gender == null) {
      gender = "unknown";
    }

    // 이메일
    String email = getStringValue(kakaoAccount, "email");

    // 전화번호
    String phoneNumber = getStringValue(kakaoAccount, "phone_number");

    // 생년월일
    String birthdate = formatBirthdate(
        getStringValue(kakaoAccount, "birthyear"),
        getStringValue(kakaoAccount, "birthday")
    );

    return new KakaoUserInfo(
        id,
        name,
        gender,
        email,
        birthdate,
        phoneNumber
    );
  }

  //////////////////

  @SuppressWarnings("unchecked")
  private Map<String, Object> getMapValue(Map<String, Object> map, String key) {
    Object value = map != null ? map.get(key) : null;
    return value instanceof Map ? (Map<String, Object>) value : Map.of();
  }

  private String getStringValue(Map<String, Object> map, String key) {
    return getStringValue(map, key, null);
  }

  private String getStringValue(Map<String, Object> map, String key, String defaultValue) {
    if (map == null) return defaultValue;
    Object value = map.get(key);
    if (value == null) return defaultValue;
    return String.valueOf(value);
  }

  private String formatBirthdate(String birthYear, String birthday) {
    if (birthYear == null || birthday == null || birthday.length() != BIRTHDAY_LENGTH) {
      return null;
    }
    return String.format("%s-%s-%s",
        birthYear,
        birthday.substring(0, BIRTHDAY_MONTH_END_INDEX),
        birthday.substring(BIRTHDAY_DAY_START_INDEX, BIRTHDAY_LENGTH)
    );
  }

  private NaverUserInfo getNaverUserInfo(OAuth2UserRequest userRequest) {
    String accessToken = userRequest.getAccessToken().getTokenValue();

    NaverUserCreateServiceRequest request = restClient.get()
        .uri(NAVER_USER_INFO_URL)
        .header(TOKEN_HEADER, BEARER_PREFIX + accessToken)
        .retrieve()
        .body(NaverUserCreateServiceRequest.class);

    return Optional.ofNullable(request)
        .map(NaverUserCreateServiceRequest::toNaverUserInfo)
        .orElseThrow(() -> new OAuth2AuthenticationException("존재하지 않는 네이버 사용자입니다."));
  }
}