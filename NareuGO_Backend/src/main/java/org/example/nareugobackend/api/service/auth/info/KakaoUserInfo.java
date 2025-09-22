package org.example.nareugobackend.api.service.auth.info;

import static org.example.nareugobackend.domain.user.ProviderType.KAKAO;
import static org.example.nareugobackend.domain.user.Sex.MALE;
import static org.example.nareugobackend.domain.user.Sex.FEMALE;

import org.apache.commons.lang3.StringUtils;
import org.example.nareugobackend.domain.user.ProviderType;
import org.example.nareugobackend.domain.user.Sex;

public record KakaoUserInfo(
    String sub,
    String name,
    String gender,
    String email,
    String birthdate,
    String phoneNumber
) implements SocialUserInfo {

  @Override
  public String getName() {
    return name;
  }

  @Override
  public String getEmail() {
    return email;
  }

  @Override
  public Sex getGender() {
    return StringUtils.equals(gender, "male") ? MALE : FEMALE;
  }

  @Override
  public String getBirth() {
    return birthdate;
  }

  @Override
  public String getProviderId() {
    return sub;
  }

  @Override
  public ProviderType getProviderType() {
    return KAKAO;
  }

  @Override
  public String getPhoneNumber() {
    if (phoneNumber == null || phoneNumber.length() <= 4) {
      return null;
    }
    String numberBody = phoneNumber.substring(4);
    return String.format("0%s", numberBody);
  }
}
