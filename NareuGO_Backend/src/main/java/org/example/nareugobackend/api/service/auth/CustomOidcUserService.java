package org.example.nareugobackend.api.service.auth;

import org.example.nareugobackend.api.service.auth.info.KakaoUserInfo;
import org.example.nareugobackend.common.jwt.AccountContext;
import org.springframework.security.oauth2.client.oidc.userinfo.OidcUserRequest;
import org.springframework.security.oauth2.client.oidc.userinfo.OidcUserService;
import org.springframework.security.oauth2.core.OAuth2AuthenticationException;
import org.springframework.security.oauth2.core.oidc.user.OidcUser;
import org.springframework.stereotype.Service;

@Service
public class CustomOidcUserService extends OidcUserService {

    @Override
    public OidcUser loadUser(OidcUserRequest userRequest) throws OAuth2AuthenticationException {
        OidcUser oidcUser = super.loadUser(userRequest);

        String registrationId = userRequest.getClientRegistration().getRegistrationId();

        if ("kakao".equals(registrationId)) {
            KakaoUserInfo kakaoUserInfo = convertToKakaoUserInfo(oidcUser);
            return AccountContext.fromOidcUser(oidcUser, kakaoUserInfo, registrationId);
        }

        throw new OAuth2AuthenticationException("Unsupported provider: " + registrationId);
    }

    private KakaoUserInfo convertToKakaoUserInfo(OidcUser oidcUser) {
        return new KakaoUserInfo(
            oidcUser.getSubject(),
            oidcUser.getFullName(),
            (String) oidcUser.getAttributes().get("gender"),
            oidcUser.getEmail(),
            (String) oidcUser.getAttributes().get("birthdate"),
            (String) oidcUser.getAttributes().get("phone_number")
        );
    }
}