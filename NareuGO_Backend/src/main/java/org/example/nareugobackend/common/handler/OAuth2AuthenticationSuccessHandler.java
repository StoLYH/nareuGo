package org.example.nareugobackend.common.handler;

import static org.example.nareugobackend.common.constant.ConstantUtil.COOKIE_NAME;
import static org.example.nareugobackend.common.constant.ConstantUtil.REFRESH_TOKEN_EXPIRE_TIME;
import static org.example.nareugobackend.common.jwt.AuthTokenType.REFRESH;

import org.example.nareugobackend.common.jwt.AuthTokenType;

import jakarta.servlet.ServletException;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import java.io.IOException;
import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.service.user.UserService;
import org.example.nareugobackend.common.cookie.CookieUtil;
import org.example.nareugobackend.common.jwt.AccountContext;
import org.example.nareugobackend.common.jwt.AuthToken;
import org.example.nareugobackend.common.jwt.JwtTokenProvider;
import org.example.nareugobackend.domain.refreshToken.RefreshToken;
import org.example.nareugobackend.domain.refreshToken.RefreshTokenRepository;
import org.example.nareugobackend.domain.user.User;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.security.core.Authentication;
import org.springframework.security.web.DefaultRedirectStrategy;
import org.springframework.security.web.RedirectStrategy;
import org.springframework.security.web.authentication.AuthenticationSuccessHandler;
import org.springframework.stereotype.Component;
import org.springframework.web.util.UriComponentsBuilder;

@Component
@RequiredArgsConstructor
public class OAuth2AuthenticationSuccessHandler implements AuthenticationSuccessHandler {

  @Value("${host.frontend}")
  private String FRONT_SERVER;

  @Value("${host.login-success-redirect-url}")
  private String LOGIN_SUCCESS_REDIRECT_URL;

  private final CookieUtil cookieUtil;
  private final UserService userService;
  private final JwtTokenProvider jwtTokenProvider;
  private final RefreshTokenRepository refreshTokenRepository;

  private final RedirectStrategy redirectStrategy = new DefaultRedirectStrategy();

  @Override
  public void onAuthenticationSuccess(
      HttpServletRequest request,
      HttpServletResponse response,
      Authentication authentication
  ) throws IOException, ServletException {

    if (response.isCommitted()) {
      return;
    }

    AccountContext accountContext = (AccountContext) authentication.getPrincipal();
    User user = userService.registerUser(accountContext.getSocialUserInfo());
    accountContext.updateUserInfo(user);

    setRefreshTokenCookie(authentication, response);

    // Access Token 생성 (프론트엔드로 전달용)
    AuthToken accessToken = jwtTokenProvider.createToken(authentication, AuthTokenType.ACCESS);

    String redirectUri = determineTargetUrl(accessToken.token());
    redirectStrategy.sendRedirect(request, response, redirectUri);
  }

  private void setRefreshTokenCookie(Authentication authentication, HttpServletResponse response) {

    AccountContext accountContext = (AccountContext) authentication.getPrincipal();

    AuthToken refreshToken = jwtTokenProvider.createToken(authentication, REFRESH);
    refreshTokenRepository.save(RefreshToken.of(accountContext.getUserId(), refreshToken.token()));
    cookieUtil.addCookie(response, COOKIE_NAME, refreshToken.token(), REFRESH_TOKEN_EXPIRE_TIME);
  }

  private String determineTargetUrl(String accessToken) {
    return UriComponentsBuilder
        .fromUriString(FRONT_SERVER + LOGIN_SUCCESS_REDIRECT_URL)
        .queryParam("token", accessToken)
        .build().toUriString();
  }
}
