//package org.example.nareugobackend.api.service.auth;
//
//import jakarta.servlet.http.HttpServletRequest;
//import jakarta.servlet.http.HttpServletResponse;
//import lombok.RequiredArgsConstructor;
//import jakarta.servlet.http.Cookie;
//import jakarta.servlet.http.HttpServletRequest;
//import jakarta.servlet.http.HttpServletResponse;
//import java.util.Optional;
//import lombok.RequiredArgsConstructor;
//import org.example.nareugobackend.api.service.user.UserService;
//import org.example.nareugobackend.common.cookie.CookieUtil;
//import org.example.nareugobackend.common.exception.auth.AuthException;
//import org.example.nareugobackend.common.exception.auth.AuthErrorCode;
//import org.example.nareugobackend.common.jwt.AuthToken;
//import org.example.nareugobackend.common.jwt.JwtTokenProvider;
//import org.example.nareugobackend.domain.refreshToken.RefreshToken;
//import org.example.nareugobackend.domain.refreshToken.RefreshTokenRepository;
//import org.example.nareugobackend.domain.user.User;
//import org.springframework.stereotype.Service;
//import org.springframework.transaction.annotation.Transactional;
//
//@Service
//@RequiredArgsConstructor
//@Transactional(readOnly = true)
//public class AuthService {
//
//  private final JwtTokenProvider jwtTokenProvider;
//  private final RefreshTokenRepository refreshTokenRepository;
//  private final CookieUtil cookieUtil;
//  private final UserService userService;
//
//  public String refreshAccessToken(HttpServletRequest request) {
//    String refreshToken = extractRefreshTokenFromCookie(request);
//    validateRefreshToken(refreshToken);
//
//    String userId = jwtTokenProvider.getUserId(refreshToken);
//    validateRefreshTokenInDatabase(refreshToken);
//
//    AuthToken newAccessToken = jwtTokenProvider.createAccessToken(userId);
//    return newAccessToken.token();
//  }
//
//  @Transactional
//  public void logout(HttpServletRequest request, HttpServletResponse response) {
//    String refreshToken = extractRefreshTokenFromCookie(request);
//
//    if (refreshToken != null && jwtTokenProvider.validateToken(refreshToken)) {
//      removeRefreshTokenFromDatabase(refreshToken);
//    }
//
//    clearAuthCookies(request, response);
//  }
//
//  public String extractAccessToken(HttpServletRequest request) {
//    String authHeader = request.getHeader("Authorization");
//    if (authHeader != null && authHeader.startsWith("Bearer ")) {
//      return authHeader.substring(7);
//    }
//
//    return cookieUtil.getCookie(request, "access_token")
//        .map(Cookie::getValue)
//        .orElse(null);
//  }
//
//  private String extractRefreshTokenFromCookie(HttpServletRequest request) {
//    return cookieUtil.getCookie(request, "refresh_token")
//        .map(Cookie::getValue)
//        .orElse(null);
//  }
//
//  private void validateRefreshToken(String refreshToken) {
//    if (refreshToken == null) {
//      throw new AuthException(AuthErrorCode.TOKEN_INVALID);
//    }
//
//    if (!jwtTokenProvider.validateToken(refreshToken)) {
//      throw new AuthException(AuthErrorCode.TOKEN_INVALID);
//    }
//  }
//
//  private void validateRefreshTokenInDatabase(String refreshToken) {
//    Optional<RefreshToken> refreshTokenEntity = refreshTokenRepository.findByToken(refreshToken);
//
//    if (refreshTokenEntity.isEmpty() || !refreshTokenEntity.get().getToken().equals(refreshToken)) {
//      throw new AuthException(AuthErrorCode.TOKEN_INVALID);
//    }
//  }
//
//  private void removeRefreshTokenFromDatabase(String refreshToken) {
//    refreshTokenRepository.findByToken(refreshToken)
//        .ifPresent(refreshTokenRepository::delete);
//  }
//
//  private void clearAuthCookies(HttpServletRequest request, HttpServletResponse response) {
//    cookieUtil.deleteCookie(request, response, "access_token");
//    cookieUtil.deleteCookie(request, response, "refresh_token");
//  }
//
//  public User getCurrentUser(HttpServletRequest request) {
//    String accessToken = extractAccessToken(request);
//    validateAccessToken(accessToken);
//    String userId = jwtTokenProvider.getUserId(accessToken);
//    return userService.findById(Long.parseLong(userId));
//  }
//
//  private void validateAccessToken(String accessToken) {
//    if (accessToken == null) {
//      throw new AuthException(AuthErrorCode.TOKEN_INVALID);
//    }
//
//    if (!jwtTokenProvider.validateToken(accessToken)) {
//      throw new AuthException(AuthErrorCode.TOKEN_INVALID);
//    }
//  }
//}