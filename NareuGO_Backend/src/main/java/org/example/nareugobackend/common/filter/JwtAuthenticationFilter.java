package org.example.nareugobackend.common.filter;

import jakarta.servlet.FilterChain;
import jakarta.servlet.ServletException;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import java.io.IOException;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.example.nareugobackend.common.jwt.JwtTokenProvider;
import org.example.nareugobackend.common.jwt.JwtTokenValidator;
import org.springframework.security.core.Authentication;
import org.springframework.security.core.context.SecurityContextHolder;
import org.springframework.stereotype.Component;
import org.springframework.web.filter.OncePerRequestFilter;

@Slf4j
@Component
@RequiredArgsConstructor
public class JwtAuthenticationFilter extends OncePerRequestFilter {

  private final JwtTokenProvider jwtTokenProvider;
  private final JwtTokenValidator jwtTokenValidator;

  @Override
  protected void doFilterInternal(HttpServletRequest request,
      HttpServletResponse response,
      FilterChain filterChain) throws ServletException, IOException {

    // CORS preflight 요청은 바로 통과
    if ("OPTIONS".equalsIgnoreCase(request.getMethod())) {
      filterChain.doFilter(request, response);
      return;
    }

    String token = null;
    try {
      token = jwtTokenValidator.resolveToken(request);
    } catch (Exception e) {
      // 토큰 파싱 중 예외가 나도 체인을 계속 태워 permitAll 경로를 막지 않음
    }

    if (token == null || token.isBlank()) {
      // 무토큰 요청은 그대로 다음 필터로
      filterChain.doFilter(request, response);
      return;
    }

    try {
      if (jwtTokenValidator.validateToken(token)) {
        Authentication authentication = jwtTokenValidator.getAuthentication(token);
        SecurityContextHolder.getContext().setAuthentication(authentication);
      } else {
        // 유효하지 않은 토큰이면 컨텍스트만 비우고 계속 진행
        SecurityContextHolder.clearContext();
      }
    } catch (Exception e) {
      // 검증 중 예외가 발생해도 여기서 401을 직접 내려보내지 않음
      SecurityContextHolder.clearContext();
    }

    filterChain.doFilter(request, response);
  }
}