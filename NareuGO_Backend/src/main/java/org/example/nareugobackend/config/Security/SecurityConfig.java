package org.example.nareugobackend.config.Security;

import java.util.List;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.common.filter.JwtAuthenticationFilter;
import org.example.nareugobackend.common.handler.OAuth2AuthenticationSuccessHandler;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.security.config.annotation.web.builders.HttpSecurity;
import org.springframework.security.config.annotation.web.configuration.EnableWebSecurity;
import org.springframework.security.config.http.SessionCreationPolicy;
import org.springframework.security.oauth2.client.registration.ClientRegistration;
import org.springframework.security.oauth2.client.registration.ClientRegistrationRepository;
import org.springframework.security.oauth2.client.registration.InMemoryClientRegistrationRepository;
import org.springframework.security.oauth2.core.AuthorizationGrantType;
import org.springframework.security.oauth2.core.ClientAuthenticationMethod;
import org.springframework.security.web.SecurityFilterChain;
import org.springframework.security.web.authentication.UsernamePasswordAuthenticationFilter;
import org.springframework.web.cors.CorsConfiguration;
import org.springframework.web.cors.CorsConfigurationSource;
import org.springframework.web.cors.UrlBasedCorsConfigurationSource;

@Configuration
@EnableWebSecurity
@RequiredArgsConstructor
public class SecurityConfig {

  private final JwtAuthenticationFilter jwtAuthenticationFilter;
  private final OAuth2AuthenticationSuccessHandler oAuth2AuthenticationSuccessHandler;

  @Bean
  public SecurityFilterChain securityFilterChain(HttpSecurity http) throws Exception {
    http
        .csrf(csrf -> csrf.disable())
        .cors(c -> c.configurationSource(corsConfigurationSource()))
        .sessionManagement(s -> s.sessionCreationPolicy(SessionCreationPolicy.STATELESS))
        .authorizeHttpRequests(auth -> auth
            .requestMatchers("/auth/**", "/oauth2/**", "/login/**", "/api/v1/auth/**").permitAll()
            .anyRequest().authenticated()
        )
        .httpBasic(b -> b.disable())
        .formLogin(f -> f.disable())
        .oauth2Login(oauth2 -> oauth2
            .authorizationEndpoint(a -> a.baseUri("/oauth2/authorization"))
            .redirectionEndpoint(r -> r.baseUri("/login/oauth2/code/*"))
            .successHandler(oAuth2AuthenticationSuccessHandler)
        );

    http.addFilterBefore(jwtAuthenticationFilter, UsernamePasswordAuthenticationFilter.class);
    return http.build();
  }


  @Bean
  public ClientRegistrationRepository clientRegistrationRepository(
      @Value("${OAUTH2_KAKAO_CLIENT_ID}") String kakaoClientId,
      @Value("${OAUTH2_KAKAO_CLIENT_SECRET_KEY}") String kakaoClientSecret,
      @Value("${OAUTH2_KAKAO_LOGIN_REDIRECT_URI}") String kakaoRedirectUri,
      @Value("${OAUTH2_NAVER_CLIENT_ID}") String naverClientId,
      @Value("${OAUTH2_NAVER_CLIENT_SECRET_KEY}") String naverClientSecret,
      @Value("${OAUTH2_NAVER_LOGIN_REDIRECT_URI}") String naverRedirectUri
  ) {
    System.out.println("=== OAuth2 Configuration ===");
    System.out.println("Kakao Redirect URI: " + kakaoRedirectUri);
    System.out.println("Naver Redirect URI: " + naverRedirectUri);
    System.out.println("=============================");
    ClientRegistration kakao = ClientRegistration.withRegistrationId("kakao")
        .clientId(kakaoClientId)
        .clientSecret(kakaoClientSecret)
        .clientAuthenticationMethod(ClientAuthenticationMethod.CLIENT_SECRET_POST)
        .authorizationGrantType(AuthorizationGrantType.AUTHORIZATION_CODE)
        .redirectUri(kakaoRedirectUri)
        .scope("profile_nickname", "profile_image", "account_email")
        .clientName("Kakao")
        .authorizationUri("https://kauth.kakao.com/oauth/authorize")
        .tokenUri("https://kauth.kakao.com/oauth/token")
        .userInfoUri("https://kapi.kakao.com/v2/user/me")
        .userNameAttributeName("id")
        .build();

    ClientRegistration naver = ClientRegistration.withRegistrationId("naver")
        .clientId(naverClientId)
        .clientSecret(naverClientSecret)
        .clientAuthenticationMethod(ClientAuthenticationMethod.CLIENT_SECRET_POST)
        .authorizationGrantType(AuthorizationGrantType.AUTHORIZATION_CODE)
        .redirectUri(naverRedirectUri)
        .scope("name", "email", "profile_image")
        .clientName("Naver")
        .authorizationUri("https://nid.naver.com/oauth2.0/authorize")
        .tokenUri("https://nid.naver.com/oauth2.0/token")
        .userInfoUri("https://openapi.naver.com/v1/nid/me")
        .userNameAttributeName("response")
        .build();

    return new InMemoryClientRegistrationRepository(kakao, naver);
  }

  @Bean
  public CorsConfigurationSource corsConfigurationSource() {
    CorsConfiguration cfg = new CorsConfiguration();
    cfg.setAllowedOrigins(List.of("http://localhost:5173", "http://127.0.0.1:5173"));
    cfg.setAllowedMethods(List.of("GET","POST","PUT","PATCH","DELETE","OPTIONS"));
    cfg.setAllowedHeaders(List.of("Authorization","Content-Type","X-Requested-With"));
    cfg.setExposedHeaders(List.of("Authorization","Set-Cookie"));
    cfg.setAllowCredentials(true);

    UrlBasedCorsConfigurationSource source = new UrlBasedCorsConfigurationSource();
    source.registerCorsConfiguration("/**", cfg);
    return source;
  }
}