//package org.example.nareugobackend.config.Security;
//
//import java.util.List;
//
//import lombok.RequiredArgsConstructor;
//import org.example.nareugobackend.api.service.auth.CustomOAuth2UserService;
//import org.example.nareugobackend.api.service.auth.CustomOidcUserService;
//import org.example.nareugobackend.common.filter.JwtAuthenticationFilter;
//import org.example.nareugobackend.common.handler.OAuth2AuthenticationFailureHandler;
//import org.example.nareugobackend.common.handler.OAuth2AuthenticationSuccessHandler;
//import org.springframework.context.annotation.Bean;
//import org.springframework.context.annotation.Configuration;
//import org.springframework.security.config.Customizer;
//import org.springframework.security.config.annotation.web.builders.HttpSecurity;
//import org.springframework.security.config.annotation.web.configuration.EnableWebSecurity;
//import org.springframework.security.config.http.SessionCreationPolicy;
//import org.springframework.security.web.SecurityFilterChain;
//import org.springframework.security.web.authentication.UsernamePasswordAuthenticationFilter;
//import org.springframework.web.cors.CorsConfiguration;
//import org.springframework.web.cors.CorsConfigurationSource;
//import org.springframework.web.cors.UrlBasedCorsConfigurationSource;
//
//@Configuration
//@EnableWebSecurity
//@RequiredArgsConstructor
//public class SecurityConfig {
//
//  private final CustomOAuth2UserService customOAuth2UserService;
//  private final CustomOidcUserService customOidcUserService;
//  private final OAuth2AuthenticationSuccessHandler oAuth2AuthenticationSuccessHandler;
//  private final OAuth2AuthenticationFailureHandler oAuth2AuthenticationFailureHandler;
//  private final JwtAuthenticationFilter jwtAuthenticationFilter;
//
//  @Bean
//  public SecurityFilterChain securityFilterChain(HttpSecurity http) throws Exception {
//    http
//        .csrf(csrf -> csrf.disable())
//        .cors(c -> c.configurationSource(corsConfigurationSource()))
//        .sessionManagement(s -> s.sessionCreationPolicy(SessionCreationPolicy.STATELESS))
//        .authorizeHttpRequests(auth -> auth
//            .requestMatchers("/auth/**", "/oauth2/**", "/login/**", "/api/v1/auth/**").permitAll()
//            .anyRequest().authenticated()
//        )
//        .httpBasic(b -> b.disable())
//        .formLogin(f -> f.disable())
//        .oauth2Login(oauth2 -> oauth2
//            .authorizationEndpoint(a -> a.baseUri("/oauth2/authorization"))
//            .redirectionEndpoint(r -> r.baseUri("/login/oauth2/code/*"))
//            .userInfoEndpoint(u -> u
//                .userService(customOAuth2UserService)
//                .oidcUserService(customOidcUserService))
//            .successHandler(oAuth2AuthenticationSuccessHandler)
//            .failureHandler(oAuth2AuthenticationFailureHandler)
//        );
//
//    http.addFilterBefore(jwtAuthenticationFilter, UsernamePasswordAuthenticationFilter.class);
//    return http.build();
//  }
//
//  @Bean
//  public CorsConfigurationSource corsConfigurationSource() {
//    CorsConfiguration cfg = new CorsConfiguration();
//    cfg.setAllowedOrigins(List.of("http://localhost:5173", "http://127.0.0.1:5173"));
//    cfg.setAllowedMethods(List.of("GET","POST","PUT","PATCH","DELETE","OPTIONS"));
//    cfg.setAllowedHeaders(List.of("Authorization","Content-Type","X-Requested-With"));
//    cfg.setExposedHeaders(List.of("Authorization","Set-Cookie"));
//    cfg.setAllowCredentials(true);
//
//    UrlBasedCorsConfigurationSource source = new UrlBasedCorsConfigurationSource();
//    source.registerCorsConfiguration("/**", cfg);
//    return source;
//  }
//}

package org.example.nareugobackend.config.Security;

import lombok.RequiredArgsConstructor;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.security.config.annotation.web.builders.HttpSecurity;
import org.springframework.security.config.annotation.web.configuration.EnableWebSecurity;
import org.springframework.security.config.http.SessionCreationPolicy;
import org.springframework.security.web.SecurityFilterChain;
import org.springframework.web.cors.CorsConfiguration;
import org.springframework.web.cors.CorsConfigurationSource;
import org.springframework.web.cors.UrlBasedCorsConfigurationSource;

import java.util.List;

@Configuration
@EnableWebSecurity
@RequiredArgsConstructor
public class SecurityConfig {

  @Bean
  public SecurityFilterChain securityFilterChain(HttpSecurity http) throws Exception {
    http
        .csrf(csrf -> csrf.disable())
        .cors(c -> c.configurationSource(corsConfigurationSource()))
        .sessionManagement(s -> s.sessionCreationPolicy(SessionCreationPolicy.STATELESS))
        .authorizeHttpRequests(auth -> auth
            .anyRequest().permitAll()  // 모든 요청 허용
        )
        .httpBasic(b -> b.disable())
        .formLogin(f -> f.disable());

    return http.build();
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