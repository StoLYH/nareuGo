//package org.example.nareugobackend.api.controller.auth;
//
//import jakarta.servlet.http.HttpServletRequest;
//import jakarta.servlet.http.HttpServletResponse;
//import java.util.Map;
//import lombok.RequiredArgsConstructor;
//import lombok.extern.slf4j.Slf4j;
//import org.example.nareugobackend.api.service.auth.AuthService;
//import org.example.nareugobackend.domain.user.User;
//import org.springframework.http.HttpStatus;
//import org.springframework.http.ResponseEntity;
//import org.springframework.web.bind.annotation.GetMapping;
//import org.springframework.web.bind.annotation.PostMapping;
//import org.springframework.web.bind.annotation.RequestMapping;
//import org.springframework.web.bind.annotation.RestController;
//
//@Slf4j
//@RestController
//@RequestMapping("/api/v1/auth")
//@RequiredArgsConstructor
//public class AuthController {
//
//  private final AuthService authService;
//
//  @PostMapping("/refresh")
//  public ResponseEntity<?> refreshToken(HttpServletRequest request, HttpServletResponse response) {
//    try {
//      String newAccessToken = authService.refreshAccessToken(request);
//      return ResponseEntity.ok().body(Map.of("accessToken", newAccessToken));
//    } catch (Exception e) {
//      return ResponseEntity.status(HttpStatus.UNAUTHORIZED)
//          .body(Map.of("error", e.getMessage()));
//    }
//  }
//
//  @PostMapping("/logout")
//  public ResponseEntity<?> logout(HttpServletRequest request, HttpServletResponse response) {
//    authService.logout(request, response);
//    return ResponseEntity.ok().body("Logout successful");
//  }
//
//  @GetMapping("/user")
//  public ResponseEntity<?> getCurrentUser(HttpServletRequest request) {
//    try {
//      User user = authService.getCurrentUser(request);
//      return ResponseEntity.ok().body(user);
//    } catch (RuntimeException e) {
//      if (e.getMessage().contains("not found")) {
//        return ResponseEntity.status(HttpStatus.UNAUTHORIZED).body(e.getMessage());
//      } else if (e.getMessage().contains("User not found")) {
//        return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage());
//      }
//      return ResponseEntity.status(HttpStatus.UNAUTHORIZED).body(e.getMessage());
//    }
//  }
//}