//package org.example.nareugobackend.domain.refreshToken;
//
//import java.util.concurrent.TimeUnit;
//import lombok.Getter;
//import lombok.NoArgsConstructor;
//import org.springframework.data.annotation.Id;
//import org.springframework.data.redis.core.RedisHash;
//import org.springframework.data.redis.core.TimeToLive;
//
//@Getter
//@NoArgsConstructor
//@RedisHash("refresh_token")
//public class RefreshToken {
//
//  @Id
//  private Long userId;
//
//  private String token;
//
//  @TimeToLive(unit = TimeUnit.DAYS)
//  private Long expiration = 7L;
//
//  private RefreshToken(Long userId, String token) {
//    this.userId = userId;
//    this.token = token;
//  }
//
//  public static RefreshToken of(Long memberId, String token) {
//    return new RefreshToken(memberId, token);
//  }
//}