package org.example.nareugobackend.common.jwt;

import static io.jsonwebtoken.Header.JWT_TYPE;
import static io.jsonwebtoken.Header.TYPE;
import static io.jsonwebtoken.SignatureAlgorithm.HS256;
import static io.jsonwebtoken.io.Decoders.BASE64;
import static org.example.nareugobackend.common.constant.ConstantUtil.ACCESS_TOKEN_EXPIRE_TIME;
import static org.example.nareugobackend.common.constant.ConstantUtil.REFRESH_TOKEN_EXPIRE_TIME;
import static org.example.nareugobackend.common.jwt.AuthTokenType.ACCESS;
import static org.example.nareugobackend.common.jwt.AuthTokenType.REFRESH;

import io.jsonwebtoken.Claims;
import io.jsonwebtoken.Jwts;
import io.jsonwebtoken.security.Keys;
import java.security.Key;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import java.util.Date;
import java.util.List;
import org.example.nareugobackend.domain.user.User;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.security.core.Authentication;
import org.springframework.security.core.GrantedAuthority;
import org.springframework.stereotype.Component;

@Component
public class JwtTokenProvider {

  private final Key key;

  public JwtTokenProvider(@Value("${jwt.secret}") String secretKey) {
    byte[] keyBytes = BASE64.decode(secretKey);
    this.key = Keys.hmacShaKeyFor(keyBytes);
  }

  public AuthToken createToken(Authentication authentication, AuthTokenType tokenType) {
    AccountContext accountContext = (AccountContext) authentication.getPrincipal();

    Date now = new Date();
    int expiration = tokenType == ACCESS ? ACCESS_TOKEN_EXPIRE_TIME : REFRESH_TOKEN_EXPIRE_TIME;
    Date expiryDate = new Date(now.getTime() + expiration);
    String jwtToken = generateJwt(accountContext, now, expiryDate);

    return AuthToken.builder()
        .memberId(accountContext.getUserId())
        .token(jwtToken)
        .expiresIn(expiration)
        .expiryDate(getLocalDateTime(expiryDate))
        .type(tokenType)
        .build();
  }

  private static String getLocalDateTime(Date expiryDate) {
    LocalDateTime localDateTime = LocalDateTime.ofInstant(expiryDate.toInstant(), ZoneId.systemDefault());
    DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss");
    return localDateTime.format(formatter);
  }

  private String generateJwt(AccountContext accountContext, Date now, Date expiryDate) {

    List<String> authorities = accountContext.getAuthorities().stream()
        .map(GrantedAuthority::getAuthority)
        .toList();

    return Jwts.builder()
        .setHeaderParam(TYPE, JWT_TYPE)
        .setSubject("NareuGo")
        .claim("userId", accountContext.getUserId())
        .claim("authorities", authorities)
        .setIssuedAt(now)
        .setExpiration(expiryDate)
        .signWith(key, HS256)
        .compact();
  }

  public AuthToken createAccessToken(String userId) {
    Date now = new Date();
    Date expiryDate = new Date(now.getTime() + ACCESS_TOKEN_EXPIRE_TIME);
    
    String jwtToken = Jwts.builder()
        .setHeaderParam(TYPE, JWT_TYPE)
        .setSubject(userId)
        .claim("userId", userId)
        .setIssuedAt(now)
        .setExpiration(expiryDate)
        .signWith(key, HS256)
        .compact();

    return AuthToken.builder()
        .memberId(Long.valueOf(userId))
        .token(jwtToken)
        .expiresIn(ACCESS_TOKEN_EXPIRE_TIME)
        .expiryDate(getLocalDateTime(expiryDate))
        .type(ACCESS)
        .build();
  }

  public AuthToken createRefreshToken(String userId) {
    Date now = new Date();
    Date expiryDate = new Date(now.getTime() + REFRESH_TOKEN_EXPIRE_TIME);
    
    String jwtToken = Jwts.builder()
        .setHeaderParam(TYPE, JWT_TYPE)
        .setSubject(userId)
        .claim("userId", userId)
        .setIssuedAt(now)
        .setExpiration(expiryDate)
        .signWith(key, HS256)
        .compact();

    return AuthToken.builder()
        .memberId(Long.valueOf(userId))
        .token(jwtToken)
        .expiresIn(REFRESH_TOKEN_EXPIRE_TIME)
        .expiryDate(getLocalDateTime(expiryDate))
        .type(REFRESH)
        .build();
  }

  public boolean validateToken(String token) {
    try {
      Jwts.parserBuilder().setSigningKey(key).build().parseClaimsJws(token);
      return true;
    } catch (Exception e) {
      return false;
    }
  }

  public Claims getClaims(String token) {
    return Jwts.parserBuilder()
        .setSigningKey(key)
        .build()
        .parseClaimsJws(token)
        .getBody();
  }

  public String getUserId(String token) {
    Claims claims = getClaims(token);
    Object userId = claims.get("userId");
    return userId != null ? userId.toString() : null;
  }

  public long getRefreshTokenExpiry() {
    return REFRESH_TOKEN_EXPIRE_TIME;
  }

  public AuthToken createTokenForUser(User user, AuthTokenType tokenType) {
    Date now = new Date();
    int expiration = tokenType == ACCESS ? ACCESS_TOKEN_EXPIRE_TIME : REFRESH_TOKEN_EXPIRE_TIME;
    Date expiryDate = new Date(now.getTime() + expiration);
    
    String jwtToken = Jwts.builder()
        .setHeaderParam(TYPE, JWT_TYPE)
        .setSubject("NareuGo")
        .claim("userId", user.getId())
        .claim("authorities", List.of("ROLE_USER"))
        .setIssuedAt(now)
        .setExpiration(expiryDate)
        .signWith(key, HS256)
        .compact();

    return AuthToken.builder()
        .memberId(user.getId())
        .token(jwtToken)
        .expiresIn(expiration)
        .expiryDate(getLocalDateTime(expiryDate))
        .type(tokenType)
        .build();
  }
}
