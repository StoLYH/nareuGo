package org.example.nareugobackend.domain.user;

import static jakarta.persistence.EnumType.STRING;
import static jakarta.persistence.GenerationType.IDENTITY;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.Enumerated;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.Id;
import java.time.Instant;
import lombok.AccessLevel;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;
import org.example.nareugobackend.api.service.auth.info.SocialUserInfo;

@Getter
@Entity
@Builder
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@AllArgsConstructor
public class User {

  @Id
  @GeneratedValue(strategy = IDENTITY)
  private Long id;

  @Column(unique = true, nullable = false)
  private String email;

  @Column(nullable = false)
  private String name;

  @Enumerated(STRING)
  private Sex sex;

  private String birth;

  private String phoneNumber;

  @Enumerated(STRING)
  private ProviderType providerType;

  private String providerId;

  @Builder.Default
  private Boolean isActive = true;

  private String nickname;

  private Integer building_dong;

  private Integer building_ho;

  private String apartment_name;

  private String si_do;

  private String si_gun_gu;

  private String eup_myeon_dong;

  private Instant deletedAt;

  @Enumerated(STRING)
  private Role role;


  public static User from(SocialUserInfo userInfo) {
    return User.builder()
        .email(userInfo.getEmail())
        .name(userInfo.getName())
        .sex(userInfo.getGender())
        .birth(userInfo.getBirth())
        .phoneNumber(userInfo.getPhoneNumber())
        .providerType(userInfo.getProviderType())
        .providerId(userInfo.getProviderId())
        .isActive(true)
        .role(Role.USER)
        .build();
  }
  
}
