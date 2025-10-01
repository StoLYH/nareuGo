package org.example.nareugobackend.domain.user;

import static jakarta.persistence.EnumType.STRING;
import static jakarta.persistence.GenerationType.IDENTITY;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.Enumerated;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.Id;
import jakarta.persistence.Table;
import java.time.Instant;
import lombok.AccessLevel;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@Entity
@Builder
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@AllArgsConstructor
@Table(name = "users")
public class User {

  @Id
  @GeneratedValue(strategy = IDENTITY)
  @Column(name = "user_id")
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

  private Integer buildingDong;

  private Integer buildingHo;

  private String apartmentName;

  private String siDo;

  private String siGunGu;

  private String eupMyeonDong;

  private Instant deletedAt;

  @Enumerated(STRING)
  private Role role;
  
}
