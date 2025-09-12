package org.example.nareugobackend.domain.user;

public enum Role {
  USER, SELLER;

  public static String toAuthority(Role role) {
    if (role == null) {
      throw new IllegalArgumentException("역할이 정해지지 않았습니다.");
    }
    return "ROLE_" + role.name();
  }
}
