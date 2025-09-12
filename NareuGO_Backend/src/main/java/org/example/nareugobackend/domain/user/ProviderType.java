package org.example.nareugobackend.domain.user;

import org.example.nareugobackend.common.exception.auth.AuthException;
import java.util.Arrays;

public enum ProviderType {
  KAKAO, NAVER;

  public static ProviderType convert(String registrationId) {

    return Arrays.stream(ProviderType.values())
        .filter(providerType -> providerType.name().equalsIgnoreCase(registrationId))
        .findFirst()
        .orElseThrow(() -> new AuthException("auth error"));
  }
}