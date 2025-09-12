package org.example.nareugobackend.common.jwt;

import lombok.Builder;

@Builder
public record AuthToken(
    Long memberId,
    String token,
    Integer expiresIn,
    String expiryDate,
    AuthTokenType type
) {

}