package org.example.nareugobackend.api.service.verification.response;

import lombok.Builder;
import lombok.Getter;

@Getter
@Builder
public class VerificationResponse {
    private boolean success;
    private String message;
    private Long verificationId;
}