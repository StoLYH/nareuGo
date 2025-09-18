package org.example.nareugobackend.api.service.verification.request;

import lombok.Getter;
import lombok.NoArgsConstructor;
import org.example.nareugobackend.domain.verification.NeighborhoodVerification;

@Getter
@NoArgsConstructor
public class VerificationSubmitRequest {
    private NeighborhoodVerification.VerificationType verificationType;
    private String verificationData;
}