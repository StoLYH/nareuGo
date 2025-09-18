package org.example.nareugobackend.api.service.verification.response;

import lombok.Builder;
import lombok.Getter;
import org.example.nareugobackend.domain.verification.NeighborhoodVerification;

import java.time.Instant;

@Getter
@Builder
public class VerificationStatusResponse {
    private boolean hasVerification;
    private NeighborhoodVerification.VerificationStatus status;
    private Instant submittedAt;
    private Instant processedAt;
    private String adminNotes;
    private String message;
}