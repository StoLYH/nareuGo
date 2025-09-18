package org.example.nareugobackend.domain.verification;

import static jakarta.persistence.EnumType.STRING;
import static jakarta.persistence.GenerationType.IDENTITY;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.Enumerated;
import jakarta.persistence.FetchType;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.Id;
import jakarta.persistence.JoinColumn;
import jakarta.persistence.ManyToOne;
import jakarta.persistence.Table;
import java.time.Instant;
import lombok.AccessLevel;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;
import org.example.nareugobackend.domain.user.User;

@Getter
@Entity
@Builder
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@AllArgsConstructor
@Table(name = "neighborhood_verification")
public class NeighborhoodVerification {

    @Id
    @GeneratedValue(strategy = IDENTITY)
    @Column(name = "verification_id")
    private Long id;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "user_id", nullable = false)
    private User user;

    @Enumerated(STRING)
    @Column(name = "verification_type", nullable = false)
    private VerificationType verificationType;

    @Column(name = "verification_data", columnDefinition = "TEXT")
    private String verificationData;

    @Enumerated(STRING)
    @Column(name = "status", nullable = false)
    @Builder.Default
    private VerificationStatus status = VerificationStatus.PENDING;

    @Column(name = "submitted_at", nullable = false)
    @Builder.Default
    private Instant submittedAt = Instant.now();

    @Column(name = "processed_at")
    private Instant processedAt;

    @Column(name = "admin_notes", columnDefinition = "TEXT")
    private String adminNotes;

    public enum VerificationType {
        PHOTO, DOCUMENT, MANUAL
    }

    public enum VerificationStatus {
        PENDING, APPROVED, REJECTED
    }

    public static NeighborhoodVerification create(User user, VerificationType type, String data) {
        return NeighborhoodVerification.builder()
            .user(user)
            .verificationType(type)
            .verificationData(data)
            .build();
    }

    public void approve(String adminNotes) {
        this.status = VerificationStatus.APPROVED;
        this.processedAt = Instant.now();
        this.adminNotes = adminNotes;
    }

    public void reject(String adminNotes) {
        this.status = VerificationStatus.REJECTED;
        this.processedAt = Instant.now();
        this.adminNotes = adminNotes;
    }
}