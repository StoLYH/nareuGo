package org.example.nareugobackend.api.service.verification;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.service.verification.request.VerificationSubmitRequest;
import org.example.nareugobackend.api.service.verification.response.VerificationResponse;
import org.example.nareugobackend.api.service.verification.response.VerificationStatusResponse;
import org.example.nareugobackend.domain.verification.NeighborhoodVerification;
import org.example.nareugobackend.domain.verification.NeighborhoodVerificationRepository;
import org.example.nareugobackend.domain.user.User;
import org.example.nareugobackend.domain.user.UserRepository;
import org.example.nareugobackend.common.exception.user.UserException;
import org.example.nareugobackend.common.exception.user.UserErrorCode;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

@Service
@RequiredArgsConstructor
@Transactional(readOnly = true)
public class NeighborhoodVerificationService {

    private final NeighborhoodVerificationRepository verificationRepository;
    private final UserRepository userRepository;

    @Transactional
    public VerificationResponse submitVerification(Long userId, VerificationSubmitRequest request) {
        User user = userRepository.findById(userId)
            .orElseThrow(() -> new UserException(UserErrorCode.USER_NOT_FOUND));

        // 이미 승인된 인증이 있는지 확인
        if (verificationRepository.existsByUserIdAndStatus(userId, NeighborhoodVerification.VerificationStatus.APPROVED)) {
            return VerificationResponse.builder()
                .success(false)
                .message("이미 동네 인증이 완료되었습니다.")
                .build();
        }

        // 대기 중인 인증이 있는지 확인
        if (verificationRepository.existsByUserIdAndStatus(userId, NeighborhoodVerification.VerificationStatus.PENDING)) {
            return VerificationResponse.builder()
                .success(false)
                .message("이미 동네 인증 신청이 진행 중입니다.")
                .build();
        }

        NeighborhoodVerification verification = NeighborhoodVerification.create(
            user,
            request.getVerificationType(),
            request.getVerificationData()
        );

        verificationRepository.save(verification);

        return VerificationResponse.builder()
            .success(true)
            .message("동네 인증 신청이 접수되었습니다.")
            .verificationId(verification.getId())
            .build();
    }

    public VerificationStatusResponse getVerificationStatus(Long userId) {
        Optional<NeighborhoodVerification> verification = verificationRepository
            .findTopByUserIdOrderBySubmittedAtDesc(userId);

        if (verification.isEmpty()) {
            return VerificationStatusResponse.builder()
                .hasVerification(false)
                .status(null)
                .message("동네 인증 신청 내역이 없습니다.")
                .build();
        }

        NeighborhoodVerification ver = verification.get();
        return VerificationStatusResponse.builder()
            .hasVerification(true)
            .status(ver.getStatus())
            .submittedAt(ver.getSubmittedAt())
            .processedAt(ver.getProcessedAt())
            .adminNotes(ver.getAdminNotes())
            .message(getStatusMessage(ver.getStatus()))
            .build();
    }

    public List<VerificationResponse> getUserVerificationHistory(Long userId) {
        List<NeighborhoodVerification> verifications = verificationRepository
            .findByUserIdOrderBySubmittedAtDesc(userId);

        return verifications.stream()
            .map(ver -> VerificationResponse.builder()
                .success(ver.getStatus() == NeighborhoodVerification.VerificationStatus.APPROVED)
                .message(getStatusMessage(ver.getStatus()))
                .verificationId(ver.getId())
                .build())
            .collect(Collectors.toList());
    }

    private String getStatusMessage(NeighborhoodVerification.VerificationStatus status) {
        return switch (status) {
            case PENDING -> "인증 심사가 진행 중입니다.";
            case APPROVED -> "동네 인증이 완료되었습니다.";
            case REJECTED -> "동네 인증이 거절되었습니다.";
        };
    }
}