package org.example.nareugobackend.api.service.neighborhood;

import org.example.nareugobackend.api.service.neighborhood.dto.NeighborhoodVerificationRequest;
import org.example.nareugobackend.api.service.neighborhood.dto.NeighborhoodVerificationResponse;

public interface NeighborhoodService {
    /**
     * 동네 인증 정보를 저장합니다.
     * @param userEmail 사용자 이메일
     * @param request 인증 정보
     * @return 저장 결과
     */
    NeighborhoodVerificationResponse saveVerificationInfo(String userEmail, NeighborhoodVerificationRequest request);
    
    /**
     * 사용자의 동네 인증 상태를 조회합니다.
     * @param userEmail 사용자 이메일
     * @return 인증 상태 정보
     */
    NeighborhoodVerificationResponse getVerificationStatus(String userEmail);
}
