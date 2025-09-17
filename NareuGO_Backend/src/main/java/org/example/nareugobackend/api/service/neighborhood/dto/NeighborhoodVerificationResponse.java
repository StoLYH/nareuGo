package org.example.nareugobackend.api.service.neighborhood.dto;

import lombok.Data;
import java.time.LocalDateTime;

@Data
public class NeighborhoodVerificationResponse {
    /**
     * 처리 성공 여부
     */
    private boolean success;
    
    /**
     * 응답 메시지
     */
    private String message;
    
    /**
     * 동네 인증 완료 여부
     */
    private boolean verified;
    
    /**
     * 인증된 위치 (GPS 주소)
     */
    private String verifiedLocation;
    
    /**
     * 인증된 주소 (OCR 주소)
     */
    private String verifiedAddress;
    
    /**
     * 인증 완료 일시
     */
    private LocalDateTime verificationDate;
}
