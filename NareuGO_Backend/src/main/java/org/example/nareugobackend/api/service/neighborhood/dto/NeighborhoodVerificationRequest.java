package org.example.nareugobackend.api.service.neighborhood.dto;

import lombok.Data;

@Data
public class NeighborhoodVerificationRequest {
    /**
     * GPS 위도
     */
    private Double latitude;
    
    /**
     * GPS 경도
     */
    private Double longitude;
    
    /**
     * GPS로 확인된 주소
     */
    private String gpsAddress;
    
    /**
     * OCR로 추출된 주소
     */
    private String ocrAddress;
    
    /**
     * 주소 일치도 점수 (0.0 ~ 1.0)
     */
    private Double matchScore;
    
    /**
     * 주소 일치 여부
     */
    private boolean addressMatched;
    
    /**
     * 사용자 이메일 (개발 단계용)
     */
    private String userEmail;
    
    /**
     * 아파트명
     */
    private String apartmentName;
    
    /**
     * 동 번호
     */
    private Integer buildingDong;
    
    /**
     * 호 번호
     */
    private Integer buildingHo;
}
