package org.example.nareugobackend.api.service.ocr.dto;

import lombok.Data;
import java.util.List;

@Data
public class OcrResponseDto {
    /**
     * OCR 처리 성공 여부
     */
    private boolean success;
    
    /**
     * 추출된 주소 정보
     */
    private String extractedAddress;
    
    /**
     * 주소 구성 요소
     */
    private AddressComponents addressComponents;
    
    /**
     * GPS 주소 (비교용)
     */
    private String gpsAddress;
    
    /**
     * GPS 주소와의 일치 여부
     */
    private boolean addressMatched;
    
    /**
     * 일치도 점수 (0.0 ~ 1.0)
     */
    private double matchScore;
    
    /**
     * 오류 메시지 (실패 시)
     */
    private String errorMessage;
    
    /**
     * OCR로 추출된 모든 텍스트 (디버깅용)
     */
    private List<String> allExtractedTexts;
    
    @Data
    public static class AddressComponents {
        private String sido;        // 시/도
        private String sigungu;     // 시/군/구
        private String dong;        // 동/면/읍
        private String detail;      // 상세주소
        private String postalCode;  // 우편번호
        
        public void setSido(String sido) { this.sido = sido; }
        public void setSigungu(String sigungu) { this.sigungu = sigungu; }
        public void setDong(String dong) { this.dong = dong; }
        public void setDetail(String detail) { this.detail = detail; }
        public void setPostalCode(String postalCode) { this.postalCode = postalCode; }
        
        public String getSido() { return sido; }
        public String getSigungu() { return sigungu; }
        public String getDong() { return dong; }
        public String getDetail() { return detail; }
        public String getPostalCode() { return postalCode; }
    }
}
