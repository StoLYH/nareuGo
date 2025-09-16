package org.example.nareugobackend.api.service.ocr.dto;

import lombok.Data;

@Data
public class OcrRequestDto {
    /**
     * Base64로 인코딩된 이미지 데이터
     */
    private String imageData;
    
    /**
     * 이미지 파일 형식 (jpg, png 등)
     */
    private String imageFormat;
    
    /**
     * GPS로 확인된 현재 위치 (비교용)
     */
    private Double latitude;
    private Double longitude;
    private String gpsAddress;
}
