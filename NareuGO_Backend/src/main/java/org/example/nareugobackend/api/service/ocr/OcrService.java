package org.example.nareugobackend.api.service.ocr;

import org.example.nareugobackend.api.service.ocr.dto.OcrRequestDto;
import org.example.nareugobackend.api.service.ocr.dto.OcrResponseDto;

public interface OcrService {
    /**
     * 신분증 이미지에서 주소 정보를 추출합니다.
     * @param requestDto 이미지 데이터와 요청 정보
     * @return OCR 결과 및 추출된 주소 정보
     */
    OcrResponseDto extractAddressFromIdCard(OcrRequestDto requestDto);
}
