package org.example.nareugobackend.api.controller.ocr;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.example.nareugobackend.api.service.ocr.OcrService;
import org.example.nareugobackend.api.service.ocr.dto.OcrRequestDto;
import org.example.nareugobackend.api.service.ocr.dto.OcrResponseDto;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

@Slf4j
@RestController
@RequestMapping("/api/ocr")
@RequiredArgsConstructor
public class OcrController {

    private final OcrService ocrService;

    /**
     * 신분증 이미지에서 주소 정보를 추출하여 동네 인증을 수행합니다.
     * 
     * @param requestDto 이미지 데이터와 GPS 위치 정보
     * @return OCR 결과 및 주소 일치 여부
     */
    @PostMapping("/verify-address")
    public ResponseEntity<OcrResponseDto> verifyAddress(@RequestBody OcrRequestDto requestDto) {
        try {
            log.info("주소 인증 요청 수신 - GPS 주소: {}", requestDto.getGpsAddress());
            
            // 입력 검증
            if (requestDto.getImageData() == null || requestDto.getImageData().isEmpty()) {
                OcrResponseDto errorResponse = new OcrResponseDto();
                errorResponse.setSuccess(false);
                errorResponse.setErrorMessage("이미지 데이터가 필요합니다.");
                return ResponseEntity.badRequest().body(errorResponse);
            }
            
            // OCR 처리
            OcrResponseDto response = ocrService.extractAddressFromIdCard(requestDto);
            
            if (response.isSuccess()) {
                log.info("주소 인증 성공 - 추출된 주소: {}, 일치도: {}", 
                    response.getExtractedAddress(), response.getMatchScore());
                return ResponseEntity.ok(response);
            } else {
                log.warn("주소 인증 실패: {}", response.getErrorMessage());
                return ResponseEntity.ok(response); // 실패도 200으로 반환하되 success=false
            }
            
        } catch (Exception e) {
            log.error("주소 인증 처리 중 오류 발생", e);
            
            OcrResponseDto errorResponse = new OcrResponseDto();
            errorResponse.setSuccess(false);
            errorResponse.setErrorMessage("서버 오류가 발생했습니다. 잠시 후 다시 시도해주세요.");
            
            return ResponseEntity.internalServerError().body(errorResponse);
        }
    }

    /**
     * OCR 서비스 상태 확인용 헬스체크 엔드포인트
     */
    @GetMapping("/health")
    public ResponseEntity<String> healthCheck() {
        return ResponseEntity.ok("OCR Service is running");
    }
}
