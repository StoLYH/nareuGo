package org.example.nareugobackend.api.controller.neighborhood;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.example.nareugobackend.api.service.neighborhood.NeighborhoodService;
import org.example.nareugobackend.api.service.neighborhood.dto.NeighborhoodVerificationRequest;
import org.example.nareugobackend.api.service.neighborhood.dto.NeighborhoodVerificationResponse;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.security.core.annotation.AuthenticationPrincipal;
import org.springframework.security.core.userdetails.UserDetails;
import org.springframework.web.bind.annotation.*;

@Slf4j
@RestController
@RequestMapping("/neighborhood")
@RequiredArgsConstructor
public class NeighborhoodController {

    private final NeighborhoodService neighborhoodService;

    /**
     * 동네 인증 정보를 저장합니다.
     * 
     * @param request 인증 정보 (GPS 위치, OCR 주소 등)
     * @param userDetails 현재 로그인한 사용자 정보
     * @return 인증 저장 결과
     */
    @PostMapping("/verify")
    public ResponseEntity<NeighborhoodVerificationResponse> verifyNeighborhood(
            @RequestBody NeighborhoodVerificationRequest request) {
        
        try {
            log.info("동네 인증 저장 요청 - GPS 주소: {}", request.getGpsAddress());
            
            // 개발 단계에서는 요청 본문의 userEmail 사용
            String userEmail = request.getUserEmail();
            if (userEmail == null || userEmail.isEmpty()) {
                NeighborhoodVerificationResponse errorResponse = new NeighborhoodVerificationResponse();
                errorResponse.setSuccess(false);
                errorResponse.setMessage("사용자 이메일이 필요합니다.");
                return ResponseEntity.badRequest().body(errorResponse);
            }
            
            // 동네 인증 정보 저장
            NeighborhoodVerificationResponse response = neighborhoodService.saveVerificationInfo(
                userEmail, request);
            
            if (response.isSuccess()) {
                log.info("동네 인증 저장 성공 - 사용자: {}", userEmail);
                return ResponseEntity.ok(response);
            } else {
                log.warn("동네 인증 저장 실패: {}", response.getMessage());
                return ResponseEntity.badRequest().body(response);
            }
            
        } catch (Exception e) {
            log.error("동네 인증 저장 중 오류 발생", e);
            
            NeighborhoodVerificationResponse errorResponse = new NeighborhoodVerificationResponse();
            errorResponse.setSuccess(false);
            errorResponse.setMessage("서버 오류가 발생했습니다. 잠시 후 다시 시도해주세요.");
            
            return ResponseEntity.internalServerError().body(errorResponse);
        }
    }

    /**
     * 사용자의 동네 인증 상태를 조회합니다.
     */
    @GetMapping("/status")
    public ResponseEntity<NeighborhoodVerificationResponse> getVerificationStatus(
            @RequestParam String userEmail) {
        
        try {
            if (userEmail == null || userEmail.isEmpty()) {
                NeighborhoodVerificationResponse errorResponse = new NeighborhoodVerificationResponse();
                errorResponse.setSuccess(false);
                errorResponse.setMessage("사용자 이메일이 필요합니다.");
                return ResponseEntity.badRequest().body(errorResponse);
            }
            
            NeighborhoodVerificationResponse response = neighborhoodService.getVerificationStatus(
                userEmail);
            
            return ResponseEntity.ok(response);
            
        } catch (Exception e) {
            log.error("동네 인증 상태 조회 중 오류 발생", e);
            
            NeighborhoodVerificationResponse errorResponse = new NeighborhoodVerificationResponse();
            errorResponse.setSuccess(false);
            errorResponse.setMessage("서버 오류가 발생했습니다.");
            
            return ResponseEntity.internalServerError().body(errorResponse);
        }
    }
}
