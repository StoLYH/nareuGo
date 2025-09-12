package org.example.nareugobackend.api.controller.payment;

import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.controller.payment.request.PaymentConfirmRequestDto;
import org.example.nareugobackend.api.service.payment.PaymentService;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

/**
 * 결제 관련 REST API 컨트롤러
 * 
 * 토스페이먼츠 API와 연동하여 실제 결제 승인을 처리합니다.
 * 결제 확인 페이지에서 "결제하기" 버튼을 눌렀을 때 호출됩니다.
 */
@RestController
@RequestMapping("/api/payments")
@RequiredArgsConstructor
public class PaymentController {

    private final PaymentService paymentService;

    /**
     * 토스페이먼츠 결제 승인을 처리합니다.
     * 
     * 프론트엔드에서 토스 결제창을 통해 결제가 완료된 후 호출되는 API입니다.
     * 다음 과정을 수행합니다:
     * 1. 주문 정보 조회 및 금액 검증 (위변조 방지)
     * 2. 토스페이먼츠 API에 결제 승인 요청
     * 3. 토스 응답 검증 (상태, 금액 재확인)
     * 4. 주문 상태를 PAYMENT_COMPLETED로 변경
     * 5. 결제 정보를 DB에 저장
     * 
     * @param requestDto 결제 승인 요청 데이터 (paymentKey, orderId, amount)
     * @return 204 No Content (성공 시)
     */
    @PostMapping("/confirm")
    public ResponseEntity<Void> confirmPayment(@Valid @RequestBody PaymentConfirmRequestDto requestDto) {
        paymentService.confirmPayment(requestDto);
        return ResponseEntity.noContent().build();
    }
}
