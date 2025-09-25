package org.example.nareugobackend.api.service.payment;

import org.example.nareugobackend.api.controller.payment.request.PaymentConfirmRequestDto;

/**
 * 결제 관련 비즈니스 로직을 처리하는 서비스 인터페이스
 *
 * 토스페이먼츠 API와 연동하여 실제 결제 승인을 처리합니다.
 * 결제 확인 페이지에서 "결제하기" 버튼을 눌렀을 때 호출됩니다.
 */
public interface PaymentService {

    /**
     * 토스페이먼츠 결제 승인을 처리합니다.
     *
     * 프론트엔드에서 토스 결제창을 통해 결제가 완료된 후 호출되며, 다음 과정을 수행합니다:
     * 1. 주문 정보 조회 및 금액 검증 (위변조 방지)
     * 2. 토스페이먼츠 API에 결제 승인 요청
     * 3. 토스 응답 검증 (상태, 금액 재확인)
     * 4. 주문 상태를 PAYMENT_COMPLETED로 변경
     * 5. 결제 정보를 DB에 저장
     *
     * 
     * @param requestDto 결제 승인 요청 데이터 (paymentKey, orderId, amount)
     * @throws IllegalArgumentException 주문이 존재하지 않거나 금액이 일치하지 않는 경우
     * @throws IllegalStateException 토스 결제 승인이 실패한 경우
     */
    void confirmPayment(PaymentConfirmRequestDto requestDto);

}
