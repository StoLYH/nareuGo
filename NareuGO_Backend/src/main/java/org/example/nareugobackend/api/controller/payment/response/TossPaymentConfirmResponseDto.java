package org.example.nareugobackend.api.controller.payment.response;

import java.time.OffsetDateTime;
import lombok.Getter;
import lombok.Setter;

/**
 * 토스페이먼츠 결제 승인 API 응답 DTO
 * 
 * 토스페이먼츠 API에서 결제 승인 응답을 받을 때 사용되는 DTO입니다.
 * 서비스 계층에서 토스 API 응답을 파싱하고 검증하는 데 사용됩니다.
 * 
 * 주요 필드:
 * - paymentKey: 토스페이먼츠 결제 고유 키
 * - orderId: 주문 ID
 * - status: 결제 상태 (DONE, CANCELED 등)
 * - totalAmount: 총 결제 금액
 * - approvedAt: 승인 시각
 */
@Getter
@Setter
public class TossPaymentConfirmResponseDto {

    private String paymentKey;      // 토스페이먼츠 결제 고유 키
    private String orderId;         // 주문 ID
    private String status;          // 결제 상태
    private Integer totalAmount;    // 총 결제 금액
    private OffsetDateTime approvedAt; // 승인 시각
}
