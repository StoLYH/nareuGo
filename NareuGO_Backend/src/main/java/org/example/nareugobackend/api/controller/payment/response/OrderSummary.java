package org.example.nareugobackend.api.controller.payment.response;

import java.math.BigDecimal;
import lombok.Getter;
import lombok.Setter;

/**
 * 주문 요약 정보를 나타내는 응답 DTO
 * 
 * 서비스 계층에서 주문 정보를 조회할 때 사용되는 내부 DTO입니다.
 * 컨트롤러에서 OrderResponseDto로 변환되어 클라이언트에 전달됩니다.
 */
@Getter
@Setter
public class OrderSummary {
    private Long orderId;           // 주문 ID
    private Long productId;         // 상품 ID
    private Long buyerId;           // 구매자 ID
    private String status;          // 주문 상태 (문자열)
    private BigDecimal amount;      // 주문 금액
}
