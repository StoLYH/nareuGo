package org.example.nareugobackend.api.controller.payment.response;

import java.math.BigDecimal;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

/**
 * 주문 요약 정보를 나타내는 응답 DTO
 * 
 * 서비스 계층에서 주문 정보를 조회할 때 사용되는 내부 DTO입니다.
 * 컨트롤러에서 OrderResponseDto로 변환되어 클라이언트에 전달됩니다.
 */
@Getter
@Setter
@NoArgsConstructor
public class OrderSummary {
    private Long orderId;           // 주문 ID
    private Long productId;         // 상품 ID
    private Long buyerId;           // 구매자 ID
    private String status;          // 주문 상태 (문자열)
    private BigDecimal amount;      // 주문 금액
    private String tossOrderId;     // 토스페이먼츠용 orderId
    private Long sellerId;          // 판매자 ID
    private String deliveryStatus;  // 배송 상태
    private String productTitle;    // 상품명
    private String buyerNickname;   // 구매자 닉네임
    private String buyerName;       // 구매자 이름

    public OrderSummary(Long orderId, Long productId, Long buyerId, String status, BigDecimal amount,
                       String tossOrderId, Long sellerId, String deliveryStatus, String productTitle,
                       String buyerNickname) {
        this.orderId = orderId;
        this.productId = productId;
        this.buyerId = buyerId;
        this.status = status;
        this.amount = amount;
        this.tossOrderId = tossOrderId;
        this.sellerId = sellerId;
        this.deliveryStatus = deliveryStatus;
        this.productTitle = productTitle;
        this.buyerNickname = buyerNickname;
    }

    public OrderSummary(Long orderId, Long productId, Long buyerId, String status, BigDecimal amount,
                       String tossOrderId, Long sellerId, String deliveryStatus, String productTitle,
                       String buyerNickname, String buyerName) {
        this.orderId = orderId;
        this.productId = productId;
        this.buyerId = buyerId;
        this.status = status;
        this.amount = amount;
        this.tossOrderId = tossOrderId;
        this.sellerId = sellerId;
        this.deliveryStatus = deliveryStatus;
        this.productTitle = productTitle;
        this.buyerNickname = buyerNickname;
        this.buyerName = buyerName;
    }
}
