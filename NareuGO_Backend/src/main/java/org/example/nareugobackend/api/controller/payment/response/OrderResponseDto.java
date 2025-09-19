package org.example.nareugobackend.api.controller.payment.response;

import java.math.BigDecimal;
import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class OrderResponseDto {
    private Long orderId;
    private Long productId;
    private Long buyerId;
    private String status;
    private BigDecimal amount;
    private String tossOrderId;     // 토스페이먼츠용 orderId
    private Long sellerId;          // 판매자 ID
    private String deliveryStatus;  // 배송 상태
    private String productTitle;    // 상품명
    private String buyerNickname;   // 구매자 닉네임
    private String buyerName;       // 구매자 이름
}


