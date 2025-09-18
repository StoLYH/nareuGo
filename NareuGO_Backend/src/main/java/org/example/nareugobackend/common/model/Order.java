package org.example.nareugobackend.common.model;

import java.math.BigDecimal;
import java.time.LocalDateTime;
import lombok.Getter;
import lombok.Setter;

/**
 * 주문 정보를 나타내는 MyBatis 엔티티
 * 
 * 중고거래에서 구매자가 상품을 주문할 때 생성되는 엔티티입니다.
 * 채팅방에서 결제 버튼을 누르면 PAYMENT_PENDING 상태로 생성되며,
 * 결제 완료 시 PAYMENT_COMPLETED로 상태가 변경됩니다.
 * 
 * MyBatis 매퍼를 통해 orders 테이블과 매핑됩니다.
 * 
 * 주요 필드:
 * - orderId: 주문 고유 ID (DB AUTO_INCREMENT)
 * - productId: 주문된 상품 ID
 * - buyerId: 구매자 ID
 * - status: 주문 상태 (PAYMENT_PENDING, PAYMENT_COMPLETED 등)
 * - amount: 주문 금액 (상품 가격 기준으로 고정)
 * - createdAt: 주문 생성 시각
 * - updatedAt: 상태 변경 시각
 */
@Getter
@Setter
public class Order {

    private Long orderId;           // 주문 고유 ID (DB 자동 생성)
    private Long productId;         // 주문된 상품 ID
    private Long buyerId;           // 구매자 ID
    private OrderStatus status;     // 주문 상태
    private BigDecimal amount;      // 주문 금액 (결제 시점의 상품 가격)
    private String tossOrderId;     // 토스페이먼츠용 orderId (영문+숫자+특수문자, 6-64자)
    private LocalDateTime createdAt; // 주문 생성 시각
    private LocalDateTime updatedAt; // 상태 변경 시각
}
