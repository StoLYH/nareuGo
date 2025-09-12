package org.example.nareugobackend.common.model;

import java.math.BigDecimal;
import java.time.LocalDateTime;
import lombok.Getter;
import lombok.Setter;

/**
 * 결제 정보를 나타내는 MyBatis 엔티티
 * 
 * 토스페이먼츠 API를 통해 실제 결제가 승인된 후 생성되는 엔티티입니다.
 * 주문(Order)과 1:1 관계를 가지며, 토스페이먼츠의 결제 고유 키와 승인 정보를 저장합니다.
 * 
 * MyBatis 매퍼를 통해 payments 테이블과 매핑됩니다.
 * 
 * 주요 필드:
 * - paymentId: 결제 고유 ID (DB AUTO_INCREMENT)
 * - orderId: 연관된 주문 ID
 * - paymentKey: 토스페이먼츠 결제 고유 키 (중복 방지용)
 * - amount: 실제 결제된 금액
 * - status: 결제 상태 (DONE, CANCELED)
 * - approvedAt: 토스페이먼츠에서 승인된 시각
 */
@Getter
@Setter
public class Payment {

    private Long paymentId;         // 결제 고유 ID (DB 자동 생성)
    private Long orderId;           // 연관된 주문 ID
    private String paymentKey;      // 토스페이먼츠 결제 고유 키
    private BigDecimal amount;      // 실제 결제된 금액
    private PaymentStatus status;   // 결제 상태
    private LocalDateTime approvedAt; // 토스페이먼츠 승인 시각
}
