package org.example.nareugobackend.domain.payment;

public enum OrderStatus {
    PAYMENT_COMPLETED, // 결제 완료
    IN_DELIVERY,       // 배송 중
    DELIVERY_COMPLETED,// 배송 완료
    CANCELLED          // 취소됨
}
