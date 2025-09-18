package org.example.nareugobackend.domain.delivery;

public enum DeliveryStatus {
    RECEIPT_COMPLETED,  // 접수완료
    DELIVERY_BEGIN,     // 배달시작
    IN_DELIVERY,        // 배달중
    DELIVERY_COMPLETED, // 완료
    CANCELLED           // 취소
}