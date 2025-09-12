package org.example.nareugobackend.domain.payment;

/**
 * 주문 상태를 나타내는 열거형
 * 
 * 주문의 생명주기를 관리하며, 각 상태는 다음과 같은 의미를 가집니다:
 * - PAYMENT_PENDING: 채팅방에서 결제 버튼을 눌러 주문이 생성된 상태 (15분 후 자동 취소)
 * - PAYMENT_COMPLETED: 토스페이먼츠 결제가 성공적으로 완료된 상태
 * - IN_DELIVERY: 결제 완료 후 로봇 배송이 시작된 상태
 * - DELIVERY_COMPLETED: 배송이 완료된 상태
 * - CANCELLED: 결제 대기 시간 초과 또는 사용자/시스템에 의한 취소 상태
 */
public enum OrderStatus {
    PAYMENT_PENDING,   // 결제 대기 (15분 자동 만료)
    PAYMENT_COMPLETED, // 결제 완료
    IN_DELIVERY,       // 배송 중
    DELIVERY_COMPLETED,// 배송 완료
    CANCELLED          // 취소됨
}
