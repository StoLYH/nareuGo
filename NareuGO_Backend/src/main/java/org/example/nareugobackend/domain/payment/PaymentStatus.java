package org.example.nareugobackend.domain.payment;

/**
 * 결제 상태를 나타내는 열거형
 * 
 * 토스페이먼츠 API와 연동하여 결제의 최종 상태를 관리합니다:
 * - DONE: 토스페이먼츠에서 결제가 성공적으로 승인된 상태
 * - CANCELED: 토스페이먼츠에서 결제가 취소된 상태 (부분 취소 포함)
 */
public enum PaymentStatus {
    DONE,     // 결제 완료 (토스 승인 성공)
    CANCELED  // 결제 취소 (토스 취소 완료)
}
