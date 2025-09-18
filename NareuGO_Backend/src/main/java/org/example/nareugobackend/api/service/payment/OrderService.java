package org.example.nareugobackend.api.service.payment;

import org.example.nareugobackend.api.controller.payment.response.OrderSummary;

/**
 * 주문 관련 비즈니스 로직을 처리하는 서비스 인터페이스
 *
 * 중고거래 플랫폼의 주문 생성, 조회, 자동 만료 등의 핵심 기능을 제공합니다.
 * 채팅방에서 결제 버튼을 누르면 주문이 생성되고, 15분 후 자동으로 만료됩니다.
 */
public interface OrderService {

    /**
     * 결제 대기 상태의 주문을 생성합니다.
     *
     * 채팅방에서 결제 버튼을 눌렀을 때 호출되며, 다음 과정을 수행합니다:
     * 1. 상품 정보 조회 및 가격 확인
     * 2. 동일 상품의 기존 결제 대기 주문 확인 (중복 방지)
     * 3. 15분 이상 된 기존 대기 주문 자동 만료 처리
     * 4. 새로운 주문을 PAYMENT_PENDING 상태로 생성
     *
     * @param productId 주문할 상품 ID
     * @param buyerId 구매자 ID
     * @return 생성된 주문 ID
     * @throws IllegalArgumentException 상품이 존재하지 않는 경우
     * @throws IllegalStateException 이미 결제 대기 중인 주문이 있는 경우
     */
    Long createPendingOrder(Long productId, Long buyerId);

    /**
     * 주문 정보를 조회하고 자동 만료 처리를 수행합니다.
     *
     * 결제 확인 페이지에서 호출되며, 다음 과정을 수행합니다:
     * 1. 주문 정보 조회
     * 2. PAYMENT_PENDING 상태이고 15분이 지났다면 CANCELLED로 변경
     * 3. 최종 주문 정보 반환
     *
     * @param orderId 조회할 주문 ID
     * @return 주문 요약 정보 (ID, 상품ID, 구매자ID, 상태, 금액)
     * @throws IllegalArgumentException 주문이 존재하지 않는 경우
     */
    OrderSummary getOrderAndAutoExpire(Long orderId);
}


