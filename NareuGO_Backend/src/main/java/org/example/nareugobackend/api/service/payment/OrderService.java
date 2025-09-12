package org.example.nareugobackend.api.service.payment;

import org.example.nareugobackend.api.service.payment.response.OrderSummary;

public interface OrderService {
    Long createPendingOrder(Long productId, Long buyerId);

    OrderSummary getOrderAndAutoExpire(Long orderId);
}


