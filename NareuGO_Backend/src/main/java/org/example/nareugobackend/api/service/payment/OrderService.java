package org.example.nareugobackend.api.service.payment;

public interface OrderService {
    Long createPendingOrder(Long productId, Long buyerId);
}


