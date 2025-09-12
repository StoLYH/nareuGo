package org.example.nareugobackend.api.service.payment;

import java.math.BigDecimal;
import java.time.Duration;
import java.time.LocalDateTime;
import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.domain.payment.Order;
import org.example.nareugobackend.domain.payment.OrderMapper;
import org.example.nareugobackend.domain.payment.OrderStatus;
import org.example.nareugobackend.domain.product.Product;
import org.example.nareugobackend.domain.product.ProductMapper;
import org.example.nareugobackend.api.service.payment.response.OrderSummary;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

@Service
@RequiredArgsConstructor
public class OrderServiceImpl implements OrderService {

    private final ProductMapper productMapper;
    private final OrderMapper orderMapper;

    @Override
    @Transactional
    public Long createPendingOrder(Long productId, Long buyerId) {
        Product product = productMapper.findById(productId)
            .orElseThrow(() -> new IllegalArgumentException("존재하지 않는 상품입니다."));

        if (product.getPrice() == null) {
            throw new IllegalStateException("상품 가격이 없습니다.");
        }

        // 동일 상품 최근 주문 조회
        Order latest = orderMapper.findByProductId(productId).orElse(null);
        if (latest != null) {
            // 자동 만료 반영(읽기 시점 만료 규칙 재사용)
            if (latest.getStatus() == OrderStatus.PAYMENT_PENDING && latest.getCreatedAt() != null) {
                if (java.time.Duration.between(latest.getCreatedAt(), java.time.LocalDateTime.now()).toMinutes() >= 15) {
                    orderMapper.updateStatus(latest.getOrderId(), OrderStatus.CANCELLED);
                    latest.setStatus(OrderStatus.CANCELLED);
                }
            }
            // 아직 유효한 대기가 있으면 차단
            if (latest.getStatus() == OrderStatus.PAYMENT_PENDING) {
                throw new IllegalStateException("이미 결제 대기 중인 주문이 있습니다.");
            }
        }

        Order order = new Order();
        order.setProductId(productId);
        order.setBuyerId(buyerId);
        order.setStatus(OrderStatus.PAYMENT_PENDING);
        order.setAmount(product.getPrice());
        orderMapper.insert(order);
        return order.getOrderId();
    }

    @Override
    @Transactional
    public OrderSummary getOrderAndAutoExpire(Long orderId) {
        Order order = orderMapper.findById(String.valueOf(orderId))
            .orElseThrow(() -> new IllegalArgumentException("존재하지 않는 주문입니다."));

        // 결제 대기 15분 초과 시 자동 취소
        if (order.getStatus() == OrderStatus.PAYMENT_PENDING && order.getCreatedAt() != null) {
            if (Duration.between(order.getCreatedAt(), LocalDateTime.now()).toMinutes() >= 15) {
                orderMapper.updateStatus(order.getOrderId(), OrderStatus.CANCELLED);
                order.setStatus(OrderStatus.CANCELLED);
            }
        }

        OrderSummary summary = new OrderSummary();
        summary.setOrderId(order.getOrderId());
        summary.setProductId(order.getProductId());
        summary.setBuyerId(order.getBuyerId());
        summary.setStatus(order.getStatus().name());
        summary.setAmount(order.getAmount());
        return summary;
    }
}


