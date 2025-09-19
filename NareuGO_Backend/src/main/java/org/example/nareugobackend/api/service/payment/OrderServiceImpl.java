package org.example.nareugobackend.api.service.payment;

import java.math.BigDecimal;
import java.time.Duration;
import java.time.LocalDateTime;
import java.util.List;
import java.util.UUID;
import java.util.stream.Collectors;
import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.common.model.Order;
import org.example.nareugobackend.common.model.OrderStatus;
import org.example.nareugobackend.mapper.OrderMapper;
import org.example.nareugobackend.mapper.ProductMapper;
import org.example.nareugobackend.api.controller.payment.response.OrderSummary;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

@Service
@RequiredArgsConstructor
public class OrderServiceImpl implements OrderService {
    private final OrderMapper orderMapper;
    private final ProductMapper productMapper; // 결제용 상품 정보 조회

    @Override
    @Transactional
    public Long createPendingOrder(Long productId, Long buyerId) {
        // 결제용 상품 가격 조회 (기존 코드와 분리)
        BigDecimal productPrice = productMapper.findProductPriceById(productId);
        if (productPrice == null) {
            throw new IllegalArgumentException("존재하지 않는 상품이거나 가격 정보가 없습니다.");
        }

        // 동일 상품 최근 주문 조회
        Order latest = orderMapper.findByProductId(productId).orElse(null);
        if (latest != null) {
            // 자동 만료 반영(읽기 시점 만료 규칙 재사용)
            if (latest.getStatus() == OrderStatus.PAYMENT_PENDING && latest.getCreatedAt() != null) {
                if (Duration.between(latest.getCreatedAt(), LocalDateTime.now()).toMinutes() >= 15) {
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
        order.setAmount(productPrice); // 실제 상품 가격 사용
        
        // 토스페이먼츠 규격에 맞는 orderId 생성 (영문 대소문자, 숫자, -, _ 허용, 6-64자)
        String tossOrderId = generateTossOrderId(productId, buyerId);
        order.setTossOrderId(tossOrderId);
        
        orderMapper.insert(order);
        return order.getOrderId();
    }

    @Override
    @Transactional
    public OrderSummary getOrderAndAutoExpire(Long orderId) {
        Order order = orderMapper.findById(orderId);
        if (order == null) {
            throw new IllegalArgumentException("주문을 찾을 수 없습니다: " + orderId);
        }

        // 자동 만료는 스케줄러에서 DB 시간 기준으로 처리합니다.

        return new OrderSummary(
            order.getOrderId(),
            order.getProductId(),
            order.getBuyerId(),
            order.getStatus().name(),
            order.getAmount(),
            order.getTossOrderId(),
            order.getSellerId(),
            order.getDeliveryStatus(),
            order.getProductTitle(),
            order.getBuyerNickname()
        );
    }

    @Override
    public OrderSummary getOrderByTossOrderId(String tossOrderId) {
        Order order = orderMapper.findByTossOrderId(tossOrderId);
        if (order == null) {
            throw new IllegalArgumentException("주문을 찾을 수 없습니다: " + tossOrderId);
        }

        return new OrderSummary(
            order.getOrderId(),
            order.getProductId(),
            order.getBuyerId(),
            order.getStatus().name(),
            order.getAmount(),
            order.getTossOrderId(),
            order.getSellerId(),
            order.getDeliveryStatus(),
            order.getProductTitle(),
            order.getBuyerNickname()
        );
    }

    @Override
    public List<OrderSummary> getOrdersBySeller(Long sellerId, String status) {
        List<Order> orders = orderMapper.findOrdersBySeller(sellerId, status);

        return orders.stream()
            .map(order -> new OrderSummary(
                order.getOrderId(),
                order.getProductId(),
                order.getBuyerId(),
                order.getStatus().name(),
                order.getAmount(),
                order.getTossOrderId(),
                order.getSellerId(),
                order.getDeliveryStatus(),
                order.getProductTitle(),
                order.getBuyerNickname()
            ))
            .collect(Collectors.toList());
    }

    // ===== 토스페이먼츠 규격 orderId 생성 (결제용) =====
    /**
     * 토스페이먼츠 규격에 맞는 orderId 생성
     * - 영문 대소문자, 숫자, 특수문자(-, _)만 허용
     * - 6자 이상 64자 이하
     */
    private String generateTossOrderId(Long productId, Long buyerId) {
        // 현재 시간을 밀리초로 변환하여 고유성 보장
        long timestamp = System.currentTimeMillis();
        
        // 형식: ORDER_productId_buyerId_timestamp (예: ORDER_18_5_1726644123456)
        String orderId = String.format("ORDER_%d_%d_%d", productId, buyerId, timestamp);
        
        // 64자 제한 확인 (초과 시 UUID 사용)
        if (orderId.length() > 64) {
            // UUID 사용 (하이픈 제거하여 32자)
            String uuid = UUID.randomUUID().toString().replace("-", "");
            orderId = "ORDER_" + uuid.substring(0, 26); // ORDER_ + 26자 = 32자
        }
        
        return orderId;
    }
}
