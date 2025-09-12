package org.example.nareugobackend.api.service.payment;

import java.math.BigDecimal;
import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.domain.payment.Order;
import org.example.nareugobackend.domain.payment.OrderMapper;
import org.example.nareugobackend.domain.payment.OrderStatus;
import org.example.nareugobackend.domain.product.Product;
import org.example.nareugobackend.domain.product.ProductMapper;
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

        Order order = new Order();
        order.setProductId(productId);
        order.setBuyerId(buyerId);
        order.setStatus(OrderStatus.PAYMENT_PENDING);
        order.setAmount(product.getPrice());
        orderMapper.insert(order);
        return order.getOrderId();
    }
}


