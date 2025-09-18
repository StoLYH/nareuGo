package org.example.nareugobackend.api.service.transaction;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.service.transaction.response.TransactionHistoryResponse;
import org.example.nareugobackend.domain.order.Order;
import org.example.nareugobackend.domain.order.OrderRepository;
import org.example.nareugobackend.domain.product.Product;
import org.example.nareugobackend.domain.product.ProductRepository;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.util.List;
import java.util.stream.Collectors;

@Service
@RequiredArgsConstructor
@Transactional(readOnly = true)
public class TransactionService {

    private final ProductRepository productRepository;
    private final OrderRepository orderRepository;

    // 판매내역 조회
    public List<TransactionHistoryResponse> getSalesHistory(Long sellerId) {
        List<Product> products = productRepository.findBySellerIdOrderByCreatedAtDesc(sellerId);

        return products.stream()
            .map(product -> {
                // 해당 상품의 주문 정보 조회
                List<Order> orders = orderRepository.findByProductIdOrderByCreatedAtDesc(product.getId());
                Order latestOrder = orders.isEmpty() ? null : orders.get(0);

                return TransactionHistoryResponse.builder()
                    .productId(product.getId())
                    .title(product.getTitle())
                    .price(product.getPrice())
                    .productStatus(product.getStatus().name())
                    .createdAt(product.getCreatedAt())
                    .orderId(latestOrder != null ? latestOrder.getId() : null)
                    .orderStatus(latestOrder != null ? latestOrder.getStatus().name() : null)
                    .orderCreatedAt(latestOrder != null ? latestOrder.getCreatedAt() : null)
                    .amount(latestOrder != null ? latestOrder.getAmount() : null)
                    .buyerNickname(latestOrder != null ? latestOrder.getBuyer().getNickname() : null)
                    .build();
            })
            .collect(Collectors.toList());
    }

    // 구매내역 조회
    public List<TransactionHistoryResponse> getPurchaseHistory(Long buyerId) {
        List<Order> orders = orderRepository.findByBuyerIdOrderByCreatedAtDesc(buyerId);

        return orders.stream()
            .map(order -> {
                // 해당 주문의 상품 정보 조회
                Product product = productRepository.findById(order.getProductId()).orElse(null);

                return TransactionHistoryResponse.builder()
                    .productId(order.getProductId())
                    .title(product != null ? product.getTitle() : "상품 정보 없음")
                    .price(product != null ? product.getPrice() : order.getAmount())
                    .productStatus(product != null ? product.getStatus().name() : "UNKNOWN")
                    .createdAt(product != null ? product.getCreatedAt() : null)
                    .orderId(order.getId())
                    .orderStatus(order.getStatus().name())
                    .orderCreatedAt(order.getCreatedAt())
                    .amount(order.getAmount())
                    .sellerNickname(product != null ? product.getSeller().getNickname() : null)
                    .build();
            })
            .collect(Collectors.toList());
    }
}