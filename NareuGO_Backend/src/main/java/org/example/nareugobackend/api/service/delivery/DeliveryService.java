package org.example.nareugobackend.api.service.delivery;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.controller.delivery.response.DeliveryResponse;
import org.example.nareugobackend.domain.delivery.Delivery;
import org.example.nareugobackend.domain.delivery.DeliveryRepository;
import org.example.nareugobackend.domain.product.Product;
import org.example.nareugobackend.domain.product.ProductRepository;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.util.List;
import java.util.stream.Collectors;

@Service
@RequiredArgsConstructor
@Transactional(readOnly = true)
public class DeliveryService {

    private final DeliveryRepository deliveryRepository;
    private final ProductRepository productRepository;

    public List<DeliveryResponse> getUserDeliveries(Long userId) {
        List<Delivery> deliveries = deliveryRepository.findDeliveriesByUserId(userId);

        return deliveries.stream()
                .map(this::convertToResponse)
                .collect(Collectors.toList());
    }

    private DeliveryResponse convertToResponse(Delivery delivery) {
        // Product title 가져오기
        String productTitle = "나르고"; // 기본값
        if (delivery.getOrder().getProductId() != null) {
            Product product = productRepository.findById(delivery.getOrder().getProductId()).orElse(null);
            if (product != null && product.getTitle() != null) {
                productTitle = product.getTitle();
            }
        }

        return DeliveryResponse.builder()
                .deliveryId(delivery.getId())
                .orderId(delivery.getOrder().getId())
                .robotId(null) // TODO: 로봇 ID 연결 필요
                .status(delivery.getStatus())
                .requestTime(delivery.getCreatedAt())
                .completeTime(delivery.getActualDeliveryTime())
                .trackingNumber(delivery.getTrackingNumber())
                .title(productTitle)
                .build();
    }
}