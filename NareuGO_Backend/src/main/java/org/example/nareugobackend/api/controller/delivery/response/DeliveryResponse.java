package org.example.nareugobackend.api.controller.delivery.response;

import lombok.Builder;
import lombok.Getter;
import org.example.nareugobackend.domain.delivery.DeliveryStatus;

import java.time.LocalDateTime;

@Getter
@Builder
public class DeliveryResponse {
    private Long deliveryId;        // 택배 id
    private Long orderId;           // 주문 ID
    private Long robotId;           // 로봇id (추후 구현 필요)
    private DeliveryStatus status;  // DeliveryStatus
    private LocalDateTime requestTime;   // 요청시간
    private LocalDateTime completeTime;  // 완료시간
    private String trackingNumber;  // 송장번호
    private String title;           // 상품 제목
}