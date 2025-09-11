package org.example.nareugobackend.api.service.payment.response;

import java.time.OffsetDateTime;
import lombok.Getter;
import lombok.Setter;

// 서비스 계층에서 토스 API 응답을 받기 위한 객체
@Getter
@Setter
public class TossPaymentConfirmResponseDto {

    private String paymentKey;
    private String orderId;
    private String status;
    private Integer totalAmount;
    private OffsetDateTime approvedAt;
}
