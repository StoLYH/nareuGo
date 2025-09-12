package org.example.nareugobackend.api.controller.payment.request;

import lombok.Getter;
import lombok.Setter;
import java.math.BigDecimal;

// 프론트엔드에서 결제 승인 요청 시 백엔드로 보낼 데이터
@Getter
@Setter
public class PaymentConfirmRequestDto {

    private String paymentKey;
    private Long orderId;
    private BigDecimal amount;
}
