package org.example.nareugobackend.api.controller.payment.request;

import lombok.Getter;
import lombok.Setter;
import java.math.BigDecimal;
import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotNull;
import jakarta.validation.constraints.Positive;

// 프론트엔드에서 결제 승인 요청 시 백엔드로 보낼 데이터
@Getter
@Setter
public class PaymentConfirmRequestDto {

    @NotBlank
    private String paymentKey;
    @NotNull
    private Long orderId;
    @NotNull
    @Positive
    private BigDecimal amount;
}
