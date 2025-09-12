package org.example.nareugobackend.api.controller.payment.response;

import java.math.BigDecimal;
import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class OrderResponseDto {
    private Long orderId;
    private Long productId;
    private Long buyerId;
    private String status;
    private BigDecimal amount;
}


