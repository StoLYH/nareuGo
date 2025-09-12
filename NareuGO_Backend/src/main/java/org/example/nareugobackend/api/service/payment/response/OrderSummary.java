package org.example.nareugobackend.api.service.payment.response;

import java.math.BigDecimal;
import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class OrderSummary {
    private Long orderId;
    private Long productId;
    private Long buyerId;
    private String status;
    private BigDecimal amount;
}


