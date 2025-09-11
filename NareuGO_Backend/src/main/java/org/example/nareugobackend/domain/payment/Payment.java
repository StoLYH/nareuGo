package org.example.nareugobackend.domain.payment;


import java.math.BigDecimal;
import java.time.LocalDateTime;
import lombok.Getter;
import lombok.Setter;

// DB의 payments 테이블 데이터를 담는 객체
@Getter
@Setter
public class Payment {

    private Long paymentId;
    private Long orderId;
    private String paymentKey;
    private BigDecimal amount;
    private PaymentStatus status;
    private LocalDateTime approvedAt;
}
