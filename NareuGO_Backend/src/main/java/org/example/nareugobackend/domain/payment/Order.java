package org.example.nareugobackend.domain.payment;


import java.time.LocalDateTime;
import lombok.Getter;
import lombok.Setter;

// orders 테이블 데이터를 담는 객체
@Getter
@Setter
public class Order {

    private Long orderId;
    private Long productId;
    private Long buyerId;
    private OrderStatus status;
    private LocalDateTime createdAt;
    private LocalDateTime updatedAt;
}
