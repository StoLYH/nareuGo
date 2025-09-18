package org.example.nareugobackend.api.service.transaction.response;

import lombok.Builder;
import lombok.Getter;

import java.math.BigDecimal;
import java.time.LocalDateTime;

@Getter
@Builder
public class TransactionHistoryResponse {
    private Long productId;
    private String title;
    private BigDecimal price;
    private String productStatus;
    private LocalDateTime createdAt;

    private Long orderId;
    private String orderStatus;
    private LocalDateTime orderCreatedAt;
    private BigDecimal amount;

    // 판매내역용
    private String buyerNickname;

    // 구매내역용
    private String sellerNickname;
}