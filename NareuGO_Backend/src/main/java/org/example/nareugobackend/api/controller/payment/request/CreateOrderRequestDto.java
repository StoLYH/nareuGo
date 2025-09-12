package org.example.nareugobackend.api.controller.payment.request;

import jakarta.validation.constraints.NotNull;
import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class CreateOrderRequestDto {
    @NotNull
    private Long productId;
    @NotNull
    private Long buyerId;
}


