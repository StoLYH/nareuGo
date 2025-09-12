package org.example.nareugobackend.api.controller.payment;

import jakarta.validation.constraints.NotNull;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.example.nareugobackend.api.service.payment.OrderService;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequestMapping("/api/orders")
@RequiredArgsConstructor
public class OrderController {

    private final OrderService orderService;

    @PostMapping
    public ResponseEntity<CreateOrderResponse> createOrder(@RequestBody CreateOrderRequest request) {
        Long orderId = orderService.createPendingOrder(request.getProductId(), request.getBuyerId());
        CreateOrderResponse response = new CreateOrderResponse();
        response.setOrderId(orderId);
        return ResponseEntity.ok(response);
    }

    @Getter
    @Setter
    public static class CreateOrderRequest {
        @NotNull
        private Long productId;
        @NotNull
        private Long buyerId;
    }

    @Getter
    @Setter
    public static class CreateOrderResponse {
        private Long orderId;
    }
}


