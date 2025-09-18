package org.example.nareugobackend.api.controller.delivery;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.controller.delivery.response.DeliveryResponse;
import org.example.nareugobackend.api.service.delivery.DeliveryService;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequiredArgsConstructor
@RequestMapping("/mypage")
public class DeliveryController {

    private final DeliveryService deliveryService;

    @GetMapping("/deliveries/{userId}")
    public ResponseEntity<List<DeliveryResponse>> getUserDeliveries(@PathVariable Long userId) {
        List<DeliveryResponse> deliveries = deliveryService.getUserDeliveries(userId);
        return ResponseEntity.ok(deliveries);
    }
}