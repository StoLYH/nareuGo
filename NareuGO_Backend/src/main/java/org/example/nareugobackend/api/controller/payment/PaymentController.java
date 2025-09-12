package org.example.nareugobackend.api.controller.payment;

import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.controller.payment.request.PaymentConfirmRequestDto;
import org.example.nareugobackend.api.service.payment.PaymentService;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequestMapping("/api/payments")
@RequiredArgsConstructor
public class PaymentController {

    private final PaymentService paymentService;

    @PostMapping("/confirm")
    public ResponseEntity<Void> confirmPayment(@Valid @RequestBody PaymentConfirmRequestDto requestDto) {
        paymentService.confirmPayment(requestDto);
        return ResponseEntity.noContent().build();
    }
}
