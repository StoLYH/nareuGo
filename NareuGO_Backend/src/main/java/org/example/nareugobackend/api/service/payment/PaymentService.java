package org.example.nareugobackend.api.service.payment;

import org.example.nareugobackend.api.controller.payment.request.PaymentConfirmRequestDto;

public interface PaymentService {

    void confirmPayment(PaymentConfirmRequestDto requestDto);

}
