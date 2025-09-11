package org.example.nareugobackend.api.service.payment;


import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.controller.payment.request.PaymentConfirmRequestDto;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class PaymentServiceImpl implements PaymentService {


    @Override
    public void confirmPayment(PaymentConfirmRequestDto requestDto) {

    }
}
