package org.example.nareugobackend.api.service.payment;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.example.nareugobackend.api.controller.payment.request.PaymentConfirmRequestDto;
import org.example.nareugobackend.common.model.Order;
import org.example.nareugobackend.common.model.OrderStatus;
import org.example.nareugobackend.common.model.Payment;
import org.example.nareugobackend.common.model.PaymentStatus;
import org.example.nareugobackend.mapper.OrderMapper;
import org.example.nareugobackend.mapper.PaymentMapper;
import org.example.nareugobackend.mapper.ProductMapper;
import org.example.nareugobackend.mapper.DeliveryMapper;
import org.example.nareugobackend.mapper.UserMapper;
import org.example.nareugobackend.common.model.ProductStatus;
import org.example.nareugobackend.api.controller.payment.response.TossPaymentConfirmResponseDto;
import org.example.nareugobackend.common.model.UserEntity;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.web.reactive.function.client.WebClient;
import reactor.core.publisher.Mono;

import java.nio.charset.StandardCharsets;
import java.time.LocalDateTime;
import java.util.Base64;
import java.util.Objects;
import java.util.Random;

@Slf4j
@Service
@RequiredArgsConstructor // final이 붙은 필드들을 자동으로 주입받는 생성자를 만들어줍니다.
public class PaymentServiceImpl implements PaymentService {

    // --- 의존성 주입 ---
    // @RequiredArgsConstructor가 생성자를 통해 자동으로 주입해주는 필드들입니다.
    private final OrderMapper orderMapper;
    private final PaymentMapper paymentMapper;
    private final ProductMapper productMapper;
    private final DeliveryMapper deliveryMapper;
    private final UserMapper userMapper;

    // WebClient는 빌더를 주입받아 필요할 때 생성해서 사용하는 것이 좋습니다.
    private final WebClient.Builder webClientBuilder;

    // --- 설정값 주입 ---
    // application.yml 파일에 설정한 값을 Java 코드에서 사용할 수 있게 해줍니다.
    @Value("${payments.toss.secret-key}")
    private String tossSecretKey;

    @Value("${payments.toss.confirm-url}")
    private String tossConfirmUrl;

    // 12자리 숫자 운송장 번호 생성 (결제 완료 시 배송 초기 레코드에 저장)
private String generateRandomTrackingNumber() {
    Random random = new Random();
    StringBuilder sb = new StringBuilder(12);
    for (int i = 0; i < 12; i++) {
        sb.append(random.nextInt(10));
    }
    return sb.toString();
}

    /**
     * 결제 승인 요청을 처리하는 핵심 비즈니스 로직입니다.
     *
     * @param requestDto 프론트엔드에서 받은 결제 승인 요청 데이터 (paymentKey, orderId, amount)
     */
    @Override
    @Transactional
    public void confirmPayment(PaymentConfirmRequestDto requestDto) {
        try {
            Order order = orderMapper.findByTossOrderId(requestDto.getOrderId());
            if (order == null) {
                throw new IllegalArgumentException("존재하지 않는 주문입니다. tossOrderId=" + requestDto.getOrderId());
            }

            if (order.getAmount() == null || requestDto.getAmount() == null || order.getAmount().compareTo(requestDto.getAmount()) != 0) {
                log.warn("[Payments] Amount mismatch - dbAmount={}, requestAmount={}", order.getAmount(), requestDto.getAmount());
                throw new IllegalArgumentException("주문 금액이 일치하지 않습니다.");
            }

            String encodedSecretKey = Base64.getEncoder()
                .encodeToString((tossSecretKey + ":").getBytes(StandardCharsets.UTF_8));
            WebClient webClient = webClientBuilder.baseUrl(tossConfirmUrl).build();

            TossPaymentConfirmResponseDto tossResponse = webClient.post()
                .header("Authorization", "Basic " + encodedSecretKey)
                .header("Content-Type", "application/json")
                .bodyValue(requestDto)
                .retrieve()
                .onStatus(
                    status -> status.is4xxClientError() || status.is5xxServerError(),
                    clientResponse -> clientResponse.bodyToMono(String.class)
                        .flatMap(body -> {
                            String message = "Toss confirm failed: " + body;
                            log.error("[Payments] Toss API error: status={}", clientResponse.statusCode());
                            if (clientResponse.statusCode().is4xxClientError()) {
                                return reactor.core.publisher.Mono.error(new org.springframework.web.server.ResponseStatusException(org.springframework.http.HttpStatus.BAD_REQUEST, message));
                            }
                            return reactor.core.publisher.Mono.error(new org.springframework.web.server.ResponseStatusException(org.springframework.http.HttpStatus.BAD_GATEWAY, message));
                        })
                )
                .bodyToMono(TossPaymentConfirmResponseDto.class)
                .block();

            if (tossResponse == null) {
                log.error("[Payments] Toss confirm returned null response");
                throw new IllegalStateException("Empty response from Toss Payments");
            }

            if (!"DONE".equalsIgnoreCase(tossResponse.getStatus())) {
                log.warn("[Payments] Toss status not DONE - status={}", tossResponse.getStatus());
                throw new IllegalStateException("Payment not completed. status=" + tossResponse.getStatus());
            }
            if (tossResponse.getTotalAmount() == null ||
                order.getAmount().compareTo(java.math.BigDecimal.valueOf(tossResponse.getTotalAmount().longValue())) != 0) {
                log.warn("[Payments] Toss amount mismatch");
                throw new IllegalArgumentException("승인 응답 금액이 주문 금액과 일치하지 않습니다.");
            }

            order.setStatus(OrderStatus.PAYMENT_COMPLETED);
            orderMapper.updateStatus(order.getOrderId(), OrderStatus.PAYMENT_COMPLETED);

            Payment payment = new Payment();
            payment.setOrderId(order.getOrderId());
            payment.setPaymentKey(requestDto.getPaymentKey());
            payment.setAmount(order.getAmount());
            payment.setStatus(PaymentStatus.DONE);
            payment.setApprovedAt(tossResponse.getApprovedAt() != null ? tossResponse.getApprovedAt().toLocalDateTime() : LocalDateTime.now());
            paymentMapper.save(payment);

            productMapper.updateStatus(order.getProductId(), ProductStatus.SOLD);

            try {
                UserEntity buyer = userMapper.findById(order.getBuyerId());
                StringBuilder addr = new StringBuilder();
                if (buyer != null) {
                    if (buyer.getApartmentName() != null && !buyer.getApartmentName().isEmpty()) {
                        addr.append(buyer.getApartmentName());
                    }
                    if (buyer.getBuildingDong() != null) {
                        if (addr.length() > 0) addr.append(" ");
                        addr.append(buyer.getBuildingDong()).append("동");
                    }
                    if (buyer.getBuildingHo() != null) {
                        if (addr.length() > 0) addr.append(" ");
                        addr.append(buyer.getBuildingHo()).append("호");
                    }
                }
                String deliveryAddress = addr.toString();
                String trackingNumber = generateRandomTrackingNumber();
                deliveryMapper.insertInitialDelivery(
                        order.getOrderId(),
                        deliveryAddress,
                        "RECEIPT_COMPLETED",
                        trackingNumber
                );
            } catch (Exception de) {
                log.error("[Payments] Failed to create initial delivery record", de);
            }

            log.info("[Payments] Payment completed - orderId={}", requestDto.getOrderId());
        } catch (RuntimeException ex) {
            log.error("[Payments] Payment failed - orderId={}", requestDto.getOrderId(), ex);
            throw ex;
        }
    }
    
}