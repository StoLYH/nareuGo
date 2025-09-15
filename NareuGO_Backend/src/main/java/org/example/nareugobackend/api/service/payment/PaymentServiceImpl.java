package org.example.nareugobackend.api.service.payment;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.controller.payment.request.PaymentConfirmRequestDto;
import org.example.nareugobackend.common.model.Order;
import org.example.nareugobackend.common.model.OrderStatus;
import org.example.nareugobackend.common.model.Payment;
import org.example.nareugobackend.common.model.PaymentStatus;
import org.example.nareugobackend.mapper.OrderMapper;
import org.example.nareugobackend.mapper.PaymentMapper;
import org.example.nareugobackend.api.controller.payment.response.TossPaymentConfirmResponseDto;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.web.reactive.function.client.WebClient;
import reactor.core.publisher.Mono;

import java.nio.charset.StandardCharsets;
import java.time.LocalDateTime;
import java.util.Base64;
import java.util.Objects;

@Service
@RequiredArgsConstructor // final이 붙은 필드들을 자동으로 주입받는 생성자를 만들어줍니다.
public class PaymentServiceImpl implements PaymentService {

    // --- 의존성 주입 ---
    // @RequiredArgsConstructor가 생성자를 통해 자동으로 주입해주는 필드들입니다.
    private final OrderMapper orderMapper;
    private final PaymentMapper paymentMapper;

    // WebClient는 빌더를 주입받아 필요할 때 생성해서 사용하는 것이 좋습니다.
    private final WebClient.Builder webClientBuilder;

    // --- 설정값 주입 ---
    // application.yml 파일에 설정한 값을 Java 코드에서 사용할 수 있게 해줍니다.
    @Value("${payments.toss.secret-key}")
    private String tossSecretKey;

    @Value("${payments.toss.confirm-url}")
    private String tossConfirmUrl;

    /**
     * 결제 승인 요청을 처리하는 핵심 비즈니스 로직입니다.
     *
     * @param requestDto 프론트엔드에서 받은 결제 승인 요청 데이터 (paymentKey, orderId, amount)
     */
    @Override
    @Transactional // 이 어노테이션이 붙은 메소드 내의 모든 DB 작업은 하나의 트랜잭션으로 묶입니다.
    // 중간에 예외가 발생하면 모든 DB 작업이 자동으로 롤백(취소)됩니다.
    public void confirmPayment(PaymentConfirmRequestDto requestDto) {

        // --- 1. DB에서 주문 정보 조회 및 검증 ---
        // requestDto에 담긴 orderId를 사용하여 우리 DB에서 실제 주문 정보를 가져옵니다.
        Order order = orderMapper.findById(String.valueOf(requestDto.getOrderId()))
            .orElseThrow(() -> new IllegalArgumentException(
                "존재하지 않는 주문입니다. orderId=" + requestDto.getOrderId()));

        // --- 2. 결제 금액 위변조 검증 (매우 중요!) ---
        // 프론트엔드에서 보낸 결제 금액과 DB에 저장된 실제 주문 금액이 일치하는지 확인합니다.
        // 이 과정을 거치지 않으면 사용자가 브라우저에서 금액을 조작하여 결제할 수 있습니다.
        if (order.getAmount() == null || requestDto.getAmount() == null || order.getAmount().compareTo(requestDto.getAmount()) != 0) {
            throw new IllegalArgumentException("주문 금액이 일치하지 않습니다.");
        }

        // --- 3. 토스페이먼츠 API 호출 준비 ---
        // Basic 인증을 위해 시크릿 키를 Base64 형식으로 인코딩합니다.
        // Toss Payments API는 Basic 인증 스킴을 요구합니다. (https://[시크릿키]:@...)
        String encodedSecretKey = Base64.getEncoder()
            .encodeToString((tossSecretKey + ":").getBytes(StandardCharsets.UTF_8));

        // WebClient 인스턴스를 생성합니다.
        WebClient webClient = webClientBuilder.baseUrl(tossConfirmUrl).build();

        // --- 4. 토스페이먼츠 결제 승인 API 호출 ---
        // WebClient를 사용하여 비동기 POST 요청을 보냅니다.
        TossPaymentConfirmResponseDto tossResponse = webClient.post()
            .header("Authorization", "Basic " + encodedSecretKey)
            .header("Content-Type", "application/json")
            .bodyValue(requestDto) // paymentKey, orderId, amount
            .retrieve()
            .onStatus(
                status -> status.isError(),
                clientResponse -> clientResponse.bodyToMono(String.class)
                    .map(body -> new IllegalStateException("Toss confirm failed: " + body))
            )
            .bodyToMono(TossPaymentConfirmResponseDto.class)
            .block();

        if (tossResponse == null) {
            throw new IllegalStateException("Empty response from Toss Payments");
        }

        // 상태/금액 재검증
        if (!"DONE".equalsIgnoreCase(tossResponse.getStatus())) {
            throw new IllegalStateException("Payment not completed. status=" + tossResponse.getStatus());
        }
        if (tossResponse.getTotalAmount() == null ||
            order.getAmount().compareTo(java.math.BigDecimal.valueOf(tossResponse.getTotalAmount().longValue())) != 0) {
            throw new IllegalArgumentException("승인 응답 금액이 주문 금액과 일치하지 않습니다.");
        }
        // 실제 프로덕션 환경에서는 비동기 로직을 그대로 활용하는 것이 성능에 더 좋습니다.

        // --- 5. DB 상태 업데이트 ---
        // API 호출이 성공적으로 완료되면, 우리 서비스의 DB 데이터를 최종 상태로 변경합니다.

        // 5-1. 주문(orders) 테이블의 상태를 'PAYMENT_COMPLETED'로 업데이트합니다.
        order.setStatus(OrderStatus.PAYMENT_COMPLETED);
        orderMapper.updateStatus(order.getOrderId(), OrderStatus.PAYMENT_COMPLETED);

        // 5-2. 결제(payments) 테이블에 최종 결제 정보를 기록합니다.
        Payment payment = new Payment();
        payment.setOrderId(order.getOrderId());
        payment.setPaymentKey(requestDto.getPaymentKey());
        payment.setAmount(order.getAmount());
        payment.setStatus(PaymentStatus.DONE);
        payment.setApprovedAt(tossResponse.getApprovedAt() != null ? tossResponse.getApprovedAt().toLocalDateTime() : LocalDateTime.now());
        paymentMapper.save(payment); // MyBatis 매퍼를 통해 DB에 저장
    }
}