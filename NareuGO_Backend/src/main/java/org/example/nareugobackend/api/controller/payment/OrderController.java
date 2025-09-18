package org.example.nareugobackend.api.controller.payment;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.controller.payment.request.CreateOrderRequestDto;
import org.example.nareugobackend.api.controller.payment.response.CreateOrderResponseDto;
import org.example.nareugobackend.api.controller.payment.response.OrderResponseDto;
import org.example.nareugobackend.api.service.payment.OrderService;
import org.example.nareugobackend.api.controller.payment.response.OrderSummary;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

/**
 * 주문 관련 REST API 컨트롤러
 * 
 * 중고거래 플랫폼의 주문 생성과 조회 기능을 제공합니다.
 * 채팅방에서 결제 버튼을 누르면 주문이 생성되고, 결제 확인 페이지에서 주문 정보를 조회할 수 있습니다.
 */
@RestController
@RequestMapping("/orders")
@RequiredArgsConstructor
public class OrderController {

    private final OrderService orderService;

    /**
     * 결제 대기 주문을 생성합니다.
     * 
     * 채팅방에서 "결제하기" 버튼을 눌렀을 때 호출되는 API입니다.
     * 상품 가격을 기준으로 주문을 생성하고, 15분 후 자동으로 만료됩니다.
     * 동일 상품에 대해 이미 결제 대기 중인 주문이 있으면 생성이 차단됩니다.
     * 
     * @param request 주문 생성 요청 (productId, buyerId)
     * @return 생성된 주문 ID
     */
    @PostMapping
    public ResponseEntity<CreateOrderResponseDto> createOrder(@RequestBody CreateOrderRequestDto request) {
        Long orderId = orderService.createPendingOrder(request.getProductId(), request.getBuyerId());
        CreateOrderResponseDto response = new CreateOrderResponseDto();
        response.setOrderId(orderId);
        return ResponseEntity.ok(response);
    }

    /**
     * 주문 정보를 조회합니다.
     * 
     * 결제 확인 페이지에서 주문 정보를 가져올 때 호출되는 API입니다.
     * 조회 시점에 15분이 지난 결제 대기 주문은 자동으로 취소 처리됩니다.
     * 
     * @param orderId 조회할 주문 ID
     * @return 주문 정보 (ID, 상품ID, 구매자ID, 상태, 금액)
     */
    @GetMapping("/{orderId}")
    public ResponseEntity<OrderResponseDto> getOrder(@PathVariable Long orderId) {
        OrderSummary summary = orderService.getOrderAndAutoExpire(orderId);
        OrderResponseDto res = new OrderResponseDto();
        res.setOrderId(summary.getOrderId());
        res.setProductId(summary.getProductId());
        res.setBuyerId(summary.getBuyerId());
        res.setStatus(summary.getStatus());
        res.setAmount(summary.getAmount());
        res.setTossOrderId(summary.getTossOrderId()); // 토스페이먼츠용 orderId 추가
        return ResponseEntity.ok(res);
    }

    /**
     * 토스페이먼츠 orderId로 주문 정보를 조회합니다.
     * 
     * 결제 완료 페이지에서 tossOrderId를 사용하여 주문 정보를 가져올 때 호출되는 API입니다.
     * 
     * @param tossOrderId 토스페이먼츠 주문 ID
     * @return 주문 정보 (ID, 상품ID, 구매자ID, 상태, 금액)
     */
    @GetMapping("/toss/{tossOrderId}")
    public ResponseEntity<OrderResponseDto> getOrderByTossOrderId(@PathVariable String tossOrderId) {
        OrderSummary summary = orderService.getOrderByTossOrderId(tossOrderId);
        OrderResponseDto res = new OrderResponseDto();
        res.setOrderId(summary.getOrderId());
        res.setProductId(summary.getProductId());
        res.setBuyerId(summary.getBuyerId());
        res.setStatus(summary.getStatus());
        res.setAmount(summary.getAmount());
        res.setTossOrderId(summary.getTossOrderId());
        return ResponseEntity.ok(res);
    }
}


