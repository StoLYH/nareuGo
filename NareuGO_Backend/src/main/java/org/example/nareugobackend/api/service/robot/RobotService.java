package org.example.nareugobackend.api.service.robot;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.example.nareugobackend.api.controller.robot.request.PickupConfirmationRequest;
import org.example.nareugobackend.api.controller.robot.response.DeliveryAddressResponse;
import org.example.nareugobackend.api.controller.robot.response.DeliveryCompletionResponse;
import org.example.nareugobackend.api.controller.robot.response.PickupConfirmationResponse;
import org.example.nareugobackend.api.controller.robot.response.RobotStatusResponse;
import org.example.nareugobackend.common.exception.robot.RobotErrorCode;
import org.example.nareugobackend.common.exception.robot.RobotException;
import org.example.nareugobackend.common.model.RobotStatus;
import org.example.nareugobackend.domain.delivery.Delivery;
import org.example.nareugobackend.domain.delivery.DeliveryRepository;
import org.example.nareugobackend.domain.order.Order;
import org.example.nareugobackend.domain.order.OrderRepository;
import org.example.nareugobackend.domain.product.Product;
import org.example.nareugobackend.domain.product.ProductRepository;
import org.example.nareugobackend.domain.user.User;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;
import org.springframework.web.reactive.socket.WebSocketMessage;
import org.springframework.web.reactive.socket.client.WebSocketClient;
import reactor.core.publisher.Mono;

import java.net.URI;
import java.time.Duration;
import java.util.concurrent.CompletableFuture;

@Slf4j
@Service
@RequiredArgsConstructor
public class RobotService {

    private final WebSocketClient webSocketClient;
    private final DeliveryRepository deliveryRepository;
    private final OrderRepository orderRepository;
    private final ProductRepository productRepository;
    private final ObjectMapper objectMapper = new ObjectMapper();

    @Value("${ros.bridge.url:ws://localhost:9090}")
    private String rosBridgeUrl;

    public CompletableFuture<RobotStatusResponse> checkRobotStatus(String robotId) {
        CompletableFuture<RobotStatusResponse> future = new CompletableFuture<>();

        try {
            URI uri = URI.create(rosBridgeUrl);

            webSocketClient.execute(uri, session -> {
                String subscribeMessage = String.format(
                    "{\"op\":\"call_service\",\"service\":\"/robot/%s/status\",\"args\":{}}",
                    robotId
                );

                return session.send(
                    Mono.just(session.textMessage(subscribeMessage))
                ).then(
                    session.receive()
                        .map(WebSocketMessage::getPayloadAsText)
                        .take(1)
                        .doOnNext(message -> {
                            try {
                                JsonNode response = objectMapper.readTree(message);
                                boolean isAvailable = response.path("values").path("available").asBoolean(false);
                                RobotStatus status = isAvailable ? RobotStatus.VALID : RobotStatus.INVALID;

                                RobotStatusResponse robotResponse = RobotStatusResponse.builder()
                                    .status(status.getValue())
                                    .message(getStatusMessage(status))
                                    .timestamp(java.time.Instant.now().toString())
                                    .build();

                                future.complete(robotResponse);
                            } catch (Exception e) {
                                log.error("ROS Bridge 응답 파싱 실패: {}", e.getMessage());
                                future.complete(createErrorResponse());
                            }
                        })
                        .doOnError(error -> {
                            log.error("ROS Bridge 통신 실패: {}", error.getMessage());
                            future.complete(createErrorResponse());
                        })
                        .then()
                );
            }).timeout(Duration.ofSeconds(5))
              .doOnError(error -> {
                  log.error("ROS Bridge 연결 실패: {}", error.getMessage());
                  future.complete(createErrorResponse());
              })
              .subscribe();

        } catch (Exception e) {
            log.error("로봇 상태 확인 중 오류 발생: {}", e.getMessage());
            future.complete(createErrorResponse());
        }

        return future;
    }

    private RobotStatusResponse createErrorResponse() {
        return RobotStatusResponse.builder()
            .status(RobotStatus.INVALID.getValue())
            .message("작업 불가능")
            .timestamp(java.time.Instant.now().toString())
            .build();
    }

    private String getStatusMessage(RobotStatus status) {
        switch (status) {
            case VALID:
                return "작업 가능";
            case INVALID:
                return "작업 불가능";
            default:
                return "상태 불명";
        }
    }

    public void sendLocationRequest(String robotId, String destination) {
        try {
            URI uri = URI.create(rosBridgeUrl);

            webSocketClient.execute(uri, session -> {
                String locationMessage = String.format(
                    "{\"op\":\"publish\",\"topic\":\"/robot/%s/move_to\",\"msg\":{\"destination\":\"%s\"}}",
                    robotId, destination
                );

                return session.send(
                    Mono.just(session.textMessage(locationMessage))
                ).then();
            }).subscribe();

            log.info("로봇 {}에게 위치 요청 전송: {}", robotId, destination);

        } catch (Exception e) {
            log.error("위치 요청 전송 실패: {}", e.getMessage());
        }
    }

    public void sendPickupConfirmation(String robotId, String orderId) {
        try {
            URI uri = URI.create(rosBridgeUrl);

            webSocketClient.execute(uri, session -> {
                String pickupMessage = String.format(
                    "{\"op\":\"publish\",\"topic\":\"/robot/%s/pickup_confirm\",\"msg\":{\"order_id\":\"%s\"}}",
                    robotId, orderId
                );

                return session.send(
                    Mono.just(session.textMessage(pickupMessage))
                ).then();
            }).subscribe();

            log.info("로봇 {}에게 픽업 확인 전송: {}", robotId, orderId);

        } catch (Exception e) {
            log.error("픽업 확인 전송 실패: {}", e.getMessage());
        }
    }

    public void sendDeliveryComplete(String robotId, String orderId) {
        try {
            URI uri = URI.create(rosBridgeUrl);

            webSocketClient.execute(uri, session -> {
                String deliveryMessage = String.format(
                    "{\"op\":\"publish\",\"topic\":\"/robot/%s/delivery_complete\",\"msg\":{\"order_id\":\"%s\"}}",
                    robotId, orderId
                );

                return session.send(
                    Mono.just(session.textMessage(deliveryMessage))
                ).then();
            }).subscribe();

            log.info("로봇 {}에게 배송 완료 전송: {}", robotId, orderId);

        } catch (Exception e) {
            log.error("배송 완료 전송 실패: {}", e.getMessage());
        }
    }

    public DeliveryAddressResponse getDeliveryAddresses(Long deliveryId) {
        try {
            log.info("배송 ID {}에 대한 주소 정보 조회 시작", deliveryId);

            // 최적화된 쿼리로 배송, 주문, 구매자 정보를 한 번에 조회
            Delivery delivery = deliveryRepository.findByIdWithOrderAndBuyer(deliveryId)
                    .orElseThrow(() -> new RobotException(RobotErrorCode.DELIVERY_NOT_FOUND,
                            "배송 정보를 찾을 수 없습니다: " + deliveryId));

            Order order = delivery.getOrder();
            User buyer = order.getBuyer();

            // 판매자 정보 조회
            User seller = getSeller(order);

            // 판매자 주소 구성
            String sellerAddress = buildUserAddress(seller);

            // 구매자 주소 구성
            String buyerAddress = buildUserAddress(buyer);

            log.info("주소 정보 조회 완료 - 판매자: {}, 구매자: {}", sellerAddress, buyerAddress);

            return DeliveryAddressResponse.builder()
                    .sellerAddress(sellerAddress)
                    .buyerAddress(buyerAddress)
                    .build();

        } catch (RobotException e) {
            log.error("로봇 예외 발생: {}", e.getMessage());
            throw e;
        } catch (Exception e) {
            log.error("배송 주소 조회 실패: {}", e.getMessage());
            throw new RobotException(RobotErrorCode.DELIVERY_ADDRESS_INVALID, "주소 조회 중 오류 발생", e);
        }
    }

    private User getSeller(Order order) {
        // 주문에서 상품 ID를 통해 상품 조회 후 판매자 정보 반환
        Product product = productRepository.findById(order.getProductId())
                .orElseThrow(() -> new RobotException(RobotErrorCode.DELIVERY_ADDRESS_INVALID,
                        "상품 정보를 찾을 수 없습니다: " + order.getProductId()));

        User seller = product.getSeller();
        if (seller == null) {
            throw new RobotException(RobotErrorCode.DELIVERY_ADDRESS_INVALID, "판매자 정보를 찾을 수 없습니다");
        }

        return seller;
    }

    private String buildUserAddress(User user) {
        // 동호수 조합하여 "XXX동 XXX호" 형태로 반환
        if (user.getBuildingDong() != null && user.getBuildingHo() != null) {
            return user.getBuildingDong() + "동 " + user.getBuildingHo() + "호";
        }

        // 동호수 정보가 없는 경우 기본값 반환 (판매자용 vs 구매자용)
        return "105동 1301호";
    }

    public void sendDeliveryStartCommand(String robotId, Long deliveryId, DeliveryAddressResponse addresses) {
        try {
            URI uri = URI.create(rosBridgeUrl);

            // 배송 시작 명령 구성
            String deliveryStartMessage = String.format(
                "{\"op\":\"publish\",\"topic\":\"/delivery/start_request\",\"msg\":{" +
                "\"robot_id\":\"%s\"," +
                "\"delivery_id\":%d," +
                "\"seller_address\":\"%s\"," +
                "\"buyer_address\":\"%s\"," +
                "\"timestamp\":%d" +
                "}}",
                robotId, deliveryId,
                addresses.getSellerAddress(),
                addresses.getBuyerAddress(),
                System.currentTimeMillis()
            );

            webSocketClient.execute(uri, session -> {
                return session.send(
                    Mono.just(session.textMessage(deliveryStartMessage))
                ).then();
            }).subscribe();

            log.info("로봇 {}에게 배송 시작 명령 전송: 배송 ID {}, 픽업 주소: {}, 배송 주소: {}",
                    robotId, deliveryId, addresses.getSellerAddress(), addresses.getBuyerAddress());

        } catch (Exception e) {
            log.error("배송 시작 명령 전송 실패: {}", e.getMessage());
            throw new RobotException(RobotErrorCode.ROBOT_COMMAND_FAILED, "배송 시작 명령 전송 실패", e);
        }
    }

    public PickupConfirmationResponse confirmPickup(Long deliveryId) {
        try {
            log.info("배송 ID {}에 대한 픽업 확인 처리 시작", deliveryId);

            // 1. 배송 정보 조회
            Delivery delivery = deliveryRepository.findByIdWithOrderAndBuyer(deliveryId)
                    .orElseThrow(() -> new RobotException(RobotErrorCode.DELIVERY_NOT_FOUND,
                            "배송 정보를 찾을 수 없습니다: " + deliveryId));

            Order order = delivery.getOrder();

            // 2. 배송 상태 검증 (RECEIPT_COMPLETED 상태여야 픽업 가능)
            if (delivery.getStatus() == null || delivery.getStatus() != org.example.nareugobackend.domain.delivery.DeliveryStatus.RECEIPT_COMPLETED) {
                throw new RobotException(RobotErrorCode.DELIVERY_ADDRESS_INVALID,
                        "픽업 가능한 상태가 아닙니다. 현재 배송 상태: " + delivery.getStatus());
            }

            // 3. 배송 상태를 IN_DELIVERY로 업데이트 (주문 상태는 그대로 유지)
            Delivery updatedDelivery = Delivery.builder()
                    .id(delivery.getId())
                    .order(delivery.getOrder())
                    .status(org.example.nareugobackend.domain.delivery.DeliveryStatus.IN_DELIVERY)
                    .trackingNumber(delivery.getTrackingNumber())
                    .deliveryAddress(delivery.getDeliveryAddress())
                    .estimatedDeliveryTime(delivery.getEstimatedDeliveryTime())
                    .actualDeliveryTime(delivery.getActualDeliveryTime())
                    .createdAt(delivery.getCreatedAt())
                    .updatedAt(java.time.LocalDateTime.now())
                    .build();

            deliveryRepository.save(updatedDelivery);

            log.info("배송 ID {} 상태를 IN_DELIVERY로 업데이트 완료 (주문 상태 유지: {})", deliveryId, order.getStatus());

            // 4. 응답 생성
            return PickupConfirmationResponse.builder()
                    .message("픽업 확인 완료")
                    .updatedStatus("IN_DELIVERY")
                    .timestamp(java.time.Instant.now().toString())
                    .build();

        } catch (RobotException e) {
            log.error("로봇 예외 발생: {}", e.getMessage());
            throw e;
        } catch (Exception e) {
            log.error("픽업 확인 처리 실패: {}", e.getMessage());
            throw new RobotException(RobotErrorCode.DELIVERY_ADDRESS_INVALID, "픽업 확인 처리 중 오류 발생", e);
        }
    }

    public DeliveryCompletionResponse completeDelivery(Long deliveryId) {
        try {
            log.info("배송 ID {}에 대한 배송 완료 처리 시작", deliveryId);

            // 1. 배송 정보 조회
            Delivery delivery = deliveryRepository.findByIdWithOrderAndBuyer(deliveryId)
                    .orElseThrow(() -> new RobotException(RobotErrorCode.DELIVERY_NOT_FOUND,
                            "배송 정보를 찾을 수 없습니다: " + deliveryId));

            Order order = delivery.getOrder();

            // 2. 배송 상태 검증 (IN_DELIVERY 상태여야 배송 완료 가능)
            if (delivery.getStatus() == null || delivery.getStatus() != org.example.nareugobackend.domain.delivery.DeliveryStatus.IN_DELIVERY) {
                throw new RobotException(RobotErrorCode.DELIVERY_ADDRESS_INVALID,
                        "배송 완료 가능한 상태가 아닙니다. 현재 배송 상태: " + delivery.getStatus());
            }

            // 3. 배송 상태를 DELIVERY_COMPLETED로 업데이트 (주문 상태는 그대로 유지)
            Delivery updatedDelivery = Delivery.builder()
                    .id(delivery.getId())
                    .order(delivery.getOrder())
                    .status(org.example.nareugobackend.domain.delivery.DeliveryStatus.DELIVERY_COMPLETED)
                    .trackingNumber(delivery.getTrackingNumber())
                    .deliveryAddress(delivery.getDeliveryAddress())
                    .estimatedDeliveryTime(delivery.getEstimatedDeliveryTime())
                    .actualDeliveryTime(java.time.LocalDateTime.now())
                    .createdAt(delivery.getCreatedAt())
                    .updatedAt(java.time.LocalDateTime.now())
                    .build();

            deliveryRepository.save(updatedDelivery);

            log.info("배송 ID {} 상태를 DELIVERY_COMPLETED로 업데이트 완료 (주문 상태 유지: {})", deliveryId, order.getStatus());

            // 4. 응답 생성
            return DeliveryCompletionResponse.builder()
                    .message("배송 완료")
                    .updatedStatus("DELIVERY_COMPLETED")
                    .timestamp(java.time.Instant.now().toString())
                    .build();

        } catch (RobotException e) {
            log.error("로봇 예외 발생: {}", e.getMessage());
            throw e;
        } catch (Exception e) {
            log.error("배송 완료 처리 실패: {}", e.getMessage());
            throw new RobotException(RobotErrorCode.DELIVERY_ADDRESS_INVALID, "배송 완료 처리 중 오류 발생", e);
        }
    }
}