package org.example.nareugobackend.api.service.robot;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.example.nareugobackend.api.controller.robot.response.DeliveryAddressResponse;
import org.example.nareugobackend.api.controller.robot.response.RobotStatusResponse;
import org.example.nareugobackend.common.exception.robot.RobotErrorCode;
import org.example.nareugobackend.common.exception.robot.RobotException;
import org.example.nareugobackend.common.model.RobotStatus;
import org.example.nareugobackend.domain.delivery.Delivery;
import org.example.nareugobackend.domain.delivery.DeliveryRepository;
import org.example.nareugobackend.domain.order.Order;
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

            // 배송 정보 조회
            Delivery delivery = deliveryRepository.findById(deliveryId)
                    .orElseThrow(() -> new RobotException(RobotErrorCode.DELIVERY_NOT_FOUND,
                            "배송 정보를 찾을 수 없습니다: " + deliveryId));

            Order order = delivery.getOrder();
            if (order == null) {
                throw new RobotException(RobotErrorCode.DELIVERY_NOT_FOUND, "주문 정보를 찾을 수 없습니다");
            }

            User buyer = order.getBuyer();
            if (buyer == null) {
                throw new RobotException(RobotErrorCode.DELIVERY_ADDRESS_INVALID, "구매자 정보를 찾을 수 없습니다");
            }

            // 판매자 주소 (배송 주소에서 추출 또는 고정값)
            String sellerAddress = delivery.getDeliveryAddress();
            if (sellerAddress == null || sellerAddress.isEmpty()) {
                sellerAddress = "105동 1301호"; // 기본 픽업 위치
            }

            // 구매자 주소 구성
            String buyerAddress = buildUserAddress(buyer);

            log.info("주소 정보 조회 완료 - 판매자: {}, 구매자: {}", sellerAddress, buyerAddress);

            return DeliveryAddressResponse.builder()
                    .sellerAddress(sellerAddress)
                    .buyerAddress(buyerAddress)
                    .build();

        } catch (Exception e) {
            log.error("배송 주소 조회 실패: {}", e.getMessage());

            // 실패 시 기본값 반환
            return DeliveryAddressResponse.builder()
                    .sellerAddress("105동 1301호")
                    .buyerAddress("103동 2101호")
                    .build();
        }
    }

    private String buildUserAddress(User user) {
        StringBuilder address = new StringBuilder();

        // 시도, 시군구, 읍면동 추가
        if (user.getSiDo() != null) {
            address.append(user.getSiDo()).append(" ");
        }
        if (user.getSiGunGu() != null) {
            address.append(user.getSiGunGu()).append(" ");
        }
        if (user.getEupMyeonDong() != null) {
            address.append(user.getEupMyeonDong()).append(" ");
        }

        // 아파트명 추가
        if (user.getApartmentName() != null) {
            address.append(user.getApartmentName()).append(" ");
        }

        // 동호수 추가
        if (user.getBuildingDong() != null && user.getBuildingHo() != null) {
            address.append(user.getBuildingDong()).append("동 ")
                   .append(user.getBuildingHo()).append("호");
        }

        String result = address.toString().trim();

        // 주소 정보가 불완전한 경우 기본값 반환
        if (result.isEmpty()) {
            return "103동 2101호";
        }

        return result;
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
}