package org.example.nareugobackend.api.service.robot;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
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
import org.example.nareugobackend.api.service.notification.FcmService;
import org.example.nareugobackend.api.service.notification.NotificationService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.web.client.RestTemplate;

import java.util.concurrent.CompletableFuture;

@Slf4j
@Service
@RequiredArgsConstructor
public class RobotService {

    private final DeliveryRepository deliveryRepository;
    private final OrderRepository orderRepository;
    private final ProductRepository productRepository;
    private final RestTemplate restTemplate;
    private final ObjectMapper objectMapper = new ObjectMapper();

    @Value("${robot.http.url:http://localhost:8888}")
    private String robotHttpUrl;

    public CompletableFuture<RobotStatusResponse> checkRobotStatus(String robotId) {
        CompletableFuture<RobotStatusResponse> future = new CompletableFuture<>();

        // 하드코딩: 개발용으로 항상 성공 응답 반환
        log.info("로봇 상태 확인 요청 (하드코딩 응답): {}", robotId);

        RobotStatusResponse robotResponse = RobotStatusResponse.builder()
            .status(RobotStatus.VALID.getValue())
            .message("작업 가능")
            .timestamp(java.time.Instant.now().toString())
            .build();

        log.info("로봇 상태 응답 성공 (하드코딩): status={}, message={}",
                 robotResponse.getStatus(), robotResponse.getMessage());
        future.complete(robotResponse);

        return future;
    }

    private RobotStatusResponse createErrorResponse() {
        // 하드코딩: 에러 발생 시에도 성공 응답 반환
        return RobotStatusResponse.builder()
            .status(RobotStatus.VALID.getValue())
            .message("작업 가능")
            .timestamp(java.time.Instant.now().toString())
            .build();
    }

    private String getStatusMessage(RobotStatus status) {
        // 하드코딩: 모든 상태에 대해 "작업 가능" 반환
        return "작업 가능";
    }

    public void sendLocationRequest(String robotId, String destination) {
        try {
            log.info("로봇 {}에게 위치 요청 전송: {}", robotId, destination);
            // HTTP 기반 로봇 제어로 변경 예정
        } catch (Exception e) {
            log.error("위치 요청 전송 실패: {}", e.getMessage());
        }
    }

    public void sendPickupConfirmation(String robotId, String orderId) {
        try {
            log.info("로봇 {}에게 픽업 확인 전송: {}", robotId, orderId);
            // HTTP 기반 로봇 제어로 변경 예정
        } catch (Exception e) {
            log.error("픽업 확인 전송 실패: {}", e.getMessage());
        }
    }

    public void sendDeliveryComplete(String robotId, String orderId) {
        try {
            log.info("로봇 {}에게 배송 완료 전송: {}", robotId, orderId);
            // HTTP 기반 로봇 제어로 변경 예정
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
            log.info("로봇 {}에게 배송 시작 명령 전송: 배송 ID {}, 픽업 주소: {}, 배송 주소: {}",
                    robotId, deliveryId, addresses.getSellerAddress(), addresses.getBuyerAddress());
            // HTTP 기반 로봇 제어로 변경 예정
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

    public void executeDeliveryScript(Long deliveryId) {
        try {
            log.info("배송 ID {}에 대한 delivery.py 스크립트 실행", deliveryId);

            // 1. 배송 주소 정보 조회
            DeliveryAddressResponse addresses = getDeliveryAddresses(deliveryId);

            // 2. Python 스크립트 실행
            ProcessBuilder processBuilder = new ProcessBuilder(
                "python3",
                "../Embedded/src/delivery.py",
                deliveryId.toString(),
                addresses.getSellerAddress(),
                addresses.getBuyerAddress()
            );

            processBuilder.directory(new java.io.File(System.getProperty("user.dir")));
            processBuilder.redirectErrorStream(true); // stderr을 stdout으로 리다이렉트

            Process process = processBuilder.start();

            // Python 스크립트의 출력을 읽어서 로그로 출력
            try (java.io.BufferedReader reader = new java.io.BufferedReader(
                    new java.io.InputStreamReader(process.getInputStream()))) {
                String line;
                while ((line = reader.readLine()) != null) {
                    log.info("delivery.py 출력: {}", line);
                }
            }

            // 프로세스 완료 대기
            int exitCode = process.waitFor();

            if (exitCode == 0) {
                log.info("delivery.py 스크립트 실행 완료: {}", deliveryId);
            } else {
                log.error("delivery.py 스크립트 실행 실패: exit code {}", exitCode);
            }

        } catch (Exception e) {
            log.error("delivery.py 스크립트 실행 중 오류 발생: {}", e.getMessage());
            throw new RobotException(RobotErrorCode.ROBOT_COMMAND_FAILED, "배송 스크립트 실행 실패", e);
        }
    }

    @Autowired
    private NotificationService notificationService;

    @Transactional
    public void notifySellerArrival(Long deliveryId, FcmService fcmService) {
        try {
            log.info("배송 ID {}에 대한 판매자 집 도착 알림 처리 시작", deliveryId);

            // 1. 배송 정보 조회 (배송, 주문, 구매자, 판매자 정보 모두 필요)
            Delivery delivery = deliveryRepository.findByIdWithOrderAndBuyer(deliveryId)
                    .orElseThrow(() -> new RobotException(RobotErrorCode.DELIVERY_NOT_FOUND,
                            "배송 정보를 찾을 수 없습니다: " + deliveryId));

            Order order = delivery.getOrder();
            User buyer = order.getBuyer();

            // 2. 판매자 정보 조회
            User seller = getSeller(order);

            // 3. 상품 정보 조회
            Product product = productRepository.findById(order.getProductId())
                    .orElseThrow(() -> new RobotException(RobotErrorCode.DELIVERY_ADDRESS_INVALID,
                            "상품 정보를 찾을 수 없습니다: " + order.getProductId()));

            // 4. 알림 DB 저장
            notificationService.createSellerArrivalNotification(
                    seller.getId(),
                    product.getTitle(),
                    buyer.getName()
            );

            // 5. FCM 알림 발송
            fcmService.sendSellerArrivalNotification(
                    seller.getId(),
                    product.getTitle(),
                    buyer.getName()
            );

            log.info("판매자 집 도착 알림 처리 완료 - 판매자: {}, 상품: {}, 구매자: {}",
                    seller.getName(), product.getTitle(), buyer.getName());

        } catch (RobotException e) {
            log.error("로봇 예외 발생: {}", e.getMessage());
            throw e;
        } catch (Exception e) {
            log.error("판매자 집 도착 알림 처리 실패: {}", e.getMessage());
            throw new RobotException(RobotErrorCode.DELIVERY_ADDRESS_INVALID, "판매자 집 도착 알림 처리 중 오류 발생", e);
        }
    }
}