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
import org.springframework.http.HttpEntity;
import org.springframework.http.HttpHeaders;
import org.springframework.http.MediaType;

import java.util.concurrent.CompletableFuture;
import java.util.HashMap;
import java.util.Map;

@Slf4j
@Service
@RequiredArgsConstructor
public class RobotService {

    private final DeliveryRepository deliveryRepository;
    private final OrderRepository orderRepository;
    private final ProductRepository productRepository;
    private final RestTemplate restTemplate;
    private final ObjectMapper objectMapper = new ObjectMapper();

    
    // 환경에 따른 로봇 URL 설정
    @Value("${server.env}")
    private String activeProfile;
    
    private String getRobotHttpUrl() {
        if ("local".equals(activeProfile)) {
            return "http://localhost:8888";
        } else {
            return "https://unfearful-orion-furuncular.ngrok-free.dev";
        }
    }

    public CompletableFuture<RobotStatusResponse> checkRobotStatus(String robotId, Long deliveryId) {
        CompletableFuture<RobotStatusResponse> future = new CompletableFuture<>();

        try {
            // 로봇 서버에 상태 확인 요청 (delivery_id 파라미터 추가)
            String robotStatusUrl = getRobotHttpUrl() + "/robot/status?robotId=" + robotId;
            if (deliveryId != null) {
                robotStatusUrl += "&delivery_id=" + deliveryId;
            }
            String response = restTemplate.getForObject(robotStatusUrl, String.class);

            // JSON 응답 파싱
            JsonNode jsonResponse = objectMapper.readTree(response);
            String status = jsonResponse.get("status").asText();
            String message = jsonResponse.get("message").asText();
            String timestamp = jsonResponse.get("timestamp").asText();

            RobotStatusResponse robotResponse = RobotStatusResponse.builder()
                .status(status)
                .message(message)
                .timestamp(timestamp)
                .build();

            // 로봇 상태 응답 성공
            future.complete(robotResponse);

        } catch (Exception e) {
            // 로봇 상태 확인 실패
            future.complete(createErrorResponse());
        }

        return future;
    }

    private RobotStatusResponse createErrorResponse() {
        return RobotStatusResponse.builder()
            .status(RobotStatus.ERROR.getValue())
            .message("로봇 통신 오류")
            .timestamp(java.time.Instant.now().toString())
            .build();
    }

    private String getStatusMessage(RobotStatus status) {
        switch (status) {
            case VALID:
                return "작업 가능";
            case BUSY:
                return "작업 중";
            case ERROR:
                return "오류 상태";
            default:
                return "알 수 없음";
        }
    }

    public void sendLocationRequest(String robotId, String destination) {
        // HTTP 기반 로봇 제어로 변경 예정
    }

    public void sendPickupConfirmation(String robotId, String orderId) {
        // HTTP 기반 로봇 제어로 변경 예정
    }

    public void sendDeliveryComplete(String robotId, String orderId) {
        // HTTP 기반 로봇 제어로 변경 예정
    }

    public DeliveryAddressResponse getDeliveryAddresses(Long deliveryId) {
        try {
            Delivery delivery = deliveryRepository.findByIdWithOrderAndBuyer(deliveryId)
                    .orElseThrow(() -> new RobotException(RobotErrorCode.DELIVERY_NOT_FOUND,
                            "배송 정보를 찾을 수 없습니다: " + deliveryId));

            Order order = delivery.getOrder();
            User buyer = order.getBuyer();
            User seller = getSeller(order);

            String sellerAddress = buildUserAddress(seller);
            String buyerAddress = buildUserAddress(buyer);

            return DeliveryAddressResponse.builder()
                    .sellerAddress(sellerAddress)
                    .buyerAddress(buyerAddress)
                    .build();
        } catch (RobotException e) {
            throw e;
        } catch (Exception e) {
            log.error("배송 주소 조회 실패", e);
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
            sendAddressesToRobot(addresses.getSellerAddress(), addresses.getBuyerAddress(), deliveryId);
        } catch (Exception e) {
            log.error("배송 시작 명령 전송 실패", e);
            throw new RobotException(RobotErrorCode.ROBOT_COMMAND_FAILED, "배송 시작 명령 전송 실패", e);
        }
    }

    /**
     * 로봇에 주소 정보를 전송하여 배송을 시작시키는 메서드
     */
    public void sendAddressesToRobot(String sellerAddress, String buyerAddress, Long deliveryId) {
        try {
            Map<String, String> addressData = new HashMap<>();
            addressData.put("sellerAddress", sellerAddress);
            addressData.put("buyerAddress", buyerAddress);

            HttpHeaders headers = new HttpHeaders();
            headers.setContentType(MediaType.APPLICATION_JSON);

            HttpEntity<Map<String, String>> requestEntity = new HttpEntity<>(addressData, headers);

            String robotUrl = getRobotHttpUrl() + "/robot/delivery/1/addresses" +
                    "?sellerAddress=" + java.net.URLEncoder.encode(sellerAddress, "UTF-8") +
                    "&buyerAddress=" + java.net.URLEncoder.encode(buyerAddress, "UTF-8");

        } catch (Exception e) {
            log.error("로봇 주소 전송 실패", e);
            throw new RobotException(RobotErrorCode.ROBOT_COMMAND_FAILED, "로봇 주소 전송 실패", e);
        }
    }

    public PickupConfirmationResponse confirmPickup(Long deliveryId) {
        try {
            Delivery delivery = deliveryRepository.findByIdWithOrderAndBuyer(deliveryId)
                    .orElseThrow(() -> new RobotException(RobotErrorCode.DELIVERY_NOT_FOUND,
                            "배송 정보를 찾을 수 없습니다: " + deliveryId));

            if (delivery.getStatus() == null || delivery.getStatus() != org.example.nareugobackend.domain.delivery.DeliveryStatus.RECEIPT_COMPLETED) {
                throw new RobotException(RobotErrorCode.DELIVERY_ADDRESS_INVALID,
                        "픽업 가능한 상태가 아닙니다. 현재 배송 상태: " + delivery.getStatus());
            }

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

            return PickupConfirmationResponse.builder()
                    .message("픽업 확인 완료")
                    .updatedStatus("IN_DELIVERY")
                    .timestamp(java.time.Instant.now().toString())
                    .build();
        } catch (RobotException e) {
            throw e;
        } catch (Exception e) {
            log.error("픽업 확인 처리 실패", e);
            throw new RobotException(RobotErrorCode.DELIVERY_ADDRESS_INVALID, "픽업 확인 처리 중 오류 발생", e);
        }
    }

    public DeliveryCompletionResponse completeDelivery(Long deliveryId) {
        try {
            Delivery delivery = deliveryRepository.findByIdWithOrderAndBuyer(deliveryId)
                    .orElseThrow(() -> new RobotException(RobotErrorCode.DELIVERY_NOT_FOUND,
                            "배송 정보를 찾을 수 없습니다: " + deliveryId));

            if (delivery.getStatus() == null || delivery.getStatus() != org.example.nareugobackend.domain.delivery.DeliveryStatus.IN_DELIVERY) {
                throw new RobotException(RobotErrorCode.DELIVERY_ADDRESS_INVALID,
                        "배송 완료 가능한 상태가 아닙니다. 현재 배송 상태: " + delivery.getStatus());
            }

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

            return DeliveryCompletionResponse.builder()
                    .message("배송 완료")
                    .updatedStatus("DELIVERY_COMPLETED")
                    .timestamp(java.time.Instant.now().toString())
                    .build();
        } catch (RobotException e) {
            throw e;
        } catch (Exception e) {
            log.error("배송 완료 처리 실패", e);
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
                "/home/hb/Git/Embedded/src/delivery.py",
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
            Delivery delivery = deliveryRepository.findByIdWithOrderAndBuyer(deliveryId)
                    .orElseThrow(() -> new RobotException(RobotErrorCode.DELIVERY_NOT_FOUND,
                            "배송 정보를 찾을 수 없습니다: " + deliveryId));

            Order order = delivery.getOrder();
            User buyer = order.getBuyer();
            User seller = getSeller(order);

            Product product = productRepository.findById(order.getProductId())
                    .orElseThrow(() -> new RobotException(RobotErrorCode.DELIVERY_ADDRESS_INVALID,
                            "상품 정보를 찾을 수 없습니다: " + order.getProductId()));

            notificationService.createSellerArrivalNotification(
                    seller.getId(),
                    product.getTitle(),
                    buyer.getName()
            );

            fcmService.sendSellerArrivalNotification(
                    seller.getId(),
                    product.getTitle(),
                    buyer.getName(),
                    deliveryId
            );
        } catch (RobotException e) {
            throw e;
        } catch (Exception e) {
            log.error("판매자 집 도착 알림 처리 실패", e);
            throw new RobotException(RobotErrorCode.DELIVERY_ADDRESS_INVALID, "판매자 집 도착 알림 처리 중 오류 발생", e);
        }
    }

    /**
     * 로봇에게 픽업 완료 신호 전송
     */
    public boolean notifyRobotPickupComplete(Long deliveryId) {
        try {
            String robotUrl = getRobotHttpUrl() + "/robot/delivery/" + deliveryId + "/seller/placed";

            HttpHeaders headers = new HttpHeaders();
            headers.setContentType(MediaType.APPLICATION_JSON);

            Map<String, Object> requestBody = new HashMap<>();
            requestBody.put("timestamp", java.time.Instant.now().toString());
            requestBody.put("deliveryId", deliveryId);

            HttpEntity<Map<String, Object>> entity = new HttpEntity<>(requestBody, headers);

            restTemplate.postForObject(robotUrl, entity, String.class);
            return true;
        } catch (Exception e) {
            log.error("로봇 픽업 완료 신호 전송 실패 - 배송 ID: {}", deliveryId, e);
            return false;
        }
    }

    /**
     * 로봇에게 구매자 수령 완료 신호 전송
     */
    public boolean notifyRobotBuyerPickupComplete(Long deliveryId) {
        try {
            String robotUrl = getRobotHttpUrl() + "/robot/delivery/" + deliveryId + "/buyer/orig_pos";

            HttpHeaders headers = new HttpHeaders();
            headers.setContentType(MediaType.APPLICATION_JSON);

            Map<String, Object> requestBody = new HashMap<>();
            requestBody.put("timestamp", java.time.Instant.now().toString());
            requestBody.put("deliveryId", deliveryId);

            HttpEntity<Map<String, Object>> entity = new HttpEntity<>(requestBody, headers);

            restTemplate.postForObject(robotUrl, entity, String.class);
            return true;
        } catch (Exception e) {
            log.error("로봇 구매자 수령 완료 신호 전송 실패 - 배송 ID: {}", deliveryId, e);
            return false;
        }
    }

    /**
     * 구매자 집 도착 알림 처리
     */
    @Transactional
    public void notifyBuyerArrival(Long deliveryId, FcmService fcmService) {
        try {
            Delivery delivery = deliveryRepository.findByIdWithOrderAndBuyer(deliveryId)
                    .orElseThrow(() -> new RobotException(RobotErrorCode.DELIVERY_NOT_FOUND,
                            "배송 정보를 찾을 수 없습니다: " + deliveryId));

            Order order = delivery.getOrder();
            User buyer = order.getBuyer();
            User seller = getSeller(order);

            Product product = productRepository.findById(order.getProductId())
                    .orElseThrow(() -> new RobotException(RobotErrorCode.DELIVERY_ADDRESS_INVALID,
                            "상품 정보를 찾을 수 없습니다: " + order.getProductId()));

            notificationService.createBuyerArrivalNotification(
                    buyer.getId(),
                    product.getTitle(),
                    seller.getName(),
                    deliveryId
            );

            fcmService.sendBuyerArrivalNotification(
                    buyer.getId(),
                    product.getTitle(),
                    seller.getName(),
                    deliveryId
            );
        } catch (RobotException e) {
            throw e;
        } catch (Exception e) {
            log.error("구매자 집 도착 알림 처리 실패", e);
            throw new RobotException(RobotErrorCode.DELIVERY_ADDRESS_INVALID, "구매자 집 도착 알림 처리 중 오류 발생", e);
        }
    }

    /**
     * 구매자 수령 완료 및 배송 완료 처리
     */
    @Transactional
    public void completeBuyerPickup(Long deliveryId) {
        try {
            Delivery delivery = deliveryRepository.findByIdWithOrderAndBuyer(deliveryId)
                    .orElseThrow(() -> new RobotException(RobotErrorCode.DELIVERY_NOT_FOUND,
                            "배송 정보를 찾을 수 없습니다: " + deliveryId));

            Order order = delivery.getOrder();

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

            Order updatedOrder = Order.builder()
                    .id(order.getId())
                    .productId(order.getProductId())
                    .buyer(order.getBuyer())
                    .status(Order.OrderStatus.DELIVERY_COMPLETED)
                    .amount(order.getAmount())
                    .createdAt(order.getCreatedAt())
                    .updatedAt(java.time.LocalDateTime.now())
                    .build();

            orderRepository.save(updatedOrder);
        } catch (RobotException e) {
            throw e;
        } catch (Exception e) {
            log.error("구매자 수령 완료 처리 실패", e);
            throw new RobotException(RobotErrorCode.DELIVERY_ADDRESS_INVALID, "구매자 수령 완료 처리 중 오류 발생", e);
        }
    }
}