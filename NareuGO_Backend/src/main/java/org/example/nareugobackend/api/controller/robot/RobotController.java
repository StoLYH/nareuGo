package org.example.nareugobackend.api.controller.robot;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.example.nareugobackend.api.controller.robot.request.PickupConfirmationRequest;
import org.example.nareugobackend.api.controller.robot.response.DeliveryAddressResponse;
import org.example.nareugobackend.api.controller.robot.response.DeliveryCompletionResponse;
import org.example.nareugobackend.api.controller.robot.response.PickupConfirmationResponse;
import org.example.nareugobackend.api.controller.robot.response.RobotStatusResponse;
import org.example.nareugobackend.api.controller.robot.response.SellerArrivedResponse;
import org.example.nareugobackend.api.controller.robot.response.BuyerArrivedResponse;
import org.example.nareugobackend.api.controller.robot.response.BuyerPickupCompleteResponse;
import org.example.nareugobackend.api.service.robot.RobotService;
import org.example.nareugobackend.api.service.notification.FcmService;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.concurrent.CompletableFuture;

@Slf4j
@RestController
@RequestMapping("/robot")
@RequiredArgsConstructor
public class RobotController {

    private final RobotService robotService;
    private final FcmService fcmService;

    @Value("${server.env}")
    private String activeProfile;

    private String getRobotHttpUrl() {
        if ("local".equals(activeProfile)) {
            return "http://localhost:8888";
        } else {
            return "https://unfearful-orion-furuncular.ngrok-free.dev";
        }
    }


    @GetMapping("/status")
    public CompletableFuture<ResponseEntity<RobotStatusResponse>> checkRobotStatus(
            @RequestParam String robotId,
            @RequestParam(required = false) Long delivery_id) {

        log.info("1. 로봇 상태확인 [프론트 -> 백앤드]");

        log.info("로봇 상태 확인 요청: robotId={}, delivery_id={}", robotId, delivery_id);

        return robotService.checkRobotStatus(robotId, delivery_id)
                .thenApply(response -> {
                    log.info("RobotService 응답: {}", response);
                    return ResponseEntity.ok(response);
                })
                .exceptionally(throwable -> {
                    log.error("로봇 상태 확인 실패: {}", throwable.getMessage());
                    RobotStatusResponse errorResponse = RobotStatusResponse.builder()
                            .status("error")
                            .message("로봇 통신 오류: " + throwable.getMessage())
                            .timestamp(java.time.Instant.now().toString())
                            .build();
                    return ResponseEntity.ok(errorResponse);
                });
    }


    @GetMapping("/delivery/{deliveryId}/addresses")
    public ResponseEntity<DeliveryAddressResponse> getDeliveryAddresses(
            @PathVariable Long deliveryId) {

        log.info("로봇이 배송 주소 정보 요청 - deliveryId: {}", deliveryId);

        try {
            log.info("배송 주소 조회 시작 - deliveryId: {}", deliveryId);

            // 실제 배송 정보에서 주소를 조회
            DeliveryAddressResponse addresses = robotService.getDeliveryAddresses(deliveryId);
            log.info("배송 주소 조회 성공: {}", addresses);

            // 로봇 서버로 주소 정보와 함께 직접 요청 (첫 번째 로직)
            try {
                String robotUrl = getRobotHttpUrl() + "/robot/delivery/1/addresses" +
                        "?sellerAddress=" + addresses.getSellerAddress() +
                        "&buyerAddress=" + addresses.getBuyerAddress();
                log.info("로봇 서버로 직접 요청 전송: {}", robotUrl);
                // RestTemplate 인스턴스 필요
                org.springframework.web.client.RestTemplate restTemplate = new org.springframework.web.client.RestTemplate();
                restTemplate.getForObject(robotUrl, String.class);
            } catch (Exception e) {
                log.error("로봇 서버 직접 요청 실패: {}", e.getMessage());
            }

            

            // 로봇 서버(8888포트)로 배송 시작 명령 전송
            log.info("로봇 서버로 배송 시작 명령 전송 - deliveryId: {}", deliveryId);
            robotService.sendDeliveryStartCommand("1", deliveryId, addresses);

            // robot_delivery.py가 HTTP 서버로 실행 중이므로 별도 스크립트 실행 불필요
            log.info("로봇 서버로 명령 전송 완료");

            return ResponseEntity.ok(addresses);
        } catch (Exception e) {
            log.error("배송 주소 조회 실패 - deliveryId: {}, 에러: {}, 스택트레이스:", deliveryId, e.getMessage(), e);
            return ResponseEntity.status(400).body(null);
        }
    }


    @PostMapping("/delivery/{deliveryId}/pickup")
    public ResponseEntity<PickupConfirmationResponse> confirmPickupFromDelivery(
            @PathVariable Long deliveryId) {

        log.info("로봇이 배송 {} 픽업 확인 요청", deliveryId);

        try {
            PickupConfirmationResponse response = robotService.confirmPickup(deliveryId);
            return ResponseEntity.ok(response);
        } catch (Exception e) {
            log.error("픽업 확인 처리 실패: {}", e.getMessage());
            return ResponseEntity.internalServerError().build();
        }
    }

    @PostMapping("/delivery/{deliveryId}/complete")
    public ResponseEntity<DeliveryCompletionResponse> completeDeliveryFromDelivery(
            @PathVariable Long deliveryId) {

        log.info("로봇이 배송 {} 완료 요청", deliveryId);

        try {
            DeliveryCompletionResponse response = robotService.completeDelivery(deliveryId);
            log.info("배송 완료 처리 성공 - 배송 ID: {}, 상태: {}, 메시지: {}",
                    deliveryId, response.getUpdatedStatus(), response.getMessage());
            return ResponseEntity.ok(response);
        } catch (Exception e) {
            log.error("배송 완료 처리 실패: {}", e.getMessage());
            return ResponseEntity.internalServerError().build();
        }
    }

    @PostMapping("/delivery/{deliveryId}/seller/arrived")
    public ResponseEntity<SellerArrivedResponse> notifySellerArrival(@PathVariable Long deliveryId) {
        log.info("로봇이 판매자 위치 도착 알림 요청 - 배송 ID: {}", deliveryId);

        try {
            robotService.notifySellerArrival(deliveryId, fcmService);

            SellerArrivedResponse response = SellerArrivedResponse.builder()
                    .timestamp(java.time.Instant.now().toString())
                    .build();

            log.info("판매자 위치 도착 알림 발송 완료 - 배송 ID: {}", deliveryId);
            return ResponseEntity.ok(response);

        } catch (Exception e) {
            log.error("판매자 위치 도착 알림 발송 실패 - 배송 ID: {}, 오류: {}", deliveryId, e.getMessage());
            return ResponseEntity.internalServerError().build();
        }
    }

    @PostMapping("/delivery/{deliveryId}/seller/placed")
    public ResponseEntity<PickupConfirmationResponse> notifyPickupPlaced(@PathVariable Long deliveryId) {
        log.info("웹에서 픽업 완료 알림 수신 - 배송 ID: {}", deliveryId);

        try {
            // 로봇에게 픽업 완료 신호 전송
            boolean success = robotService.notifyRobotPickupComplete(deliveryId);

            PickupConfirmationResponse response = PickupConfirmationResponse.builder()
                    .timestamp(java.time.Instant.now().toString())
                    .build();

            if (success) {
                log.info("로봇에게 픽업 완료 신호 전송 성공 - 배송 ID: {}", deliveryId);
                return ResponseEntity.ok(response);
            } else {
                log.warn("로봇에게 픽업 완료 신호 전송 실패 - 배송 ID: {}", deliveryId);
                return ResponseEntity.status(500).body(response);
            }

        } catch (Exception e) {
            log.error("픽업 완료 처리 실패 - 배송 ID: {}, 오류: {}", deliveryId, e.getMessage());
            return ResponseEntity.internalServerError().build();
        }
    }

    @PostMapping("/delivery/{deliveryId}/buyer/arrived")
    public ResponseEntity<BuyerArrivedResponse> notifyBuyerArrival(@PathVariable Long deliveryId) {
        log.info("로봇이 구매자 위치 도착 알림 요청 - 배송 ID: {}", deliveryId);

        try {
            // 구매자에게 FCM 알림 발송
            robotService.notifyBuyerArrival(deliveryId, fcmService);

            BuyerArrivedResponse response = BuyerArrivedResponse.builder()
                    .timestamp(java.time.Instant.now().toString())
                    .build();

            log.info("구매자 위치 도착 알림 발송 완료 - 배송 ID: {}", deliveryId);
            return ResponseEntity.ok(response);

        } catch (Exception e) {
            log.error("구매자 위치 도착 알림 발송 실패 - 배송 ID: {}, 오류: {}", deliveryId, e.getMessage());
            return ResponseEntity.internalServerError().build();
        }
    }

    @PostMapping("/delivery/{deliveryId}/buyer/orig_pos")
    public ResponseEntity<BuyerPickupCompleteResponse> notifyBuyerPickupComplete(@PathVariable Long deliveryId) {
        log.info("로봇이 구매자 수령 완료 및 원점 복귀 알림 - 배송 ID: {}", deliveryId);

        try {
            // 배송 완료 처리
            robotService.completeBuyerPickup(deliveryId);

            // 로봇에게 구매자 수령 완료 신호 전송
            boolean success = robotService.notifyRobotBuyerPickupComplete(deliveryId);

            BuyerPickupCompleteResponse response = BuyerPickupCompleteResponse.builder()
                    .timestamp(java.time.Instant.now().toString())
                    .build();

            if (success) {
                log.info("로봇에게 구매자 수령 완료 신호 전송 성공 - 배송 ID: {}", deliveryId);
            } else {
                log.warn("로봇에게 구매자 수령 완료 신호 전송 실패 - 배송 ID: {}", deliveryId);
            }

            log.info("구매자 수령 완료 및 배송 완료 처리 성공 - 배송 ID: {}", deliveryId);
            return ResponseEntity.ok(response);

        } catch (Exception e) {
            log.error("구매자 수령 완료 처리 실패 - 배송 ID: {}, 오류: {}", deliveryId, e.getMessage());
            return ResponseEntity.internalServerError().build();
        }
    }
}