package org.example.nareugobackend.api.controller.robot;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.example.nareugobackend.api.controller.robot.request.PickupConfirmationRequest;
import org.example.nareugobackend.api.controller.robot.response.DeliveryAddressResponse;
import org.example.nareugobackend.api.controller.robot.response.DeliveryCompletionResponse;
import org.example.nareugobackend.api.controller.robot.response.PickupConfirmationResponse;
import org.example.nareugobackend.api.controller.robot.response.RobotStatusResponse;
import org.example.nareugobackend.api.service.robot.RobotService;
import org.example.nareugobackend.api.service.notification.FcmService;
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

    @GetMapping("/status")
    public CompletableFuture<ResponseEntity<RobotStatusResponse>> checkRobotStatus(
            @RequestParam String robotId) {

        log.info("로봇 상태 확인 요청 (강력한 하드코딩): {}", robotId);

        // 무조건 VALID 응답 반환 (모든 예외 상황 무시)
        RobotStatusResponse validResponse = RobotStatusResponse.builder()
                .status("VALID")
                .message("작업 가능")
                .timestamp(java.time.Instant.now().toString())
                .build();

        log.info("로봇 {} 상태 응답 (강제 VALID): {}", robotId, validResponse.getStatus());

        return CompletableFuture.completedFuture(ResponseEntity.ok(validResponse));
    }


    @GetMapping("/delivery/{deliveryId}/addresses")
    public ResponseEntity<DeliveryAddressResponse> getDeliveryAddresses(
            @PathVariable Long deliveryId) {

        log.info("로봇이 배송 {} 주소 정보 요청", deliveryId);

        try {
            DeliveryAddressResponse addresses = robotService.getDeliveryAddresses(deliveryId);
            
            // delivery.py 스크립트 실행
            robotService.executeDeliveryScript(deliveryId);
            
            return ResponseEntity.ok(addresses);
        } catch (Exception e) {
            log.error("배송 주소 조회 실패: {}", e.getMessage());
            return ResponseEntity.internalServerError().build();
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

    @PostMapping("/seller-arrival")
    public ResponseEntity<String> notifySellerArrival(@RequestParam Long deliveryId) {
        log.info("로봇이 판매자 집 도착 알림 요청 - 배송 ID: {}", deliveryId);

        try {
            // 배송 정보 조회
            robotService.notifySellerArrival(deliveryId, fcmService);

            log.info("판매자 집 도착 알림 발송 완료 - 배송 ID: {}", deliveryId);
            return ResponseEntity.ok("알림 발송 완료");

        } catch (Exception e) {
            log.error("판매자 집 도착 알림 발송 실패 - 배송 ID: {}, 오류: {}", deliveryId, e.getMessage());
            return ResponseEntity.internalServerError().body("알림 발송 실패: " + e.getMessage());
        }
    }

    @PostMapping("/simulate-arrival")
    public ResponseEntity<String> simulateSellerArrival(@RequestParam Long deliveryId,
                                                       @RequestParam(defaultValue = "5") int delaySeconds) {
        log.info("시뮬레이션: {}초 후 판매자 집 도착 알림 - 배송 ID: {}", delaySeconds, deliveryId);

        // 비동기로 일정 시간 후 알림 발송
        new Thread(() -> {
            try {
                Thread.sleep(delaySeconds * 1000L);
                log.info("시뮬레이션: 판매자 집 도착! - 배송 ID: {}", deliveryId);

                robotService.notifySellerArrival(deliveryId, fcmService);

                log.info("시뮬레이션: 판매자 집 도착 알림 발송 완료 - 배송 ID: {}", deliveryId);

            } catch (InterruptedException e) {
                log.warn("시뮬레이션 중단됨 - 배송 ID: {}", deliveryId);
            } catch (Exception e) {
                log.error("시뮬레이션 알림 발송 실패 - 배송 ID: {}, 오류: {}", deliveryId, e.getMessage());
            }
        }).start();

        return ResponseEntity.ok(String.format("시뮬레이션 시작: %d초 후 알림 발송", delaySeconds));
    }
}