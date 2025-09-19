package org.example.nareugobackend.api.controller.robot;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.example.nareugobackend.api.controller.robot.response.DeliveryAddressResponse;
import org.example.nareugobackend.api.controller.robot.response.RobotStatusResponse;
import org.example.nareugobackend.api.service.robot.RobotService;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.concurrent.CompletableFuture;

@Slf4j
@RestController
@RequestMapping("/robot")
@RequiredArgsConstructor
public class RobotController {

    private final RobotService robotService;

    @GetMapping("/status")
    public CompletableFuture<ResponseEntity<RobotStatusResponse>> checkRobotStatus(
            @RequestParam String robotId) {

        log.info("로봇 상태 확인 요청: {}", robotId);

        return robotService.checkRobotStatus(robotId)
                .thenApply(response -> {
                    log.info("로봇 {} 상태 응답: {}", robotId, response.getStatus());
                    return ResponseEntity.ok(response);
                })
                .exceptionally(ex -> {
                    log.error("로봇 상태 확인 실패: {}", ex.getMessage());
                    RobotStatusResponse errorResponse = RobotStatusResponse.builder()
                            .status("invalid")
                            .message("서버 오류로 인한 상태 확인 실패")
                            .timestamp(java.time.Instant.now().toString())
                            .build();
                    return ResponseEntity.internalServerError().body(errorResponse);
                });
    }

    @PostMapping("/location")
    public ResponseEntity<String> requestLocation(
            @RequestParam String robotId,
            @RequestParam String destination) {

        log.info("로봇 {} 위치 요청: {}", robotId, destination);

        try {
            robotService.sendLocationRequest(robotId, destination);
            return ResponseEntity.ok("위치 요청이 전송되었습니다.");
        } catch (Exception e) {
            log.error("위치 요청 전송 실패: {}", e.getMessage());
            return ResponseEntity.internalServerError().body("위치 요청 전송에 실패했습니다.");
        }
    }

    @PostMapping("/pickup")
    public ResponseEntity<String> confirmPickup(
            @RequestParam String robotId,
            @RequestParam String orderId) {

        log.info("로봇 {} 픽업 확인: {}", robotId, orderId);

        try {
            robotService.sendPickupConfirmation(robotId, orderId);
            return ResponseEntity.ok("픽업 확인이 전송되었습니다.");
        } catch (Exception e) {
            log.error("픽업 확인 전송 실패: {}", e.getMessage());
            return ResponseEntity.internalServerError().body("픽업 확인 전송에 실패했습니다.");
        }
    }

    @PostMapping("/delivery/complete")
    public ResponseEntity<String> completeDelivery(
            @RequestParam String robotId,
            @RequestParam String orderId) {

        log.info("로봇 {} 배송 완료: {}", robotId, orderId);

        try {
            robotService.sendDeliveryComplete(robotId, orderId);
            return ResponseEntity.ok("배송 완료가 전송되었습니다.");
        } catch (Exception e) {
            log.error("배송 완료 전송 실패: {}", e.getMessage());
            return ResponseEntity.internalServerError().body("배송 완료 전송에 실패했습니다.");
        }
    }

    @GetMapping("/delivery/{deliveryId}/addresses")
    public ResponseEntity<DeliveryAddressResponse> getDeliveryAddresses(
            @PathVariable Long deliveryId) {

        log.info("로봇이 배송 {} 주소 정보 요청", deliveryId);

        try {
            DeliveryAddressResponse addresses = robotService.getDeliveryAddresses(deliveryId);
            return ResponseEntity.ok(addresses);
        } catch (Exception e) {
            log.error("배송 주소 조회 실패: {}", e.getMessage());
            return ResponseEntity.internalServerError().build();
        }
    }

    @PostMapping("/delivery/start")
    public ResponseEntity<String> startDelivery(
            @RequestParam String robotId,
            @RequestParam Long deliveryId) {

        log.info("로봇 {} 배송 시작 요청: 배송 ID {}", robotId, deliveryId);

        try {
            // 1. 로봇 상태 확인
            CompletableFuture<RobotStatusResponse> statusCheck = robotService.checkRobotStatus(robotId);
            RobotStatusResponse status = statusCheck.get();

            if (!"valid".equals(status.getStatus())) {
                log.warn("로봇 {} 상태 불량으로 배송 시작 불가: {}", robotId, status.getMessage());
                return ResponseEntity.badRequest().body("로봇 상태 불량: " + status.getMessage());
            }

            // 2. 배송 주소 정보 조회
            DeliveryAddressResponse addresses = robotService.getDeliveryAddresses(deliveryId);

            // 3. ROS2에 배송 시작 명령 전송
            robotService.sendDeliveryStartCommand(robotId, deliveryId, addresses);

            log.info("로봇 {} 배송 시작 명령 전송 완료: 배송 ID {}", robotId, deliveryId);

            return ResponseEntity.ok("배송이 시작되었습니다. 로봇이 픽업 위치로 이동합니다.");

        } catch (Exception e) {
            log.error("배송 시작 실패: {}", e.getMessage());
            return ResponseEntity.internalServerError()
                    .body("배송 시작에 실패했습니다: " + e.getMessage());
        }
    }
}