package org.example.nareugobackend.api.controller.robot;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.example.nareugobackend.api.controller.robot.request.PickupConfirmationRequest;
import org.example.nareugobackend.api.controller.robot.response.DeliveryAddressResponse;
import org.example.nareugobackend.api.controller.robot.response.DeliveryCompletionResponse;
import org.example.nareugobackend.api.controller.robot.response.PickupConfirmationResponse;
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
                            .status("INVALID")
                            .message("서버 오류로 인한 상태 확인 실패")
                            .timestamp(java.time.Instant.now().toString())
                            .build();
                    return ResponseEntity.internalServerError().body(errorResponse);
                });
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
}