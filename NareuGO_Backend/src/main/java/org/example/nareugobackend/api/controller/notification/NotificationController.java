package org.example.nareugobackend.api.controller.notification;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.example.nareugobackend.api.service.notification.NotificationService;
import org.example.nareugobackend.api.service.notification.dto.NotificationResponse;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;
import java.util.Map;

@Slf4j
@RestController
@RequestMapping("/notifications")
@RequiredArgsConstructor
public class NotificationController {

    private final NotificationService notificationService;

    /**
     * 사용자의 읽지 않은 알림 개수 조회
     */
    @GetMapping("/unread-count")
    public ResponseEntity<Map<String, Object>> getUnreadCount(@RequestParam Long userId) {
        try {
            int unreadCount = notificationService.getUnreadCount(userId);

            return ResponseEntity.ok(Map.of(
                "success", true,
                "unreadCount", unreadCount
            ));

        } catch (Exception e) {
            log.error("읽지 않은 알림 개수 조회 실패 - userId: {}, 오류: {}", userId, e.getMessage());
            return ResponseEntity.internalServerError()
                .body(Map.of("success", false, "error", e.getMessage()));
        }
    }

    /**
     * 사용자의 모든 알림 목록 조회
     */
    @GetMapping("/list")
    public ResponseEntity<Map<String, Object>> getNotifications(@RequestParam Long userId,
                                                              @RequestParam(defaultValue = "0") int page,
                                                              @RequestParam(defaultValue = "20") int size) {
        try {
            List<NotificationResponse> notifications = notificationService.getUserNotifications(userId, page, size);

            return ResponseEntity.ok(Map.of(
                "success", true,
                "notifications", notifications
            ));

        } catch (Exception e) {
            log.error("알림 목록 조회 실패 - userId: {}, 오류: {}", userId, e.getMessage());
            return ResponseEntity.internalServerError()
                .body(Map.of("success", false, "error", e.getMessage()));
        }
    }

    /**
     * 알림 읽음 처리
     */
    @PostMapping("/{notificationId}/read")
    public ResponseEntity<Map<String, Object>> markAsRead(@PathVariable Long notificationId,
                                                         @RequestParam Long userId) {
        try {
            boolean success = notificationService.markAsRead(notificationId, userId);

            return ResponseEntity.ok(Map.of(
                "success", success,
                "message", success ? "알림이 읽음 처리되었습니다." : "알림 처리에 실패했습니다."
            ));

        } catch (Exception e) {
            log.error("알림 읽음 처리 실패 - notificationId: {}, userId: {}, 오류: {}", notificationId, userId, e.getMessage());
            return ResponseEntity.internalServerError()
                .body(Map.of("success", false, "error", e.getMessage()));
        }
    }

    /**
     * 모든 알림 읽음 처리
     */
    @PostMapping("/read-all")
    public ResponseEntity<Map<String, Object>> markAllAsRead(@RequestParam Long userId) {
        try {
            int readCount = notificationService.markAllAsRead(userId);

            return ResponseEntity.ok(Map.of(
                "success", true,
                "readCount", readCount,
                "message", readCount + "개의 알림이 읽음 처리되었습니다."
            ));

        } catch (Exception e) {
            log.error("모든 알림 읽음 처리 실패 - userId: {}, 오류: {}", userId, e.getMessage());
            return ResponseEntity.internalServerError()
                .body(Map.of("success", false, "error", e.getMessage()));
        }
    }

    /**
     * 테스트용: 샘플 알림 생성
     */
    @PostMapping("/test-notification")
    public ResponseEntity<Map<String, Object>> createTestNotification(@RequestParam Long userId,
                                                                     @RequestParam(defaultValue = "테스트 알림") String title,
                                                                     @RequestParam(defaultValue = "이것은 테스트 알림입니다.") String message) {
        try {
            notificationService.createNotification(userId, title, message, "TEST");

            return ResponseEntity.ok(Map.of(
                "success", true,
                "message", "테스트 알림이 생성되었습니다."
            ));

        } catch (Exception e) {
            log.error("테스트 알림 생성 실패 - userId: {}, 오류: {}", userId, e.getMessage());
            return ResponseEntity.internalServerError()
                .body(Map.of("success", false, "error", e.getMessage()));
        }
    }
}