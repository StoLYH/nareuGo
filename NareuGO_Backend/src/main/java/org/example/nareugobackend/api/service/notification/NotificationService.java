package org.example.nareugobackend.api.service.notification;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.example.nareugobackend.api.service.notification.dto.NotificationResponse;
import org.example.nareugobackend.domain.notification.Notification;
import org.example.nareugobackend.domain.notification.NotificationRepository;
import org.example.nareugobackend.domain.user.User;
import org.example.nareugobackend.domain.user.UserRepository;
import org.springframework.data.domain.PageRequest;
import org.springframework.data.domain.Pageable;
import org.springframework.data.domain.Sort;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.util.List;
import java.util.stream.Collectors;

@Slf4j
@Service
@RequiredArgsConstructor
@Transactional(readOnly = true)
public class NotificationService {

    private final NotificationRepository notificationRepository;
    private final UserRepository userRepository;

    /**
     * 사용자의 읽지 않은 알림 개수 조회
     */
    public int getUnreadCount(Long userId) {
        return notificationRepository.countByUserIdAndIsReadFalse(userId);
    }

    /**
     * 사용자의 알림 목록 조회
     */
    public List<NotificationResponse> getUserNotifications(Long userId, int page, int size) {
        Pageable pageable = PageRequest.of(page, size, Sort.by(Sort.Direction.DESC, "createdAt"));

        List<Notification> notifications = notificationRepository.findByUserId(userId, pageable);

        return notifications.stream()
                .map(this::convertToResponse)
                .collect(Collectors.toList());
    }

    /**
     * 알림 읽음 처리
     */
    @Transactional
    public boolean markAsRead(Long notificationId, Long userId) {
        return notificationRepository.findByIdAndUserId(notificationId, userId)
                .map(notification -> {
                    notification.markAsRead();
                    notificationRepository.save(notification);
                    return true;
                })
                .orElse(false);
    }

    /**
     * 모든 알림 읽음 처리
     */
    @Transactional
    public int markAllAsRead(Long userId) {
        List<Notification> unreadNotifications = notificationRepository.findByUserIdAndIsReadFalse(userId);

        unreadNotifications.forEach(Notification::markAsRead);
        notificationRepository.saveAll(unreadNotifications);

        return unreadNotifications.size();
    }

    /**
     * 알림 생성
     */
    @Transactional
    public void createNotification(Long userId, String title, String message, String type) {
        User user = userRepository.findById(userId)
                .orElseThrow(() -> new IllegalArgumentException("사용자를 찾을 수 없습니다: " + userId));

        Notification notification = Notification.builder()
                .user(user)
                .title(title)
                .message(message)
                .content(message)
                .type(type)
                .isRead(false)
                .createdAt(LocalDateTime.now())
                .build();

        notificationRepository.save(notification);
    }

    /**
     * 판매자 집 도착 알림 생성
     */
    @Transactional
    public void createSellerArrivalNotification(Long sellerId, String productTitle, String buyerName) {
        String title = "나르고가 도착했습니다!";
        String message = String.format("'%s' 상품을 로봇에 넣어주세요. 구매자: %s", productTitle, buyerName);

        createNotification(sellerId, title, message, "SELLER_ARRIVAL");
    }

    /**
     * 구매자 집 도착 알림 생성
     */
    @Transactional
    public void createBuyerArrivalNotification(Long buyerId, String productTitle, String sellerName, Long deliveryId) {
        String title = "나르고가 도착했습니다!";
        String message = String.format("'%s' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: %s", productTitle, sellerName);

        createNotification(buyerId, title, message, "BUYER_ARRIVAL");
    }

    private NotificationResponse convertToResponse(Notification notification) {
        return NotificationResponse.builder()
                .id(notification.getId())
                .title(notification.getTitle())
                .message(notification.getMessage())
                .type(notification.getType())
                .isRead(notification.isRead())
                .createdAt(notification.getCreatedAt())
                .build();
    }
}