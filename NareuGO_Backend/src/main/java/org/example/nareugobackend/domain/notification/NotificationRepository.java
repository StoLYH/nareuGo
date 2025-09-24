package org.example.nareugobackend.domain.notification;

import org.springframework.data.domain.Pageable;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import java.util.List;
import java.util.Optional;

@Repository
public interface NotificationRepository extends JpaRepository<Notification, Long> {

    /**
     * 사용자의 읽지 않은 알림 개수 조회
     */
    int countByUserIdAndIsReadFalse(Long userId);

    /**
     * 사용자의 모든 알림 조회 (페이징)
     */
    List<Notification> findByUserId(Long userId, Pageable pageable);

    /**
     * 사용자의 읽지 않은 알림 목록 조회
     */
    List<Notification> findByUserIdAndIsReadFalse(Long userId);

    /**
     * 특정 알림 조회 (사용자 확인)
     */
    Optional<Notification> findByIdAndUserId(Long notificationId, Long userId);
}