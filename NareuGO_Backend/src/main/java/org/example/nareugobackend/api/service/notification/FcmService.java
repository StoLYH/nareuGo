package org.example.nareugobackend.api.service.notification;

import com.google.firebase.messaging.FirebaseMessaging;
import com.google.firebase.messaging.Message;
import com.google.firebase.messaging.Notification;
import com.google.firebase.messaging.WebpushConfig;
import com.google.firebase.messaging.WebpushNotification;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.example.nareugobackend.domain.notification.FcmToken;
import org.example.nareugobackend.domain.notification.FcmTokenRepository;
import org.example.nareugobackend.domain.user.User;
import org.example.nareugobackend.domain.user.UserRepository;
import org.example.nareugobackend.common.exception.user.UserException;
import org.example.nareugobackend.common.exception.user.UserErrorCode;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.autoconfigure.condition.ConditionalOnBean;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

@Slf4j
@Service
@RequiredArgsConstructor
@Transactional
public class FcmService {

    private final FcmTokenRepository fcmTokenRepository;
    private final UserRepository userRepository;

    // NotificationService 의존성 주입은 순환 참조를 피하기 위해 별도 메서드로 처리

    @Autowired(required = false)
    private FirebaseMessaging firebaseMessaging;

    public void saveToken(Long userId, String token, String deviceType) {
        User user = userRepository.findById(userId)
            .orElseThrow(() -> new UserException(UserErrorCode.USER_NOT_FOUND));

        // 기존 토큰이 있으면 업데이트, 없으면 새로 생성
        fcmTokenRepository.findByUserIdAndTokenAndIsActiveTrue(userId, token)
            .ifPresentOrElse(
                existingToken -> existingToken.updateToken(token),
                () -> {
                    FcmToken fcmToken = FcmToken.create(user, token, deviceType);
                    fcmTokenRepository.save(fcmToken);
                }
            );
    }

    public void removeToken(Long userId, String token) {
        fcmTokenRepository.deleteByUserIdAndToken(userId, token);
    }

    public void sendNotificationToUser(Long userId, String title, String body, Map<String, String> data) {
        List<FcmToken> tokens = fcmTokenRepository.findByUserIdAndIsActiveTrue(userId);

        for (FcmToken fcmToken : tokens) {
            sendNotification(fcmToken.getToken(), title, body, data);
        }
    }

    public void sendLikeNotification(Long sellerId, String productTitle, String likerName) {
        String title = "새로운 관심 표시!";
        String body = String.format("%s님이 '%s' 상품에 관심을 표시했습니다.", likerName, productTitle);

        Map<String, String> data = new HashMap<>();
        data.put("type", "LIKE");
        data.put("productTitle", productTitle);
        data.put("likerName", likerName);

        sendNotificationToUser(sellerId, title, body, data);
    }

    public void sendPurchaseNotification(Long sellerId, String productTitle, String buyerName) {
        String title = "상품이 판매되었습니다!";
        String body = String.format("%s님이 '%s' 상품을 구매했습니다.", buyerName, productTitle);

        Map<String, String> data = new HashMap<>();
        data.put("type", "PURCHASE");
        data.put("productTitle", productTitle);
        data.put("buyerName", buyerName);

        sendNotificationToUser(sellerId, title, body, data);
    }

    public void sendSaleConfirmationNotification(Long buyerId, String productTitle) {
        String title = "구매가 완료되었습니다!";
        String body = String.format("'%s' 상품 구매가 완료되었습니다.", productTitle);

        Map<String, String> data = new HashMap<>();
        data.put("type", "SALE_CONFIRMATION");
        data.put("productTitle", productTitle);

        sendNotificationToUser(buyerId, title, body, data);
    }

    public void sendSellerArrivalNotification(Long sellerId, String productTitle, String buyerName, Long deliveryId) {
        String title = "나르고가 도착했습니다!";
        String body = String.format("'%s' 상품을 로봇에 넣어주세요. 구매자: %s", productTitle, buyerName);

        Map<String, String> data = new HashMap<>();
        data.put("type", "DELIVERY_ROBOT_ARRIVED");
        data.put("productTitle", productTitle);
        data.put("buyerName", buyerName);
        data.put("deliveryId", String.valueOf(deliveryId));

        sendNotificationToUser(sellerId, title, body, data);
    }

    public void sendBuyerArrivalNotification(Long buyerId, String productTitle, String sellerName, Long deliveryId) {
        String title = "나르고가 도착했습니다!";
        String body = String.format("'%s' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: %s", productTitle, sellerName);

        Map<String, String> data = new HashMap<>();
        data.put("type", "DELIVERY_ROBOT_ARRIVED_BUYER");
        data.put("productTitle", productTitle);
        data.put("sellerName", sellerName);
        data.put("deliveryId", String.valueOf(deliveryId));

        sendNotificationToUser(buyerId, title, body, data);
    }

    private void sendNotification(String token, String title, String body, Map<String, String> data) {
        if (firebaseMessaging == null) {
            return;
        }

        try {
            // 웹 푸시 알림 설정
            WebpushNotification webpushNotification = WebpushNotification.builder()
                .setTitle(title)
                .setBody(body)
                .setIcon("/icons/icon-192x192.png")
                .setBadge("/icons/badge-72x72.png")
                .build();

            WebpushConfig webpushConfig = WebpushConfig.builder()
                .setNotification(webpushNotification)
                .putAllData(data != null ? data : new HashMap<>())
                .build();

            // 기본 알림 설정
            Notification notification = Notification.builder()
                .setTitle(title)
                .setBody(body)
                .build();

            // 메시지 구성
            Message message = Message.builder()
                .setToken(token)
                .setNotification(notification)
                .setWebpushConfig(webpushConfig)
                .putAllData(data != null ? data : new HashMap<>())
                .build();

            String response = firebaseMessaging.send(message);

        } catch (Exception e) {
            // 토큰이 만료되었을 경우 비활성화
            fcmTokenRepository.findByTokenAndIsActiveTrue(token)
                .forEach(FcmToken::deactivate);
        }
    }
}