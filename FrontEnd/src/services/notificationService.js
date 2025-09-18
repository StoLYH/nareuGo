import axios from 'axios';
import { getFCMToken, requestNotificationPermission, onMessageListener } from '../config/firebase';

class NotificationService {
  constructor() {
    this.currentToken = null;
    this.userId = null;
    this.setupForegroundListener();
  }

  // 알림 서비스 초기화
  async initialize(userId) {
    this.userId = userId;

    // 서비스 워커 등록
    await this.registerServiceWorker();

    // 알림 권한 요청
    const hasPermission = await requestNotificationPermission();
    if (!hasPermission) {
      console.log('Notification permission not granted');
      return false;
    }

    // FCM 토큰 가져오기 및 저장
    await this.getAndSaveToken();

    return true;
  }

  // 서비스 워커 등록
  async registerServiceWorker() {
    if ('serviceWorker' in navigator) {
      try {
        const registration = await navigator.serviceWorker.register('/firebase-messaging-sw.js');
        console.log('Service Worker registered successfully:', registration);
        return registration;
      } catch (error) {
        console.error('Service Worker registration failed:', error);
        throw error;
      }
    }
  }

  // FCM 토큰 가져오기 및 서버에 저장
  async getAndSaveToken() {
    try {
      const token = await getFCMToken();
      if (token && this.userId) {
        this.currentToken = token;
        await this.saveTokenToServer(token);
        console.log('FCM token saved successfully');
      }
      return token;
    } catch (error) {
      console.error('Error getting FCM token:', error);
      throw error;
    }
  }

  // 서버에 토큰 저장
  async saveTokenToServer(token) {
    try {
      await axios.post('/api/fcm/token', {
        userId: this.userId,
        token: token,
        deviceType: 'WEB'
      });
    } catch (error) {
      console.error('Error saving token to server:', error);
      throw error;
    }
  }

  // 서버에서 토큰 제거
  async removeTokenFromServer() {
    if (this.currentToken && this.userId) {
      try {
        await axios.delete('/api/fcm/token', {
          data: {
            userId: this.userId,
            token: this.currentToken
          }
        });
        console.log('FCM token removed successfully');
      } catch (error) {
        console.error('Error removing token from server:', error);
      }
    }
  }

  // 포그라운드 메시지 리스너 설정
  setupForegroundListener() {
    onMessageListener()
      .then((payload) => {
        console.log('Foreground notification received:', payload);
        this.showForegroundNotification(payload);
      })
      .catch((err) => {
        console.error('Error setting up foreground listener:', err);
      });
  }

  // 포그라운드에서 알림 표시
  showForegroundNotification(payload) {
    const { title, body } = payload.notification || {};
    const { type } = payload.data || {};

    // 브라우저 알림 API 사용
    if (Notification.permission === 'granted') {
      const notification = new Notification(title || '새 알림', {
        body: body || '',
        icon: '/icons/icon-192x192.png',
        badge: '/icons/badge-72x72.png',
        tag: type || 'default',
        data: payload.data,
        requireInteraction: true
      });

      notification.onclick = () => {
        // 알림 클릭 시 적절한 페이지로 이동
        this.handleNotificationClick(payload.data);
        notification.close();
      };

      // 자동으로 5초 후 닫기
      setTimeout(() => {
        notification.close();
      }, 5000);
    }

    // 앱 내 알림 표시 (선택사항)
    this.showInAppNotification(payload);
  }

  // 앱 내 알림 표시
  showInAppNotification(payload) {
    // Vuetify snackbar나 toast 알림으로 구현 가능
    const event = new CustomEvent('show-notification', {
      detail: {
        title: payload.notification?.title,
        body: payload.notification?.body,
        type: payload.data?.type
      }
    });
    window.dispatchEvent(event);
  }

  // 알림 클릭 처리
  handleNotificationClick(data) {
    let path = '/';

    switch (data?.type) {
      case 'LIKE':
        path = '/mypage';
        break;
      case 'PURCHASE':
        path = '/sales-history';
        break;
      case 'SALE_CONFIRMATION':
        path = '/purchase-history';
        break;
      default:
        path = '/';
    }

    // Vue Router를 통한 페이지 이동
    if (window.location.pathname !== path) {
      window.location.href = path;
    }
  }

  // 알림 서비스 종료
  async cleanup() {
    await this.removeTokenFromServer();
    this.currentToken = null;
    this.userId = null;
  }
}

// 싱글톤 인스턴스 생성
export const notificationService = new NotificationService();

export default NotificationService;