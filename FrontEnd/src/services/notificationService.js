// 알림 서비스 (Firebase 비활성화됨 - HTTP 환경에서는 지원되지 않음)

class NotificationService {
  constructor() {
    this.currentToken = null;
    this.userId = null;
    console.log('NotificationService: Firebase messaging disabled in HTTP environment');
  }

  // 알림 서비스 초기화 (비활성화됨)
  async initialize(userId) {
    this.userId = userId;
    console.log('NotificationService: Notification service disabled');
    return false; // 항상 false 반환
  }

  // 서버에 토큰 저장 (비활성화됨)
  async saveTokenToServer(token) {
    console.log('NotificationService: Token saving disabled');
    return false;
  }

  // 서버에서 토큰 제거 (비활성화됨)
  async removeTokenFromServer() {
    console.log('NotificationService: Token removal disabled');
    return false;
  }

  // 포그라운드 메시지 리스너 설정 (비활성화됨)
  setupForegroundListener() {
    console.log('NotificationService: Foreground listener disabled');
  }

  // 포그라운드에서 알림 표시 (비활성화됨)
  showForegroundNotification(payload) {
    console.log('NotificationService: Foreground notification disabled');
  }

  // 앱 내 알림 표시 (기본 브라우저 알림만 사용)
  showInAppNotification(payload) {
    // 기본 브라우저 알림만 사용 (Firebase 없이)
    if (Notification.permission === 'granted') {
      const notification = new Notification(payload.title || '새 알림', {
        body: payload.body || '',
        icon: '/images/logo.png'
      });

      // 자동으로 5초 후 닫기
      setTimeout(() => {
        notification.close();
      }, 5000);
    }
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
    console.log('NotificationService: Cleanup completed');
    this.currentToken = null;
    this.userId = null;
  }
}

// 싱글톤 인스턴스 생성
export const notificationService = new NotificationService();

export default NotificationService;