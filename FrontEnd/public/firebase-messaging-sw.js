// Firebase SDK import
importScripts('https://www.gstatic.com/firebasejs/9.0.0/firebase-app-compat.js');
importScripts('https://www.gstatic.com/firebasejs/9.0.0/firebase-messaging-compat.js');

// Firebase 설정 (실제 설정값으로 교체 필요)
const firebaseConfig = {
  apiKey: "your-api-key",
  authDomain: "your-project.firebaseapp.com",
  projectId: "your-project-id",
  storageBucket: "your-project.appspot.com",
  messagingSenderId: "your-sender-id",
  appId: "your-app-id"
};

// Firebase 초기화
firebase.initializeApp(firebaseConfig);

// Firebase Messaging 인스턴스 생성
const messaging = firebase.messaging();

// 백그라운드 메시지 처리
messaging.onBackgroundMessage((payload) => {
  console.log('Background message received:', payload);

  const notificationTitle = payload.notification?.title || '새 알림';
  const notificationOptions = {
    body: payload.notification?.body || '',
    icon: '/icons/icon-192x192.png',
    badge: '/icons/badge-72x72.png',
    tag: payload.data?.type || 'default',
    data: payload.data,
    requireInteraction: true,
    actions: [
      {
        action: 'view',
        title: '보기',
        icon: '/icons/view-icon.png'
      },
      {
        action: 'close',
        title: '닫기',
        icon: '/icons/close-icon.png'
      }
    ]
  };

  self.registration.showNotification(notificationTitle, notificationOptions);
});

// 알림 클릭 이벤트 처리
self.addEventListener('notificationclick', (event) => {
  console.log('Notification clicked:', event);

  event.notification.close();

  if (event.action === 'view') {
    // 알림 데이터에 따라 적절한 페이지로 이동
    const notificationData = event.notification.data;
    let url = '/';

    switch (notificationData?.type) {
      case 'LIKE':
        url = '/mypage';
        break;
      case 'PURCHASE':
        url = '/sales-history';
        break;
      case 'SALE_CONFIRMATION':
        url = '/purchase-history';
        break;
      default:
        url = '/';
    }

    event.waitUntil(
      clients.matchAll({ type: 'window' }).then((clientList) => {
        // 이미 열린 탭이 있는지 확인
        for (const client of clientList) {
          if (client.url.includes(url) && 'focus' in client) {
            return client.focus();
          }
        }
        // 새 탭에서 열기
        if (clients.openWindow) {
          return clients.openWindow(url);
        }
      })
    );
  }
});

// 푸시 이벤트 처리
self.addEventListener('push', (event) => {
  console.log('Push event received:', event);

  if (event.data) {
    const data = event.data.json();
    const notificationTitle = data.notification?.title || '새 알림';
    const notificationOptions = {
      body: data.notification?.body || '',
      icon: '/icons/icon-192x192.png',
      badge: '/icons/badge-72x72.png',
      data: data.data
    };

    event.waitUntil(
      self.registration.showNotification(notificationTitle, notificationOptions)
    );
  }
});