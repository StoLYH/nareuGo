// Firebase 메시징 Service Worker
// 백그라운드 메시지 수신을 위한 Service Worker

// Firebase SDK 초기화
importScripts('https://www.gstatic.com/firebasejs/10.7.1/firebase-app-compat.js');
importScripts('https://www.gstatic.com/firebasejs/10.7.1/firebase-messaging-compat.js');

// Firebase 설정 (config/firebase.js와 동일해야 함)
const firebaseConfig = {
  apiKey: "AIzaSyAjuwpKSdrcpA3zG3B1scbogbNnq9jBNy0",
  authDomain: "nareugo.firebaseapp.com",
  projectId: "nareugo",
  storageBucket: "nareugo.firebasestorage.app",
  messagingSenderId: "922768085258",
  appId: "1:922768085258:web:9a574b1cc6f5de57f92af3",
  measurementId: "G-DWZ3P4VKJS"
};

// Firebase 앱 초기화
firebase.initializeApp(firebaseConfig);

// 메시징 인스턴스 가져오기
const messaging = firebase.messaging();

// 백그라운드 메시지 처리
messaging.onBackgroundMessage((payload) => {
  console.log('[firebase-messaging-sw.js] 백그라운드 메시지 수신:', payload);

  // 메인 앱에 메시지 전달 (포그라운드 상태일 때)
  self.clients.matchAll({ type: 'window', includeUncontrolled: true }).then((clients) => {
    clients.forEach((client) => {
      console.log('[firebase-messaging-sw.js] 메인 앱으로 메시지 전달:', payload);
      client.postMessage({
        type: 'FCM_MESSAGE',
        payload: payload
      });
    });
  });

  const notificationTitle = payload.notification?.title || 'NareuGO 알림';
  const notificationOptions = {
    body: payload.notification?.body || '새로운 알림이 있습니다.',
    icon: payload.notification?.icon || '/icons/icon-192x192.png',
    badge: payload.notification?.badge || '/icons/badge-72x72.png',
    tag: payload.data?.type || 'default',
    data: payload.data || {},
    actions: [
      {
        action: 'open',
        title: '확인'
      },
      {
        action: 'close',
        title: '닫기'
      }
    ],
    requireInteraction: true
  };

  // 알림 표시
  self.registration.showNotification(notificationTitle, notificationOptions);
});

// 알림 클릭 처리
self.addEventListener('notificationclick', (event) => {
  console.log('[firebase-messaging-sw.js] 알림 클릭:', event);

  event.notification.close();

  if (event.action === 'close') {
    return;
  }

  // 앱 열기 또는 포커스
  event.waitUntil(
    clients.matchAll({ type: 'window', includeUncontrolled: true }).then((clientList) => {
      // 알림 타입별 라우팅
      let targetUrl = self.location.origin;

      if (event.notification.data?.type) {
        switch (event.notification.data.type) {
          case 'LIKE':
            if (event.notification.data.productId) {
              targetUrl = `${self.location.origin}/product/${event.notification.data.productId}`;
            }
            break;
          case 'PURCHASE':
            targetUrl = `${self.location.origin}/my/sales`;
            break;
          case 'SALE_CONFIRMATION':
            targetUrl = `${self.location.origin}/my/purchases`;
            break;
          case 'CHAT':
            targetUrl = `${self.location.origin}/chat`;
            break;
          case 'DELIVERY_SELLER_ARRIVED':
          case 'ROBOT_ARRIVAL':
          case 'ROBOT_ARRIVAL_SELLER':
            // 판매자에게 로봇 도착 알림 - 메인 페이지로 이동
            targetUrl = `${self.location.origin}/`;
            break;
          case 'DELIVERY_BUYER_ARRIVED':
          case 'ROBOT_ARRIVAL_BUYER':
          case 'DELIVERY_ROBOT_ARRIVED_BUYER':
            // 구매자에게 로봇 도착 알림 - 메인 페이지로 이동
            targetUrl = `${self.location.origin}/`;
            break;
        }
      }

      // 이미 열린 창이 있으면 포커스하고 메시지 전달
      for (const client of clientList) {
        if (client.url.includes(self.location.origin) && 'focus' in client) {
          // 로봇 도착 알림의 경우 메인 앱에 메시지 전달
          if (event.notification.data?.type &&
              (event.notification.data.type === 'ROBOT_ARRIVAL' ||
               event.notification.data.type === 'ROBOT_ARRIVAL_SELLER' ||
               event.notification.data.type === 'ROBOT_ARRIVAL_BUYER' ||
               event.notification.data.type === 'DELIVERY_SELLER_ARRIVED' ||
               event.notification.data.type === 'DELIVERY_BUYER_ARRIVED' ||
               event.notification.data.type === 'DELIVERY_ROBOT_ARRIVED_BUYER')) {
            client.postMessage({
              type: 'FCM_MESSAGE',
              payload: {
                notification: {
                  title: event.notification.title,
                  body: event.notification.body
                },
                data: event.notification.data
              }
            });
          }
          return client.focus();
        }
      }

      // 새 창 열기
      if (clients.openWindow) {
        return clients.openWindow(targetUrl);
      }
    })
  );
});

// Service Worker 설치
self.addEventListener('install', (event) => {
  console.log('[firebase-messaging-sw.js] Service Worker 설치됨');
  self.skipWaiting();
});

// Service Worker 활성화
self.addEventListener('activate', (event) => {
  console.log('[firebase-messaging-sw.js] Service Worker 활성화됨');
  event.waitUntil(self.clients.claim());
});