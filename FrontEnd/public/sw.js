// Service Worker for NareuGO PWA
const CACHE_NAME = 'nareugo-v1';
const STATIC_CACHE_NAME = 'nareugo-static-v1';
const DYNAMIC_CACHE_NAME = 'nareugo-dynamic-v1';

// 캐시할 정적 파일들
const STATIC_FILES = [
  '/',
  '/index.html',
  '/manifest.json',
  '/images/logo.png',
  '/images/social/사용자더미.png',
  '/images/social/상품더미1.png',
  '/images/social/상품더미2.png',
  // CSS, JS 파일들은 빌드 시 동적으로 생성되므로 runtime에 캐시
];

// API 엔드포인트들 (오프라인 시 캐시된 데이터 제공)
const API_CACHE_PATTERNS = [
  /^https?:\/\/.*\/api\//,
  /^https?:\/\/.*\/products/,
  /^https?:\/\/.*\/user/,
  /^https?:\/\/.*\/chat/,
];

// 설치 이벤트
self.addEventListener('install', (event) => {
  console.log('Service Worker: Installing...');
  
  event.waitUntil(
    caches.open(STATIC_CACHE_NAME)
      .then((cache) => {
        console.log('Service Worker: Caching static files');
        return cache.addAll(STATIC_FILES);
      })
      .then(() => {
        console.log('Service Worker: Static files cached successfully');
        return self.skipWaiting();
      })
      .catch((error) => {
        console.error('Service Worker: Error caching static files:', error);
      })
  );
});

// 활성화 이벤트
self.addEventListener('activate', (event) => {
  console.log('Service Worker: Activating...');
  
  event.waitUntil(
    caches.keys()
      .then((cacheNames) => {
        return Promise.all(
          cacheNames.map((cacheName) => {
            // 이전 버전의 캐시 삭제
            if (cacheName !== STATIC_CACHE_NAME && cacheName !== DYNAMIC_CACHE_NAME) {
              console.log('Service Worker: Deleting old cache:', cacheName);
              return caches.delete(cacheName);
            }
          })
        );
      })
      .then(() => {
        console.log('Service Worker: Activated successfully');
        return self.clients.claim();
      })
  );
});

// Fetch 이벤트 (네트워크 요청 가로채기)
self.addEventListener('fetch', (event) => {
  const { request } = event;
  const url = new URL(request.url);

  // HTML 페이지 요청 처리
  if (request.headers.get('accept').includes('text/html')) {
    event.respondWith(
      fetch(request)
        .then((response) => {
          // 네트워크 요청 성공 시 캐시에 저장
          const responseClone = response.clone();
          caches.open(DYNAMIC_CACHE_NAME)
            .then((cache) => {
              cache.put(request, responseClone);
            });
          return response;
        })
        .catch(() => {
          // 네트워크 실패 시 캐시에서 반환
          return caches.match(request)
            .then((response) => {
              return response || caches.match('/index.html');
            });
        })
    );
    return;
  }

  // API 요청 처리
  const isApiRequest = API_CACHE_PATTERNS.some(pattern => pattern.test(request.url));
  if (isApiRequest) {
    event.respondWith(
      // Network First 전략 (최신 데이터 우선)
      fetch(request)
        .then((response) => {
          if (response.ok) {
            const responseClone = response.clone();
            caches.open(DYNAMIC_CACHE_NAME)
              .then((cache) => {
                cache.put(request, responseClone);
              });
          }
          return response;
        })
        .catch(() => {
          // 네트워크 실패 시 캐시된 데이터 반환
          return caches.match(request)
            .then((response) => {
              if (response) {
                console.log('Service Worker: Serving from cache:', request.url);
                return response;
              }
              // 캐시된 데이터도 없으면 오프라인 페이지 반환
              return new Response(
                JSON.stringify({ 
                  error: 'Offline', 
                  message: '오프라인 상태입니다. 네트워크 연결을 확인해주세요.' 
                }),
                {
                  status: 503,
                  headers: { 'Content-Type': 'application/json' }
                }
              );
            });
        })
    );
    return;
  }

  // 정적 파일 요청 처리 (이미지, CSS, JS 등)
  event.respondWith(
    // Cache First 전략 (캐시 우선)
    caches.match(request)
      .then((response) => {
        if (response) {
          return response;
        }
        
        return fetch(request)
          .then((response) => {
            // 성공적인 응답만 캐시
            if (response.ok) {
              const responseClone = response.clone();
              caches.open(DYNAMIC_CACHE_NAME)
                .then((cache) => {
                  cache.put(request, responseClone);
                });
            }
            return response;
          });
      })
      .catch(() => {
        // 이미지 요청 실패 시 기본 이미지 반환
        if (request.url.includes('/images/')) {
          return caches.match('/images/social/사용자더미.png');
        }
        return new Response('', { status: 404 });
      })
  );
});

// 푸시 알림 이벤트 (향후 확장용)
self.addEventListener('push', (event) => {
  if (event.data) {
    const data = event.data.json();
    const options = {
      body: data.body || '새로운 알림이 있습니다.',
      icon: '/images/logo.png',
      badge: '/icons/icon-72x72.png',
      vibrate: [100, 50, 100],
      data: {
        url: data.url || '/'
      }
    };

    event.waitUntil(
      self.registration.showNotification(data.title || 'NareuGO', options)
    );
  }
});

// 알림 클릭 이벤트
self.addEventListener('notificationclick', (event) => {
  event.notification.close();

  event.waitUntil(
    clients.openWindow(event.notification.data.url || '/')
  );
});

// 백그라운드 동기화 (향후 확장용)
self.addEventListener('sync', (event) => {
  if (event.tag === 'background-sync') {
    console.log('Service Worker: Background sync triggered');
    // 오프라인 중 저장된 데이터 동기화 로직
  }
});

console.log('Service Worker: Loaded successfully');
