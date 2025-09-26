import { createApp } from "vue";
import { createPinia } from "pinia";
import App from "./App.vue";
import router from "./router";
import { connectToROS2 } from "./utils/ros2Communication.js";
import { useNotificationStore } from "./stores/notification.js";
import "./utils/buyerNotification.js";
import { initFCMMessageHandler } from "./utils/fcmMessageHandler.js";
import fcmService from "./api/fcmService.js";

const app = createApp(App);
const pinia = createPinia();

app.use(pinia);
app.use(router);

app.mount("#app");

// 앱 마운트 후 전역 알림 이벤트 리스너 초기화
const notificationStore = useNotificationStore();
notificationStore.initEventListeners();

// FCM 서비스 및 메시지 핸들러 초기화
const initializeFCM = async () => {
  try {
    // 사용자 ID 가져오기 (로컬스토리지에서)
    const getUserId = () => {
      // 여러 키에서 사용자 정보 찾기
      const userInfo = localStorage.getItem('user_info') || localStorage.getItem('user')
      if (userInfo) {
        try {
          const parsedInfo = JSON.parse(userInfo)
          return parsedInfo.id || parsedInfo.userId || parsedInfo.user_id || 1
        } catch (error) {
          console.warn('🔔 [FCM INIT] 사용자 정보 파싱 실패, 기본값 사용:', error)
          return 1
        }
      }
      console.warn('🔔 [FCM INIT] 사용자 정보 없음, 기본값 사용')
      return 1 // 테스트용 기본값
    }

    const userId = getUserId()
    console.log('🔔 [FCM INIT] FCM 서비스 초기화 시작 - 사용자 ID:', userId)

    // FCM 서비스 초기화
    const success = await fcmService.initialize(userId)
    if (success) {
      console.log('✅ [FCM INIT] FCM 서비스 초기화 성공')
    } else {
      console.warn('⚠️ [FCM INIT] FCM 서비스 초기화 실패 (HTTP 환경이거나 권한 문제)')
    }

    // FCM 메시지 핸들러 초기화
    initFCMMessageHandler()
    console.log('✅ [FCM INIT] FCM 메시지 핸들러 초기화 완료')

  } catch (error) {
    console.error('❌ [FCM INIT] FCM 초기화 실패:', error)
  }
}

// FCM 서비스를 전역에 노출
window.fcmService = fcmService

// FCM 초기화 실행 (비동기) - 로그인 후에 실행되도록 지연
setTimeout(initializeFCM, 3000)

// ROS2 연결 초기화 (선택적)
const initializeROS2 = async () => {
  try {
    console.log('🤖 [INIT] ROS2 연결 시도 중...')
    await connectToROS2('ws://localhost:9090')
    console.log('✅ [INIT] ROS2 연결 성공')
  } catch (error) {
    console.warn('⚠️ [INIT] ROS2 연결 실패 (시뮬레이션 모드로 동작):', error.message)
  }
}

// 앱 로드 후 ROS2 연결 시도 (비활성화)
// setTimeout(initializeROS2, 1000)

// PWA Service Worker
// 개발 모드에서는 SW가 HMR/리소스 캐시를 방해할 수 있으므로 등록하지 않고, 기존 SW/캐시를 정리합니다.
if ('serviceWorker' in navigator) {
  if (import.meta.env.PROD) {
    // 프로덕션에서만 등록
    window.addEventListener('load', () => {
      navigator.serviceWorker.register('/sw.js')
        .then((registration) => {
          console.log('SW registered: ', registration);
          // 업데이트 확인
          registration.addEventListener('updatefound', () => {
            const newWorker = registration.installing;
            if (!newWorker) return;
            newWorker.addEventListener('statechange', () => {
              if (newWorker.state === 'installed' && navigator.serviceWorker.controller) {
                console.log('새 버전이 사용 가능합니다. 페이지를 새로고침해주세요.');
              }
            });
          });
        })
        .catch((registrationError) => {
          console.log('SW registration failed: ', registrationError);
        });
    });
  } else {
    // 개발 모드: 기존 서비스워커 정리 + 캐시 삭제로 HMR 방해 제거
    navigator.serviceWorker.getRegistrations?.().then((regs) => {
      if (regs?.length) {
        console.log(`[DEV] 기존 서비스워커 ${regs.length}개 해제 시도`);
      }
      regs.forEach((reg) => reg.unregister());
    }).catch(() => {});

    // 프로젝트에서 사용한 캐시 네임프리픽스에 맞춰 정리
    if (window.caches?.keys) {
      caches.keys().then((keys) => {
        const targets = keys.filter((k) => k.startsWith('nareugo-'));
        if (targets.length) {
          console.log(`[DEV] 캐시 제거: ${targets.join(', ')}`);
        }
        return Promise.all(targets.map((k) => caches.delete(k)));
      }).catch(() => {});
    }
  }
}