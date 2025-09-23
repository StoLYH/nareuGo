import { createApp } from "vue";
import { createPinia } from "pinia";
import App from "./App.vue";
import router from "./router";
import { connectToROS2 } from "./utils/ros2Communication.js";

const app = createApp(App);
const pinia = createPinia();

app.use(pinia);
app.use(router);

app.mount("#app");

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

// PWA Service Worker 등록
if ('serviceWorker' in navigator) {
  window.addEventListener('load', () => {
    navigator.serviceWorker.register('/sw.js')
      .then((registration) => {
        console.log('SW registered: ', registration);
        
        // 업데이트 확인
        registration.addEventListener('updatefound', () => {
          const newWorker = registration.installing;
          newWorker.addEventListener('statechange', () => {
            if (newWorker.state === 'installed' && navigator.serviceWorker.controller) {
              // 새 버전 사용 가능 알림
              console.log('새 버전이 사용 가능합니다. 페이지를 새로고침해주세요.');
            }
          });
        });
      })
      .catch((registrationError) => {
        console.log('SW registration failed: ', registrationError);
      });
  });
}