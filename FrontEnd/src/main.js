import { createApp } from "vue";
import { createPinia } from "pinia";
import App from "./App.vue";
import router from "./router";
import { notificationService } from "./services/notificationService";

const app = createApp(App);
const pinia = createPinia();

app.use(pinia);
app.use(router);

app.mount("#app");

// 페이지 로드 후 알림 서비스 초기화
window.addEventListener('load', async () => {
  // 로그인된 사용자 정보가 있으면 알림 서비스 초기화
  const userId = localStorage.getItem('userId');
  if (userId) {
    try {
      await notificationService.initialize(parseInt(userId));
      console.log('Notification service initialized');
    } catch (error) {
      console.error('Failed to initialize notification service:', error);
    }
  }
});
