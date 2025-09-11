import { createApp } from "vue";
import { createPinia } from "pinia";
import App from "./App.vue";
import router from "./router";
import { useAuthStore } from "./stores/auth.js";

const app = createApp(App);
const pinia = createPinia();

app.use(pinia);
app.use(router);

// 앱 시작 시 저장된 인증 상태 복원
const authStore = useAuthStore();
authStore.restoreAuth();

app.mount("#app");
