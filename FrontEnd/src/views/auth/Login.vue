<template>
  <div class="login-page">
    <div class="login-container">
      <h1>로그인</h1>
      <p>소셜 계정으로 간편하게 로그인하세요</p>
      
      <SocialLoginButtons />
      
      <div class="divider">
        <span>또는</span>
      </div>
      
      <div class="manual-form">
        <p>테스트용 수동 로그인</p>
        <button @click="simulateLogin" class="manual-login-btn">
          테스트 로그인
        </button>
      </div>
    </div>
  </div>
</template>

<script setup>
import { useRouter } from 'vue-router'
import { useAuthStore } from '@/stores/auth'
import SocialLoginButtons from '@/components/SocialLoginButtons.vue'

const router = useRouter()
const authStore = useAuthStore()

const simulateLogin = () => {
  // 테스트용 로그인 시뮬레이션
  const testToken = 'test-token-' + Date.now()
  authStore.setTokens(testToken)
  authStore.setUser({
    id: '1',
    name: '테스트 사용자',
    email: 'test@example.com',
    provider: 'manual'
  })
  router.push('/')
}
</script>

<style scoped>
.login-page {
  display: flex;
  justify-content: center;
  align-items: center;
  min-height: 80vh;
}

.login-container {
  background: white;
  padding: 40px;
  border-radius: 16px;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
  max-width: 400px;
  width: 100%;
  text-align: center;
}

.login-container h1 {
  color: var(--main);
  margin-bottom: 8px;
  font-size: 28px;
  font-weight: 600;
}

.login-container > p {
  color: var(--deepgray);
  margin-bottom: 32px;
}

.divider {
  margin: 32px 0;
  position: relative;
}

.divider::before {
  content: '';
  position: absolute;
  top: 50%;
  left: 0;
  right: 0;
  height: 1px;
  background: #e2e8f0;
}

.divider span {
  background: white;
  padding: 0 16px;
  color: var(--deepgray);
  position: relative;
}

.manual-form {
  text-align: center;
}

.manual-form p {
  margin-bottom: 16px;
  color: var(--deepgray);
  font-size: 14px;
}

.manual-login-btn {
  background-color: var(--main);
  color: white;
  padding: 12px 24px;
  border-radius: 8px;
  font-size: 16px;
  font-weight: 600;
  transition: background-color 0.2s;
}

.manual-login-btn:hover {
  background-color: #7a1a1a;
}
</style>