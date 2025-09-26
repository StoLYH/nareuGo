<template>
  <div class="login-page">
    <div class="top-section">
      <div class="logo-container">
        <img src="/images/logo.png" alt="NareuGO 로고" class="robot-logo splash-logo" />
        <div class="logo-text-group">
          <h1 class="app-title splash-text">NareuGO</h1>
          <p class="app-tagline splash-text">From Your Neighbor, To Your Door</p>
        </div>
      </div>
    </div>

    <div class="login-form-card">
      <div class="input-group">
        <input 
          type="email" 
          placeholder="이메일" 
          class="login-input animate-fade-in delay-1" 
          v-model="loginForm.email"
          @keyup.enter="handleLogin"
        />
      </div>
      <div class="input-group">
        <input 
          type="password" 
          placeholder="비밀번호" 
          class="login-input animate-fade-in delay-2" 
          v-model="loginForm.password"
          @keyup.enter="handleLogin"
        />
      </div>
      
      <div v-if="errorMessage" class="error-message animate-fade-in delay-3">
        {{ errorMessage }}
      </div>
      
      <button class="login-btn animate-fade-in delay-4" @click="handleLogin" :disabled="isLoading">
        {{ isLoading ? '로그인 중...' : '로그인' }}
      </button>
      <button class="signup-btn animate-fade-in delay-5">회원가입</button>
      
      <div class="social-login animate-fade-in delay-6">
        <button class="social-btn kakao" @click="loginWithKakao">
          <img src="/images/social/kakao-login.png" alt="카카오 로그인" class="social-icon" />
        </button>
        <button class="social-btn google" @click="loginWithGoogle">
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
            <path d="M22.56 12.25c0-.78-.07-1.53-.2-2.25H12v4.26h5.92c-.26 1.37-1.04 2.53-2.21 3.31v2.77h3.57c2.08-1.92 3.28-4.74 3.28-8.09z" fill="#4285F4"/>
            <path d="M12 23c2.97 0 5.46-.98 7.28-2.66l-3.57-2.77c-.98.66-2.23 1.06-3.71 1.06-2.86 0-5.29-1.93-6.16-4.53H2.18v2.84C3.99 20.53 7.7 23 12 23z" fill="#34A853"/>
            <path d="M5.84 14.09c-.22-.66-.35-1.36-.35-2.09s.13-1.43.35-2.09V7.07H2.18C1.43 8.55 1 10.22 1 12s.43 3.45 1.18 4.93l2.85-2.22.81-.62z" fill="#FBBC05"/>
            <path d="M12 5.38c1.62 0 3.06.56 4.21 1.64l3.15-3.15C17.45 2.09 14.97 1 12 1 7.7 1 3.99 3.47 2.18 7.07l3.66 2.84c.87-2.6 3.3-4.53 6.16-4.53z" fill="#EA4335"/>
          </svg>
        </button>
        <button class="social-btn naver" @click="loginWithNaver">
          <img src="/images/social/naver-login.png" alt="네이버 로그인" class="social-icon" />
        </button>
      </div>
    </div>
  </div>
</template>

<script setup>
// 기존 스크립트 코드 유지
import { ref } from 'vue'
import { useRouter } from 'vue-router'
import { useAuthStore } from '@/stores/auth'
import axios from 'axios'

const router = useRouter()
const authStore = useAuthStore()

const loginForm = ref({
  email: '',
  password: ''
})

const isLoading = ref(false)
const errorMessage = ref('')

const getApiBaseUrl = () => {
  return import.meta.env.VITE_BASE_URL || 'http://localhost:8080'
}

const handleLogin = async () => {
  if (!loginForm.value.email) {
    errorMessage.value = '이메일을 입력해주세요.'
    return
  }
  if (!loginForm.value.password) {
    errorMessage.value = '비밀번호를 입력해주세요.'
    return
  }

  isLoading.value = true
  errorMessage.value = ''

  try {
    const response = await axios.post(`${getApiBaseUrl()}/general-login`, {
      email: loginForm.value.email
    })

    if (response.data.success) {
      console.log('로그인 성공:', response.data)
      
      authStore.setUser({
        userId: response.data.userId,
        email: response.data.email,
        name: response.data.name,
        nickname: response.data.nickname
      })
      
      authStore.setTokens('basic-login-token')
      
      localStorage.setItem('user', JSON.stringify({
        userId: response.data.userId,
        email: response.data.email,
        name: response.data.name,
        nickname: response.data.nickname
      }))
      
      console.log('localStorage 저장 확인:', localStorage.getItem('user'))
      console.log('authStore 토큰 확인:', authStore.accessToken)
      
      router.push('/items')
    } else {
      errorMessage.value = response.data.message || '로그인에 실패했습니다.'
    }
  } catch (error) {
    console.error('로그인 오류:', error)
    if (error.response?.data?.message) {
      errorMessage.value = error.response.data.message
    } else {
      errorMessage.value = '서버 연결에 실패했습니다.'
    }
  } finally {
    isLoading.value = false
  }
}
</script>

<style scoped>
.login-page {
  width: 100%;
  height: 100vh;
  background: linear-gradient(180deg, #4682B4 40%, #F5F9FC 100%);
  display: flex;
  flex-direction: column;
  font-family: "Pretendard", sans-serif;
  overflow: hidden;
}

/* ================== 상단 로고 영역 ================== */
.top-section {
  flex: 1;
  display: flex;
  justify-content: center;
  align-items: center;
  padding-bottom: 40px;
}

.logo-container {
  display: flex;
  flex-direction: row;
  align-items: center;
  gap: 16px;
}

.robot-logo {
  width: 120px;
  height: 150px;
  object-fit: cover;
  filter: drop-shadow(0 4px 8px rgba(0, 0, 0, 0.1));
}

.logo-text-group {
  display: flex;
  flex-direction: column;
  align-items: flex-start;
}

.app-title {
  font-size: 32px;
  font-weight: 700;
  color: #ffffff;
  margin: 0;
  letter-spacing: -0.02em;
}

.app-tagline {
  font-size: 14px;
  font-weight: 400;
  color: #E3F2FD;
  margin: 4px 0 0 0;
  letter-spacing: -0.01em;
}

/* Splash 애니메이션 */
.splash-logo {
  opacity: 0;
  transform: translateY(-20px);
  animation: logo-fade-in 1s ease-out forwards;
}

.splash-text {
  opacity: 0;
  transform: translateY(20px);
  animation: text-fade-in 1s ease-out 0.2s forwards;
}

@keyframes logo-fade-in {
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

@keyframes text-fade-in {
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

/* ================== 로그인 카드 ================== */
.login-form-card {
  background-color: #ffffff;
  border-top-left-radius: 24px;
  border-top-right-radius: 24px;
  padding: 40px 24px;
  box-shadow: 0 -6px 16px rgba(0, 0, 0, 0.08);
  display: flex;
  flex-direction: column;
  align-items: center;
  transform: translateY(100%);
  animation: slide-up 0.8s ease-out 0.5s forwards;
}

@keyframes slide-up {
  to {
    transform: translateY(0);
  }
}

.input-group {
  width: 100%;
  margin-bottom: 16px;
}

/* ================== 입력창 ================== */
.login-input {
  width: 100%;
  padding: 14px 16px;
  border: 1px solid #dfe6ee;
  border-radius: 12px;
  font-size: 15px;
  color: #2C3E50;
  background-color: #F9FBFD;
  transition: border-color 0.2s ease, box-shadow 0.2s ease;
  opacity: 0;
}

.login-input:focus {
  border-color: #4682B4;
  box-shadow: 0 0 0 3px rgba(70, 130, 180, 0.2);
  outline: none;
}

.login-input::placeholder {
  color: #a0acb9;
}

/* ================== 버튼 ================== */
.login-btn {
  width: 100%;
  padding: 16px;
  background: linear-gradient(90deg, #4682B4, #6EC6CA);
  color: #fff;
  border: none;
  border-radius: 12px;
  font-size: 16px;
  font-weight: 600;
  cursor: pointer;
  margin-bottom: 12px;
  transition: transform 0.2s ease, box-shadow 0.2s ease, background 0.3s ease;
  opacity: 0;
}

.login-btn:hover {
  background: linear-gradient(90deg, #5A9BD6, #7FD7DA);
  box-shadow: 0 4px 12px rgba(70, 130, 180, 0.25);
  transform: translateY(-2px);
}

.login-btn:disabled {
  background: #b0bec5;
  cursor: not-allowed;
}

.signup-btn {
  width: 100%;
  padding: 16px;
  background-color: #ffffff;
  color: #4682B4;
  border: 1px solid #dfe6ee;
  border-radius: 12px;
  font-size: 16px;
  font-weight: 600;
  cursor: pointer;
  margin-bottom: 30px;
  transition: all 0.3s ease;
  opacity: 0;
}

.signup-btn:hover {
  background-color: #FFB347;
  border-color: #FFB347;
  color: #fff;
}

/* ================== 에러 메시지 ================== */
.error-message {
  color: #e74c3c;
  font-size: 14px;
  margin-bottom: 12px;
  text-align: center;
  padding: 10px;
  background-color: #fdecea;
  border: 1px solid #f5c6cb;
  border-radius: 10px;
  opacity: 0;
}

.error-message {
  color: #e74c3c;
  font-size: 14px;
  margin-bottom: 12px;
  text-align: center;
  padding: 8px;
  background-color: #fdf2f2;
  border: 1px solid #fecaca;
  border-radius: 8px;
  /* Initial state for fade-in animation */
  opacity: 0;
}

.signup-btn {
  width: 100%;
  padding: 16px;
  background-color: white;
  color: #666;
  border: 1px solid #e0e0e0;
  border-radius: 12px;
  font-size: 16px;
  font-weight: 600;
  cursor: pointer;
  margin-bottom: 30px;
  transition: all 0.2s;
  /* Initial state for fade-in animation */
  opacity: 0;
}

.signup-btn:hover {
  background-color: #f8f8f8;
}

/* 소셜 로그인 */
.social-login {
  display: flex;
  justify-content: center;
  gap: 16px;
  width: 100%;
  /* Initial state for fade-in animation */
  opacity: 0;
}

.social-btn {
  width: 48px;
  height: 48px;
  border-radius: 50%;
  border: none;
  display: flex;
  justify-content: center;
  align-items: center;
  cursor: pointer;
  box-shadow: 0 2px 5px rgba(0, 0, 0, 0.1);
  flex-shrink: 0;
}

.social-btn .social-icon {
  width: 24px;
  height: 24px;
  object-fit: contain;
}

.social-btn.naver .social-icon {
  width: 48px;
  height: 48px;
}

.social-btn.kakao .social-icon {
  width: 48px;
  height: 48px;
}

.social-btn.kakao {
  background-color: transparent;
  box-shadow: none;
}

.social-btn.google {
  background-color: white;
}

.social-btn.naver {
  background-color: transparent;
  box-shadow: none;
}

/* Staggered fade-in animation classes */
.animate-fade-in {
  animation: fade-in 0.5s ease-out forwards;
}

.delay-1 {
  animation-delay: 1.3s;
}

.delay-2 {
  animation-delay: 1.4s;
}

.delay-3 {
  animation-delay: 1.5s;
}

.delay-4 {
  animation-delay: 1.6s;
}

.delay-5 {
  animation-delay: 1.7s;
}

.delay-6 {
  animation-delay: 1.8s;
}

/* Fade-in keyframes */
@keyframes fade-in {
  from {
    opacity: 0;
    transform: translateY(10px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}
</style>