<template>
  <div class="login-page">
    <!-- 상단 파란색 섹션 -->
    <div class="top-section">
      <div class="logo-container">
        <img src="/images/logo.png" alt="NareuGO 로고" class="robot-logo" />
        <div class="logo-text-group">
          <h1 class="app-title">NareuGO</h1>
          <p class="app-tagline">Pasti Aman, Cepat, Mudah</p>
        </div>
      </div>
    </div>

    <!-- 하단 흰색 카드 섹션 -->
    <div class="login-form-card">
      <div class="input-group">
        <input 
          type="email" 
          placeholder="이메일" 
          class="login-input" 
          v-model="loginForm.email"
          @keyup.enter="handleLogin"
        />
      </div>
      <div class="input-group">
        <input 
          type="password" 
          placeholder="비밀번호" 
          class="login-input" 
          v-model="loginForm.password"
          @keyup.enter="handleLogin"
        />
      </div>
      
      <div v-if="errorMessage" class="error-message">
        {{ errorMessage }}
      </div>
      
      <button class="login-btn" @click="handleLogin" :disabled="isLoading">
        {{ isLoading ? '로그인 중...' : '로그인' }}
      </button>
      <button class="signup-btn">회원가입</button>
      
      <!-- 소셜 로그인 -->
      <div class="social-login">
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
import { ref } from 'vue'
import { useRouter } from 'vue-router'
import { useAuthStore } from '@/stores/auth'
import axios from 'axios'

const router = useRouter()
const authStore = useAuthStore()

// 로그인 폼 데이터
const loginForm = ref({
  email: '',
  password: ''
})

// 상태 관리
const isLoading = ref(false)
const errorMessage = ref('')

// API 기본 URL
const getApiBaseUrl = () => {
  return import.meta.env.VITE_BASE_URL || 'http://localhost:8080'
}

// 기본 로그인 함수
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
      // 로그인 성공
      console.log('로그인 성공:', response.data)
      
      // authStore에 사용자 정보 저장 (토큰 없이)
      authStore.setUser({
        userId: response.data.userId,
        email: response.data.email,
        name: response.data.name
      })
      
      // 더미 토큰 설정 (라우터 가드 통과용)
      authStore.setTokens('basic-login-token')
      
      // localStorage에 사용자 정보도 별도 저장
      localStorage.setItem('user', JSON.stringify({
        userId: response.data.userId,
        email: response.data.email,
        name: response.data.name
      }))
      
      console.log('localStorage 저장 확인:', localStorage.getItem('user'))
      console.log('authStore 토큰 확인:', authStore.accessToken)
      
      // ItemList 페이지로 이동
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
  background-color: #B0E0E6; /* 파우더 블루 */
  display: flex;
  flex-direction: column;
  font-family: "Pretendard-Regular", sans-serif;
}

/* 상단 파란색 섹션 */
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
  color: white;
  margin: 0;
  letter-spacing: -0.02em;
}

.app-tagline {
  font-size: 14px;
  font-weight: 400;
  color: white;
  margin: 4px 0 0 0;
  letter-spacing: -0.01em;
}

/* 하단 흰색 카드 섹션 */
.login-form-card {
  background-color: white;
  border-top-left-radius: 20px;
  border-top-right-radius: 20px;
  padding: 40px 20px;
  box-shadow: 0 -4px 10px rgba(0, 0, 0, 0.05);
  display: flex;
  flex-direction: column;
  align-items: center;
}

.input-group {
  width: 100%;
  margin-bottom: 16px;
}

.login-input {
  width: 100%;
  padding: 14px 16px;
  border: 1px solid #e0e0e0;
  border-radius: 12px;
  font-size: 15px;
  color: #333;
  outline: none;
  box-sizing: border-box;
}

.login-input::placeholder {
  color: #ccc;
}

.login-btn {
  width: 100%;
  padding: 16px;
  background-color: #6A93C7; /* 상단과 동일한 뮤트 블루 */
  color: white;
  border: none;
  border-radius: 12px;
  font-size: 16px;
  font-weight: 600;
  cursor: pointer;
  margin-bottom: 12px;
  transition: background-color 0.3s ease;
}

.login-btn:hover {
  background-color: #5A83B7;
}

.login-btn:disabled {
  background-color: #ccc;
  cursor: not-allowed;
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
</style>
