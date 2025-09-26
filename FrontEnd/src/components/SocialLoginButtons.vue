<!-- components/auth/SocialLoginButtons.vue -->
  <template>                                                                                                                                                                        
    <div class="social-login-container">                                                                                                                                            
      <h2>소셜 로그인</h2>
      
      <div class="social-buttons-row">
        <!-- 카카오 로그인 버튼 -->
        <button
          @click="loginWithKakao"
          class="social-login-btn kakao-btn">
            <img src="/images/social/kakao-login.png" alt="카카오 로그인" class="social-logo" />
        </button>

        <!-- 구글 로그인 버튼 -->
        <button
          @click="loginWithGoogle"
          class="social-login-btn google-btn"
        >
          <div class="icon-circle">
            <div class="google-icon">G</div>
          </div>
        </button>

        <!-- 네이버 로그인 버튼 -->
        <button
            @click="loginWithNaver"
            class="social-login-btn naver-btn"
            >
            <img src="/images/social/naver-login.png" alt="네이버 로그인" />
            </button>
      </div>
    </div>
  </template>

  <script setup>                                                                                                                                                                    
  import { useAuthStore } from '@/stores/auth'

  const authStore = useAuthStore()

  const getApiBaseUrl = () => {
    return import.meta.env.VITE_API_BASE_URL || 'http://localhost:8080'
  }

  const loginWithKakao = () => {
    const kakaoAuthUrl = `${getApiBaseUrl()}/oauth2/authorization/kakao`
    window.location.href = kakaoAuthUrl
  }

  const loginWithGoogle = () => {
    const googleAuthUrl = `${getApiBaseUrl()}/oauth2/authorization/google`
    window.location.href = googleAuthUrl
  }

  const loginWithNaver = () => {
    const naverAuthUrl = `${getApiBaseUrl()}/oauth2/authorization/naver`
    window.location.href = naverAuthUrl
  }
  </script>                                                                                                                                                                         

 <style scoped>
.social-login-container {
  display: flex;
  flex-direction: column;
  gap: 20px;
  max-width: 400px;
  margin: 0 auto;
  align-items: center;
}

.social-buttons-row {
  display: flex;
  gap: 20px;
  justify-content: center;
  align-items: center;
}

.social-login-btn {
  background: none;
  border: none;
  padding: 0;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: transform 0.2s ease;
  width: 60px; /* 버튼의 크기를 원의 크기와 동일하게 설정 */
  height: 60px;
}

.social-login-btn:hover {
  transform: scale(1.05);
}

/* Google 버튼에 필요한 기존 icon-circle 스타일 유지 */
.icon-circle {
  width: 60px;
  height: 60px;
  border-radius: 50%;
  background: white;
  display: flex;
  align-items: center;
  justify-content: center;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.15);
  border: 3px solid #ff4444;
  position: relative;
}

/* 카카오와 네이버 로고 이미지 크기 조정 */
.social-login-btn img {
  width: 60px; /* 버튼 크기에 맞춰 로고 크기 설정 */
  height: 60px;
  object-fit: contain; /* 이미지 비율 유지 */
}

.google-icon {
  font-size: 24px;
  font-weight: bold;
  color: #4285f4;
  font-family: 'Roboto', sans-serif;
}
</style>