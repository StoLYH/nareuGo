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
        <input type="text" placeholder="비밀번호" class="login-input" />
      </div>
      <div class="input-group">
        <input type="password" placeholder="비밀번호" class="login-input" />
      </div>
      
      <button class="login-btn">로그인</button>
      <button class="signup-btn">회원가입</button>
      
      <!-- 소셜 로그인 -->
      <div class="social-login">
        <button class="social-btn kakao">
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
            <path d="M12 2C6.477 2 2 6.477 2 12c0 3.314 1.602 6.24 4.06 8.06L4 22l2.06-2.06C8.76 21.398 10.398 22 12 22c5.523 0 10-4.477 10-10S17.523 2 12 2z" fill="black"/>
          </svg>
        </button>
        <button class="social-btn google">
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
            <path d="M22.56 12.25c0-.78-.07-1.53-.2-2.25H12v4.26h5.92c-.26 1.37-1.04 2.53-2.21 3.31v2.77h3.57c2.08-1.92 3.28-4.74 3.28-8.09z" fill="#4285F4"/>
            <path d="M12 23c2.97 0 5.46-.98 7.28-2.66l-3.57-2.77c-.98.66-2.23 1.06-3.71 1.06-2.86 0-5.29-1.93-6.16-4.53H2.18v2.84C3.99 20.53 7.7 23 12 23z" fill="#34A853"/>
            <path d="M5.84 14.09c-.22-.66-.35-1.36-.35-2.09s.13-1.43.35-2.09V7.07H2.18C1.43 8.55 1 10.22 1 12s.43 3.45 1.18 4.93l2.85-2.22.81-.62z" fill="#FBBC05"/>
            <path d="M12 5.38c1.62 0 3.06.56 4.21 1.64l3.15-3.15C17.45 2.09 14.97 1 12 1 7.7 1 3.99 3.47 2.18 7.07l3.66 2.84c.87-2.6 3.3-4.53 6.16-4.53z" fill="#EA4335"/>
          </svg>
        </button>
        <button class="social-btn naver">
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
            <path d="M16.273 12.845 7.376 0H0v24h7.726V11.156L16.624 24H24V0h-7.727v12.845Z" fill="white"/>
          </svg>
        </button>
      </div>
    </div>
  </div>
</template>

<script setup>
// 로그인 관련 로직은 나중에 구현
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
  transition: background-color 0.2s;
}

.login-btn:hover {
  background-color: #5A83B7;
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

.social-btn svg {
  width: 24px;
  height: 24px;
}

.social-btn.kakao {
  background-color: #FEE500; /* 카카오 노란색 */
}

.social-btn.google {
  background-color: white; /* 구글 흰색 */
}

.social-btn.naver {
  background-color: #03C75A; /* 네이버 초록색 */
}
</style>
