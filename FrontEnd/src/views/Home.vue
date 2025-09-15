<template>
  <div class="home-page">
    <!-- 상단 헤더 섹션 -->
    <header class="header-section">
      <div class="header-container">
        <div class="logo-section">
          <img src="/images/logo.png" alt="NareuGO 로고" class="logo-image" />
          <div class="brand-info">
            <h1 class="brand-title">NareuGO</h1>
            <p class="brand-tagline">Pasti Aman, Cepat, Mudah</p>
          </div>
        </div>
        <div class="user-section" v-if="authStore.user">
          <div class="user-greeting">
            <span class="greeting-text">안녕하세요, {{ authStore.user.name }}님</span>
            <button class="logout-btn" @click="handleLogout">로그아웃</button>
          </div>
        </div>
      </div>
    </header>

    <!-- 메인 컨텐츠 영역 -->
    <main class="main-content">
      메인페이지
    </main>
  </div>
</template>

<script setup>
import { onMounted } from 'vue'
import { useAuthStore } from '@/stores/auth'
import { useRouter, useRoute } from 'vue-router'
import { authAPI } from '@/api/auth'

const authStore = useAuthStore()
const router = useRouter()
const route = useRoute()

onMounted(async () => {
  // URL에서 토큰 확인 (OAuth2 로그인 후 리다이렉트)
  const token = route.query.token
  
  if (token) {
    try {
      // 토큰 저장
      authStore.setTokens(token)
      // 사용자 정보 조회 (토큰을 명시적으로 헤더에 실어 호출)
      const { data } = await authAPI.getUserByToken(token)
      authStore.setUser(data)
      // URL에서 토큰 파라미터 제거
      router.replace('/home')
    } catch (error) {
      router.push('/login')
    }
  }
})

const handleLogout = async () => {
  try {
    await authStore.logout()
    await router.push('/login')
  } catch (error) {
    // 에러가 있어도 강제로 로그인 페이지로 이동
    authStore.clearAuth()
    await router.push('/login')
  }
}
</script>

<style scoped>
.home-page {
  width: 100%;
  min-height: 100vh;
  background-color: #B0E0E6; /* 파우더 블루 */
  font-family: "Pretendard-Regular", sans-serif;
}

/* 헤더 섹션 */
.header-section {
  background-color: #B0E0E6;
  padding: 20px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

.header-container {
  max-width: 1200px;
  margin: 0 auto;
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.logo-section {
  display: flex;
  align-items: center;
  gap: 16px;
}

.logo-image {
  width: 60px;
  height: 75px;
  object-fit: cover;
  filter: drop-shadow(0 2px 4px rgba(0, 0, 0, 0.1));
}

.brand-info {
  display: flex;
  flex-direction: column;
  align-items: flex-start;
}

.brand-title {
  font-size: 28px;
  font-weight: 700;
  color: white;
  margin: 0;
  letter-spacing: -0.02em;
}

.brand-tagline {
  font-size: 12px;
  font-weight: 400;
  color: white;
  margin: 2px 0 0 0;
  opacity: 0.9;
}

.user-section {
  display: flex;
  align-items: center;
}

.user-greeting {
  display: flex;
  align-items: center;
  gap: 12px;
}

.greeting-text {
  color: white;
  font-size: 14px;
  font-weight: 500;
}

.logout-btn {
  background-color: white;
  color: #6A93C7;
  border: none;
  padding: 8px 16px;
  border-radius: 20px;
  font-size: 12px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s;
}

.logout-btn:hover {
  background-color: #f8f8f8;
  transform: translateY(-1px);
}

/* 메인 컨텐츠 */
.main-content {
  max-width: 1200px;
  margin: 0 auto;
  padding: 40px 20px;
}

.hero-section {
  text-align: center;
  margin-bottom: 50px;
}

.hero-title {
  font-size: 32px;
  font-weight: 700;
  color: white;
  margin-bottom: 16px;
  letter-spacing: -0.02em;
}

.hero-description {
  font-size: 16px;
  color: white;
  opacity: 0.9;
  line-height: 1.5;
}

/* 기능 카드 그리드 */
.features-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 24px;
  margin-bottom: 50px;
}

.feature-card {
  background-color: white;
  border-radius: 16px;
  padding: 30px 24px;
  text-align: center;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
  transition: all 0.3s ease;
  cursor: pointer;
}

.feature-card:hover {
  transform: translateY(-4px);
  box-shadow: 0 8px 24px rgba(0, 0, 0, 0.15);
}

.feature-icon {
  font-size: 48px;
  margin-bottom: 16px;
}

.feature-title {
  font-size: 20px;
  font-weight: 700;
  color: #333;
  margin-bottom: 12px;
}

.feature-description {
  font-size: 14px;
  color: #666;
  line-height: 1.5;
  margin: 0;
}

/* CTA 섹션 */
.cta-section {
  text-align: center;
}

.cta-button {
  background-color: #6A93C7;
  color: white;
  border: none;
  padding: 16px 32px;
  border-radius: 25px;
  font-size: 18px;
  font-weight: 700;
  cursor: pointer;
  transition: all 0.3s ease;
  box-shadow: 0 4px 12px rgba(106, 147, 199, 0.3);
}

.cta-button:hover {
  background-color: #5A83B7;
  transform: translateY(-2px);
  box-shadow: 0 6px 16px rgba(106, 147, 199, 0.4);
}

/* 반응형 디자인 */
@media (max-width: 768px) {
  .header-container {
    flex-direction: column;
    gap: 16px;
  }
  
  .hero-title {
    font-size: 24px;
  }
  
  .features-grid {
    grid-template-columns: 1fr;
    gap: 16px;
  }
  
  .main-content {
    padding: 30px 16px;
  }
}
</style>