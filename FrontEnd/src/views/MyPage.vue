<template>
  <div class="mypage">
    <!-- 헤더 -->
    <MyPageHeader />
    
    <!-- 본문 컨텐츠: 스크롤 영역 -->
    <div class="mypage-content">
      <!-- 사용자 프로필 섹션 -->
      <UserProfile />
      
      <!-- 시작하기 버튼 -->
      <StartButton />
      
      <!-- 상단 3개 버튼 카드 -->
      <TopButtonsCard />
      
      <!-- 하단 4개 개별 버튼들 -->
      <BottomButtons />
    </div>

    <!-- 공통 하단 네비게이션 -->
    <BottomNavigation 
      active-tab="profile"
      @navigate="handleNavigation"
    />
  </div>
</template>

<script setup>
import { useRouter } from 'vue-router'
import MyPageHeader from '../components/MyPageHeader.vue';
import UserProfile from '../components/UserProfile.vue';
import StartButton from '../components/StartButton.vue';
import TopButtonsCard from '../components/TopButtonsCard.vue';
import BottomButtons from '../components/BottomButtons.vue';
import BottomNavigation from '../components/BottomNavigation.vue';

const router = useRouter()

const handleNavigation = (tab) => {
  switch(tab) {
    case 'home':
      router.push('/items')
      break
    case 'chat':
      router.push('/chat')
      break
    case 'profile':
      router.push('/profile')
      break
    default:
      console.log('Unknown navigation tab:', tab)
  }
}
</script>

<style scoped>
.mypage {
  min-height: 100vh;
  display: flex;
  flex-direction: column;
  /* Brand palette */
  --primary: #4682b4;
  --primary-600: #3a6b9a;
  --secondary: #2c3e50;
  --muted: #6b7280;
  --surface: #ffffff;
  --surface-2: #f3f6fa;

  background-color: var(--surface-2);
}

/* 콘텐츠 영역 내부 카드들만 흰 배경 적용 (헤더/바텀내브는 제외) */
.mypage-content > * {
  background-color: var(--surface);
}

/* 콘텐츠 영역은 남는 공간을 차지하여 하단 네비가 바닥에 고정되도록 함 */
.mypage-content {
  flex: 1;
  overflow-y: auto;
  /* 상단 고정 헤더 높이만큼 여백 확보 */
  padding-top: 80px;
  /* 좌우 여백 통일 */
  padding-left: 16px;
  padding-right: 16px;
  /* 하단 네비게이션 영역 확보 */
  padding-bottom: calc(96px + env(safe-area-inset-bottom));
  /* 섹션 간 간격 통일 */
  display: flex;
  flex-direction: column;
  gap: 12px;
  background-color: var(--surface-2);
}

/* 자식 섹션들의 외부 마진 제거 (페이지 레이아웃에서 간격 관리) */
.mypage-content > * {
  margin: 0;
}

/* 컴포넌트 내부의 중복 여백/마진을 정리하여 일관성 유지 */
:deep(.top-buttons-card) {
  margin: 0 !important;
}

:deep(.bottom-buttons) {
  padding: 0 !important;
}

/* 필요시 다른 내부 섹션도 동일하게 정리 가능 */
/* :deep(.some-section) { margin: 0 !important; } */
</style>