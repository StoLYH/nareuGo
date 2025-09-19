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
  background-color: #f8f9fa;
}

/* 전체 페이지 스타일링 */
.mypage > * {
  background-color: white;
}

/* 콘텐츠 영역은 남는 공간을 차지하여 하단 네비가 바닥에 고정되도록 함 */
.mypage-content {
  flex: 1;
  overflow-y: auto;
}
</style>