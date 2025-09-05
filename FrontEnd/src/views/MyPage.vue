<template>
  <div class="mypage">
    <!-- 헤더 -->
    <MyPageHeader />
    
    <!-- 사용자 프로필 섹션 -->
    <UserProfile />
    
    <!-- 시작하기 버튼 -->
    <StartButton />
    
    <!-- 상단 3개 버튼 카드 -->
    <TopButtonsCard />
    
    <!-- 하단 4개 개별 버튼들 -->
    <BottomButtons />
  </div>
</template>

<script setup>
import MyPageHeader from '../components/MyPageHeader.vue';
import UserProfile from '../components/UserProfile.vue';
import StartButton from '../components/StartButton.vue';
import TopButtonsCard from '../components/TopButtonsCard.vue';
import BottomButtons from '../components/BottomButtons.vue';
</script>

<style scoped>
.mypage {
  min-height: 100vh;
  background-color: #f8f9fa;
}

/* 전체 페이지 스타일링 */
.mypage > * {
  background-color: white;
}
</style>