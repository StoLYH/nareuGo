<template>
  <div class="delivery-status">
    <!-- 헤더 -->
    <DeliveryStatusHeader />
    
    <!-- 택배 현황 카드들 -->
    <DeliveryCards />
    
    <!-- 하단 네비게이션 -->
    <BottomNavigation
      active-tab=""
      @navigate="handleNavigation"
    />
  </div>
</template>

<script setup>
import DeliveryStatusHeader from '../components/DeliveryStatusHeader.vue';
import DeliveryCards from '../components/DeliveryCards.vue';
import BottomNavigation from '../components/BottomNavigation.vue';
import { useRouter } from 'vue-router';

const router = useRouter();
const handleNavigation = (tab) => {
  switch(tab) {
    case 'home':
      router.push('/items');
      break;
    case 'chat':
      router.push('/chat');
      break;
    case 'profile':
      router.push('/profile');
      break;
    default:
      console.log('Unknown navigation tab:', tab);
  }
}
</script>

<style scoped>
.delivery-status {
  min-height: 100vh;
  background-color: #f8f9fa;
  /* 고정 헤더 높이만큼 상단 여백 확보 */
  padding-top: 72px;
  display: flex;
  flex-direction: column;
  /* 하단 네비게이션 가려짐 방지 */
  padding-bottom: calc(88px + env(safe-area-inset-bottom));
}

.delivery-cards {
  background-color: white;
  flex: 1; /* 헤더 아래 남은 높이를 자연스럽게 채우기 */
}
</style>
