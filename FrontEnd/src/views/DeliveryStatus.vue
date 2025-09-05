<template>
  <div class="delivery-status">
    <!-- 헤더 -->
    <DeliveryStatusHeader />
    
    <!-- 택배 현황 카드들 -->
    <DeliveryCards />
  </div>
</template>

<script setup>
import DeliveryStatusHeader from '../components/DeliveryStatusHeader.vue';
import DeliveryCards from '../components/DeliveryCards.vue';
</script>

<style scoped>
.delivery-status {
  min-height: 100vh;
  background-color: #f8f9fa;
}

/* 헤더를 제외한 나머지 요소들만 흰색 배경 적용 */
.delivery-cards {
  background-color: white;
}
</style>
