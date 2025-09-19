<template>
  <div class="start-button-container">
    <button class="start-button" @click="openDeliveryModal">
      나르고 시작하기
    </button>

    <!-- 배송 시작 모달 -->
    <DeliveryStartModal
      :isVisible="showModal"
      @close="closeModal"
      @delivery-started="handleDeliveryStarted"
    />
  </div>
</template>

<script setup>
import { ref } from 'vue'
import DeliveryStartModal from './DeliveryStartModal.vue'

const showModal = ref(false)

const openDeliveryModal = () => {
  console.log('🔍 [DEBUG] 나르고 시작하기 버튼 클릭됨')
  showModal.value = true
  console.log('🔍 [DEBUG] 모달 상태 변경:', showModal.value)
}

const closeModal = () => {
  showModal.value = false
}

const handleDeliveryStarted = (product) => {
  console.log('배송이 시작되었습니다:', product)
  showModal.value = false
}
</script>

<style scoped>
.start-button-container {
  padding: 20px;
  margin-bottom: 32px;
}

.start-button {
  width: 100%;
  padding: 16px 24px;
  background: linear-gradient(135deg, #007AFF 0%, #0051D5 100%);
  color: white;
  border: none;
  border-radius: 12px;
  font-size: 16px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s ease;
  box-shadow: 0 2px 8px rgba(0, 122, 255, 0.3);
}

.start-button:hover {
  background: linear-gradient(135deg, #0056CC 0%, #003D99 100%);
  box-shadow: 0 4px 12px rgba(0, 122, 255, 0.4);
  transform: translateY(-1px);
}

.start-button:active {
  transform: translateY(0);
  box-shadow: 0 2px 8px rgba(0, 122, 255, 0.3);
}
</style>