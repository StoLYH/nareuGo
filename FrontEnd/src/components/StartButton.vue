<template>
  <div class="start-button-container">
    <button 
      class="start-button" 
      @click="openDeliveryModal" 
      :disabled="checkingRobotStatus"
    >
      {{ checkingRobotStatus ? '로봇 상태 확인 중...' : '나르고 시작하기' }}
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
import { getRobotStatus, getCurrentDeliveryId } from '../api/delivery/delivery.js'
import { useAuthStore } from '@/stores/auth'

const showModal = ref(false)
const checkingRobotStatus = ref(false)
const authStore = useAuthStore()

const openDeliveryModal = async () => {
  console.log('🔍 [DEBUG] 나르고 시작하기 버튼 클릭됨')
  
  try {
    checkingRobotStatus.value = true
    
    // DB에서 현재 진행 중인 배송 ID 조회
    const userId = authStore.user?.userId || 3
    console.log('📋 [DEBUG] 현재 배송 ID 조회 중... userId:', userId)
    const currentDeliveryId = await getCurrentDeliveryId(userId)
    console.log('📋 [DEBUG] 조회된 배송 ID:', currentDeliveryId)

    // 로봇 상태 확인 (delivery_id 파라미터 포함)
    console.log('🤖 [DEBUG] 로봇 상태 확인 중...')
    const robotStatus = await getRobotStatus(1, currentDeliveryId)
    console.log('🤖 [DEBUG] 로봇 상태 응답:', robotStatus)
    
    if (robotStatus.status === 'INVALID' || robotStatus.status === 'invalid') {
      alert('나르고가 다른 일을 처리 중입니다. 잠시 후 다시 시도해주세요.')
      return
    }

    if (robotStatus.status === 'VALID' || robotStatus.status === 'valid') {
      // 로봇이 사용 가능한 상태일 때만 모달 열기
      console.log('✅ [DEBUG] 로봇 사용 가능, 모달 열기')
      showModal.value = true
      console.log('🔍 [DEBUG] 모달 상태 변경:', showModal.value)
    } else {
      alert('로봇 상태를 확인할 수 없습니다. 다시 시도해주세요.')
    }
  } catch (error) {
    console.error('❌ [ERROR] 로봇 상태 확인 실패:', error)
    alert('로봇 상태 확인에 실패했습니다. 네트워크 연결을 확인해주세요.')
  } finally {
    checkingRobotStatus.value = false
  }
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
  background: linear-gradient(90deg, #4682B4, #6EC6CA);
  /* background: linear-gradient(90deg, #5A9BD6, #7FD7DA); */
  color: white;
  border: none;
  border-radius: 12px;
  font-size: 16px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s ease;
  box-shadow: 0 2px 8px rgba(70, 130, 180, 0.25);
}

.start-button:hover {
  background: linear-gradient(90deg, #5A9BD6, #7FD7DA);
  box-shadow: 0 4px 12px rgba(70, 130, 180, 0.35);
  transform: translateY(-1px);
}

.start-button:active {
  transform: translateY(0);
  box-shadow: 0 2px 8px rgba(70, 130, 180, 0.25);
}

.start-button:disabled {
  background: linear-gradient(90deg, #b7c7d6, #c5d7da);
  cursor: not-allowed;
  transform: none;
  box-shadow: 0 2px 8px rgba(70, 130, 180, 0.15);
}
</style>