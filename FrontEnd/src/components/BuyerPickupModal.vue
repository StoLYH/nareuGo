<template>
  <div v-if="isVisible" class="modal-overlay" @click="closeModal">
    <div class="modal-container" @click.stop>
      <div class="modal-header">
        <div class="header-content">
          <div class="logo-container">
            <img src="/images/logo.png" alt="나르고 로고" class="nareugo-logo" />
            <span class="brand-name">나르고</span>
          </div>
          <h2>배송 완료 알림</h2>
        </div>
        <button class="close-button" @click="closeModal">
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
            <path d="M18 6L6 18M6 6l12 12" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
          </svg>
        </button>
      </div>

      <div class="modal-body">
        <div v-if="loading" class="loading">
          처리 중...
        </div>

        <div v-else class="pickup-info">
          <div class="arrival-status">
            <div class="status-icon-container">
              <div class="pulse-ring"></div>
              <img src="/images/logo.png" alt="나르고" class="robot-logo" />
            </div>
            <h3 class="status-title">배송이 완료되었습니다!</h3>
            <p class="status-subtitle">나르고에서 상품을 꺼내주세요</p>
          </div>

          <div class="delivery-card" v-if="deliveryData">
            <div class="card-header">
              <div class="card-icon"></div>
              <h4>주문 정보</h4>
            </div>
            <div class="info-grid">
              <div class="info-row">
                <span class="info-label">상품명</span>
                <span class="info-value">{{ deliveryData.productTitle || '상품명 없음' }}</span>
              </div>
              <div class="info-row">
                <span class="info-label">판매자</span>
                <span class="info-value">{{ deliveryData.sellerName || '판매자명 없음' }}</span>
              </div>
              <div class="info-row">
                <span class="info-label">배송 ID</span>
                <span class="info-value">#{{ deliveryData.deliveryId }}</span>
              </div>
            </div>
          </div>

          <div class="instruction-card">
            <div class="instruction-steps">
              <div class="step">
                <div class="step-number">1</div>
                <p class="step-text">나르고의 문을 열어주세요</p>
              </div>
              <div class="step">
                <div class="step-number">2</div>
                <p class="step-text">주문하신 상품을 꺼내주세요</p>
              </div>
              <div class="step">
                <div class="step-number">3</div>
                <p class="step-text">아래 완료 버튼을 눌러주세요</p>
              </div>
            </div>
          </div>
        </div>
      </div>

      <div class="modal-footer">
        <button class="secondary-button" @click="closeModal" :disabled="loading">
          <span>나중에 하기</span>
        </button>
        <button
          class="primary-button"
          @click="confirmPickup"
          :disabled="loading"
        >
          <div v-if="loading" class="loading-spinner"></div>
          <span>{{ loading ? '처리 중...' : '물건 수령 완료' }}</span>
          <svg v-if="!loading" width="20" height="20" viewBox="0 0 24 24" fill="none">
            <path d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
          </svg>
        </button>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, watch, onMounted } from 'vue'
import { confirmBuyerPickup } from '../api/delivery/delivery.js'

const props = defineProps({
  isVisible: {
    type: Boolean,
    default: false
  },
  deliveryData: {
    type: Object,
    default: null
  }
})

const emit = defineEmits(['close', 'pickup-confirmed'])

const loading = ref(false)

// Props 변화 감지
watch(() => props.isVisible, (newValue) => {
  console.log('🔔 [구매자 모달] isVisible 변경됨:', newValue)
  if (newValue) {
    console.log('🔔 [구매자 모달] ✅ 모달창 표시되어야 함')
    console.log('🔔 [구매자 모달] deliveryData:', props.deliveryData)
  } else {
    console.log('🔔 [구매자 모달] ❌ 모달창 숨김')
  }
})

// 컴포넌트 마운트 시 초기 상태 로그
onMounted(() => {
  console.log('🔔 [구매자 모달] 컴포넌트 마운트됨')
  console.log('🔔 [구매자 모달] 초기 isVisible:', props.isVisible)
  console.log('🔔 [구매자 모달] 초기 deliveryData:', props.deliveryData)
})

const closeModal = () => {
  console.log('🔔 [구매자 모달] 취소 버튼 클릭됨')
  console.log('🔔 [구매자 모달] loading 상태:', loading.value)

  if (!loading.value) {
    console.log('🔔 [구매자 모달] ✅ 로딩 중이 아니라 모달 닫기 처리')
    emit('close')
    console.log('🔔 [구매자 모달] close 이벤트 발송 완료')
  } else {
    console.log('🔔 [구매자 모달] ❌ 로딩 중이라 닫기 방지')
  }
}

const confirmPickup = async () => {
  console.log('🔔 [구매자 모달] 물건 수령 완료 버튼 클릭됨')
  console.log('🔔 [구매자 모달] props.deliveryData:', props.deliveryData)

  if (!props.deliveryData?.deliveryId) {
    console.log('🔔 [구매자 모달] ❌ deliveryId가 없음')
    alert('배송 정보가 올바르지 않습니다.')
    return
  }

  console.log('🔔 [구매자 모달] ✅ deliveryId 확인:', props.deliveryData.deliveryId)
  loading.value = true
  console.log('🔔 [구매자 모달] loading 상태 시작')

  try {
    console.log('📦 [구매자 모달] 물건 수령 완료 확인 중 - deliveryId:', props.deliveryData.deliveryId)

    // 백엔드에 물건 수령 완료 신호 전송
    console.log('📦 [구매자 모달] confirmBuyerPickup API 호출 시작')
    const response = await confirmBuyerPickup(props.deliveryData.deliveryId)

    console.log('✅ [구매자 모달] 물건 수령 완료 확인 성공:', response)

    // 성공 알림
    alert('물건 수령이 완료되었습니다! 배송이 완료되었습니다.')

    // 부모 컴포넌트에 완료 신호 전송
    emit('pickup-confirmed', {
      deliveryId: props.deliveryData.deliveryId,
      response: response
    })

    // 모달 닫기
    emit('close')

  } catch (error) {
    console.error('❌ [구매자 모달] 물건 수령 완료 확인 실패:', error)

    // 사용자에게 에러 메시지 표시
    alert(error.response?.data?.message ||
          error.message ||
                        '물건 수령 완료 확인 중 오류가 발생했습니다.')
  } finally {
    loading.value = false
    console.log('🔔 [구매자 모달] loading 상태 종료')
  }
}
</script>

<style scoped>
.modal-overlay {
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background: rgba(0, 0, 0, 0.6);
  backdrop-filter: blur(8px);
  display: flex;
  justify-content: center;
  align-items: center;
  z-index: 9999;
  animation: fadeIn 0.3s ease-out;
}

@keyframes fadeIn {
  from { opacity: 0; }
  to { opacity: 1; }
}

.modal-container {
  background: white;
  border-radius: 20px;
  padding: 0;
  max-width: 520px;
  width: 90%;
  max-height: 85vh;
  overflow: hidden;
  box-shadow: 0 20px 60px rgba(0, 0, 0, 0.2);
  animation: slideIn 0.4s ease-out;
}

@keyframes slideIn {
  from {
    opacity: 0;
    transform: translateY(30px) scale(0.95);
  }
  to {
    opacity: 1;
    transform: translateY(0) scale(1);
  }
}

.modal-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 24px 28px 20px;
  background: linear-gradient(135deg, #27AE60 0%, #2ECC71 100%);
  color: white;
}

.header-content {
  flex: 1;
  text-align: center;
}

.logo-container {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 12px;
  margin-bottom: 8px;
}

.nareugo-logo {
  width: 48px;
  height: 48px;
}

.brand-name {
  font-size: 16px;
  font-weight: 700;
  letter-spacing: 0.5px;
}

.header-content h2 {
  margin: 0;
  font-size: 18px;
  font-weight: 600;
  opacity: 0.95;
}

.close-button {
  background: rgba(255, 255, 255, 0.1);
  border: none;
  cursor: pointer;
  padding: 8px;
  border-radius: 50%;
  color: white;
  transition: all 0.2s ease;
  backdrop-filter: blur(10px);
}

.close-button:hover {
  background: rgba(255, 255, 255, 0.2);
  transform: scale(1.05);
}

.modal-body {
  padding: 32px 28px;
  overflow-y: auto;
  max-height: calc(85vh - 200px);
}

.loading {
  text-align: center;
  padding: 60px 20px;
  color: #64748b;
  font-size: 16px;
}

.pickup-info {
  text-align: center;
}

.arrival-status {
  margin-bottom: 32px;
}

.status-icon-container {
  position: relative;
  display: inline-block;
  margin-bottom: 20px;
}

.pulse-ring {
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  width: 120px;
  height: 120px;
  border: 2px solid #27AE60;
  border-radius: 20px;
  opacity: 0.4;
  animation: deliveryPulse 3s ease-in-out infinite;
}

.pulse-ring::before {
  content: '';
  position: absolute;
  top: -10px;
  left: -10px;
  right: -10px;
  bottom: -10px;
  border: 1px solid #2ECC71;
  border-radius: 25px;
  opacity: 0.3;
  animation: deliveryPulse 3s ease-in-out infinite 0.5s;
}

.pulse-ring::after {
  content: '';
  position: absolute;
  top: -20px;
  left: -20px;
  right: -20px;
  bottom: -20px;
  border: 1px solid #16A085;
  border-radius: 30px;
  opacity: 0.2;
  animation: deliveryPulse 3s ease-in-out infinite 1s;
}

@keyframes deliveryPulse {
  0% {
    transform: translate(-50%, -50%) scale(0.7);
    opacity: 0.6;
  }
  50% {
    transform: translate(-50%, -50%) scale(1);
    opacity: 0.3;
  }
  100% {
    transform: translate(-50%, -50%) scale(1.3);
    opacity: 0;
  }
}

.robot-logo {
  position: relative;
  width: 80px;
  height: 80px;
  z-index: 2;
  animation: logoFloat 4s ease-in-out infinite;
  filter: drop-shadow(0 4px 12px rgba(39, 174, 96, 0.3));
}

@keyframes logoFloat {
  0%, 100% {
    transform: translateY(0px);
  }
  50% {
    transform: translateY(-8px);
  }
}

.status-title {
  margin: 0 0 8px 0;
  font-size: 24px;
  font-weight: 700;
  color: #1e293b;
  letter-spacing: -0.5px;
}

.status-subtitle {
  margin: 0;
  font-size: 16px;
  color: #64748b;
  font-weight: 500;
}

.delivery-card {
  background: linear-gradient(135deg, #f8fafc 0%, #e2e8f0 100%);
  border-radius: 16px;
  padding: 24px;
  margin-bottom: 28px;
  border: 1px solid #e2e8f0;
  text-align: left;
}

.card-header {
  display: flex;
  align-items: center;
  gap: 12px;
  margin-bottom: 20px;
}

.card-icon {
  width: 20px;
  height: 28px;
  background: white;
  border: 1px solid #ddd;
  border-radius: 2px 2px 4px 4px;
  position: relative;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
}

.card-icon::before {
  content: '';
  position: absolute;
  top: 4px;
  left: 2px;
  right: 2px;
  height: 1px;
  background: #666;
  box-shadow:
    0 3px 0 #666,
    0 6px 0 #666,
    0 9px 0 #666,
    0 12px 0 #666;
}

.card-icon::after {
  content: '';
  position: absolute;
  top: -2px;
  left: 2px;
  right: 2px;
  height: 4px;
  background:
    radial-gradient(circle at 2px 2px, transparent 1px, white 1px),
    radial-gradient(circle at 6px 2px, transparent 1px, white 1px),
    radial-gradient(circle at 10px 2px, transparent 1px, white 1px),
    radial-gradient(circle at 14px 2px, transparent 1px, white 1px);
  background-size: 4px 4px;
}

.card-header h4 {
  margin: 0;
  font-size: 18px;
  font-weight: 600;
  color: #334155;
}

.info-grid {
  display: flex;
  flex-direction: column;
  gap: 16px;
}

.info-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12px 0;
  border-bottom: 1px solid rgba(226, 232, 240, 0.5);
}

.info-row:last-child {
  border-bottom: none;
}

.info-label {
  font-weight: 500;
  color: #64748b;
  font-size: 14px;
}

.info-value {
  font-weight: 600;
  color: #334155;
  font-size: 15px;
}

.instruction-card {
  background: linear-gradient(135deg, #ecfdf5 0%, #d1fae5 100%);
  border-radius: 16px;
  padding: 24px;
  border: 1px solid #10b981;
}

.instruction-steps {
  display: flex;
  flex-direction: column;
  gap: 16px;
}

.step {
  display: flex;
  align-items: center;
  gap: 16px;
}

.step-number {
  width: 32px;
  height: 32px;
  background: linear-gradient(135deg, #10b981, #059669);
  color: white;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: 700;
  font-size: 14px;
  flex-shrink: 0;
}

.step-text {
  margin: 0;
  font-size: 15px;
  font-weight: 500;
  color: #047857;
  line-height: 1.5;
}

.modal-footer {
  display: flex;
  gap: 12px;
  padding: 24px 28px;
  background: #f8fafc;
  border-top: 1px solid #e2e8f0;
}

.secondary-button {
  flex: 1;
  background: white;
  color: #64748b;
  border: 2px solid #e2e8f0;
  padding: 14px 20px;
  border-radius: 12px;
  cursor: pointer;
  font-weight: 600;
  font-size: 15px;
  transition: all 0.2s ease;
}

.secondary-button:hover:not(:disabled) {
  border-color: #cbd5e1;
  color: #475569;
  background: #f8fafc;
}

.primary-button {
  flex: 2;
  background: linear-gradient(135deg, #27AE60, #2ECC71);
  color: white;
  border: none;
  padding: 14px 20px;
  border-radius: 12px;
  cursor: pointer;
  font-weight: 600;
  font-size: 15px;
  transition: all 0.2s ease;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  box-shadow: 0 4px 20px rgba(39, 174, 96, 0.3);
}

.primary-button:hover:not(:disabled) {
  background: linear-gradient(135deg, #2ECC71, #27AE60);
  transform: translateY(-1px);
  box-shadow: 0 6px 25px rgba(39, 174, 96, 0.4);
}

.loading-spinner {
  width: 20px;
  height: 20px;
  border: 2px solid rgba(255, 255, 255, 0.3);
  border-top: 2px solid white;
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}

.secondary-button:disabled,
.primary-button:disabled {
  opacity: 0.6;
  cursor: not-allowed;
  transform: none;
}

.primary-button:disabled {
  box-shadow: none;
}
</style>