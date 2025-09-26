<template>
  <div v-if="isVisible" class="modal-overlay" @click="closeModal">
    <div class="modal-container" @click.stop>
      <div class="modal-header">
        <h2>나르고 시작하기</h2>
        <button class="close-button" @click="closeModal">×</button>
      </div>

      <div class="modal-body">
        <!-- 배송 시작 성공 화면 -->
        <div v-if="showDeliveryStarted" class="delivery-started-screen">
          <div class="success-animation">
            <div class="robot-icon">🤖</div>
            <div class="success-message">
              <h3>배송 시작!</h3>
              <p>나르고가 판매자 주소로 이동 중입니다.</p>
            </div>
          </div>
          <div class="delivery-progress">
            <div class="progress-step active">
              <div class="step-icon">📍</div>
              <div class="step-text">
                <div class="step-title">이동 중</div>
                <div class="step-desc">판매자 주소로 이동</div>
              </div>
            </div>
            <div class="progress-step">
              <div class="step-icon">📦</div>
              <div class="step-text">
                <div class="step-title">대기 중</div>
                <div class="step-desc">물건 픽업 대기</div>
              </div>
            </div>
            <div class="progress-step">
              <div class="step-icon">🚀</div>
              <div class="step-text">
                <div class="step-title">예정</div>
                <div class="step-desc">구매자에게 배송</div>
              </div>
            </div>
          </div>
        </div>

        <!-- 기존 상품 선택 화면 -->
        <div v-else>
          <div v-if="loading" class="loading">
            데이터를 불러오는 중...
          </div>

          <div v-else-if="paidProducts.length === 0" class="no-products">
            결제 완료된 상품이 없습니다.
          </div>

          <div v-else class="product-list">
            <h3>결제 완료된 상품을 선택하세요 (1개)</h3>
            <div class="product-items">
              <label
                v-for="product in paidProducts"
                :key="product.id"
                class="product-item"
                :class="{
                  'selected': selectedProduct?.id === product.id,
                  'disabled': product.deliveryStarted
                }"
              >
                <input
                  type="radio"
                  :value="product.id"
                  v-model="selectedProductId"
                  @change="selectProduct(product)"
                  :disabled="product.deliveryStarted"
                />
                <div class="product-info">
                  <img
                    v-if="product.imageUrl"
                    :src="product.imageUrl"
                    :alt="product.title"
                    class="product-image"
                  />
                  <div class="product-details">
                    <h4>{{ product.title }}</h4>
                    <p class="product-price">{{ formatPrice(product.price) }}원</p>
                    <p class="product-buyer">구매자: {{ product.buyerName }}</p>
                    <p v-if="product.deliveryStarted" class="delivery-status">🤖 배송 중</p>
                  </div>
                </div>
              </label>
            </div>
          </div>
        </div>
      </div>

      <div class="modal-footer">
        <button v-if="showDeliveryStarted" class="confirm-button" @click="closeModal">
          확인
        </button>
        <template v-else>
          <button class="cancel-button" @click="closeModal">취소</button>
          <button
            class="start-button"
            @click="startDelivery"
            :disabled="!selectedProduct || deliveryStarting || selectedProduct?.deliveryStarted"
          >
            {{ deliveryStarting ? '배송 시작 중...' :
               selectedProduct?.deliveryStarted ? '이미 배송 중' : '나르고 시작하기' }}
          </button>
        </template>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, watch } from 'vue'
import { getPaidSalesProducts, startDelivery as startDeliveryAPI } from '../api/delivery/delivery.js'
import { sendAddressesToROS2 } from '../utils/ros2Communication.js'
import { checkStoreAvailability } from '../utils/sellerNotification.js'

const props = defineProps({
  isVisible: {
    type: Boolean,
    default: false
  }
})

const emit = defineEmits(['close', 'delivery-started'])

const loading = ref(false)
const deliveryStarting = ref(false)
const paidProducts = ref([])
const selectedProductId = ref(null)
const selectedProduct = ref(null)
const showDeliveryStarted = ref(false)

const selectProduct = (product) => {
  selectedProduct.value = product
}

const formatPrice = (price) => {
  return new Intl.NumberFormat('ko-KR').format(price)
}

const getUserId = () => {
  // localStorage의 모든 키 확인
  console.log('🔍 [DEBUG] localStorage의 모든 키들:', Object.keys(localStorage))

  // 가능한 사용자 정보 키들 확인
  const possibleKeys = ['user_info', 'userInfo', 'user', 'auth', 'token', 'access_token']
  for (const key of possibleKeys) {
    const value = localStorage.getItem(key)
    console.log(`🔍 [DEBUG] localStorage[${key}]:`, value)
  }

  const userInfo = localStorage.getItem('user_info')
  console.log('🔍 [DEBUG] localStorage의 user_info:', userInfo)

  if (userInfo) {
    try {
      const parsedInfo = JSON.parse(userInfo)
      console.log('🔍 [DEBUG] 파싱된 사용자 정보:', parsedInfo)

      const userId = parsedInfo.id || parsedInfo.userId || parsedInfo.user_id
      console.log('🔍 [DEBUG] 추출된 userId:', userId)

      if (userId) {
        return userId
      }
    } catch (error) {
      console.error('❌ [ERROR] user_info 파싱 실패:', error)
    }
  }

  // 다른 가능한 키들에서 사용자 정보 찾기
  for (const key of possibleKeys) {
    const value = localStorage.getItem(key)
    if (value && value !== 'null' && value !== 'undefined') {
      try {
        const parsed = JSON.parse(value)
        console.log(`🔍 [DEBUG] ${key}에서 파싱된 정보:`, parsed)

        const userId = parsed.id || parsed.userId || parsed.user_id
        if (userId) {
          console.log(`🔍 [DEBUG] ${key}에서 추출된 userId:`, userId)
          return userId
        }
      } catch (error) {
        // JSON이 아닌 경우 무시
        console.log(`🔍 [DEBUG] ${key}는 JSON이 아님:`, value)
      }
    }
  }

  console.warn('⚠️ [WARNING] localStorage에서 사용자 정보를 찾을 수 없습니다')
  console.log('🔍 [DEBUG] 테스트용 userId 반환: 1')
  return 1 // 테스트용으로 임시 userId 반환 (null 대신 1을 반환)
}

const loadPaidProducts = async () => {
  try {
    loading.value = true
    
    // 로그인한 사용자의 실제 ID 사용
    const userId = getUserId()
    console.log('🔍 [DEBUG] loadPaidProducts - 로그인한 사용자 ID 사용:', userId)
    
    // userId가 없으면 기본값 사용 (개발/테스트용)
    const finalUserId = userId || 3
    console.log('🔍 [DEBUG] loadPaidProducts - 최종 사용할 userId:', finalUserId)

    const response = await getPaidSalesProducts(finalUserId)
    console.log('🔍 [DEBUG] DeliveryStartModal에서 받은 응답:', response)
    paidProducts.value = response || []
  } catch (error) {
    console.error('결제 완료된 상품 조회 실패:', error)
    console.log('🔍 [DEBUG] 에러 발생, 빈 배열로 설정')
    paidProducts.value = []
  } finally {
    loading.value = false
  }
}

const startDelivery = async () => {
  if (!selectedProduct.value) {
    alert('배송할 상품을 선택해주세요.')
    return
  }

  try {
    deliveryStarting.value = true

    // 1. 가게 가능 여부 확인
    console.log('🏪 [DEBUG] 가게 가능 여부 확인 시작')
    const availability = await checkStoreAvailability()
    console.log('🏪 [DEBUG] 가게 가능 여부 결과:', availability)

    if (!availability.isAvailable) {
      alert(`배송을 시작할 수 없습니다.\n\n사유: ${availability.reason}`)
      return
    }

    console.log('✅ [DEBUG] 가게 가능 여부 확인 완료 - 배송 가능')

    // 2. 로봇에게 주소 정보 요청 및 전송
    const deliveryData = {
      deliveryId: selectedProduct.value.deliveryId,
      productId: selectedProduct.value.id,
      buyerId: selectedProduct.value.buyerId,
      sellerId: getUserId(),
      robotId: 1
    }

    console.log('🚀 [DEBUG] 배송 데이터:', deliveryData)
    const result = await startDeliveryAPI(deliveryData)
    console.log('🚀 [DEBUG] 나르고 시작 결과:', result)
    console.log('🏠 [DEBUG] 로봇에게 전송된 주소:', result.addresses)

    // 백엔드에서 로봇 서버로 자동 전송하므로 프론트엔드에서는 추가 작업 불필요

    // 3. 현재 진행 중인 배송 ID를 세션에 저장
    sessionStorage.setItem('currentDeliveryId', deliveryData.deliveryId.toString())
    console.log('💾 [DEBUG] 현재 배송 ID 저장:', deliveryData.deliveryId)

    // 4. 배송 시작된 상품 상태 업데이트
    const targetProduct = paidProducts.value.find(p => p.id === selectedProduct.value.id)
    if (targetProduct) {
      targetProduct.deliveryStarted = true
    }

    // 5. 배송 시작 성공 화면 표시
    showDeliveryStarted.value = true

    emit('delivery-started', selectedProduct.value)

    // 6. 로봇 도착은 실제 로봇에서 처리
    // 로봇이 판매자 호수에 도착하면 백엔드의 /robot/delivery/{deliveryId}/seller/arrived 엔드포인트가 호출되고
    // 백엔드에서 FCM을 통해 판매자에게 알림을 전송합니다.
    console.log('🤖 [INFO] 로봇이 판매자 집으로 이동을 시작합니다.')
    console.log('🤖 [INFO] 로봇 도착 시 백엔드에서 자동으로 FCM 알림이 전송됩니다.')

    // 7. 5초 후 자동으로 모달 닫기 (선택사항)
    setTimeout(() => {
      closeModal()
    }, 5000)
  } catch (error) {
    console.error('❌ [ERROR] 배송 시작 실패:', error)
    const errorMessage = error.response?.data?.message || error.message || '배송 시작에 실패했습니다.'
    alert(`배송 시작 실패: ${errorMessage}`)
  } finally {
    deliveryStarting.value = false
  }
}


const closeModal = () => {
  selectedProductId.value = null
  selectedProduct.value = null
  showDeliveryStarted.value = false
  emit('close')
}

// props.isVisible이 변경될 때마다 데이터 로드
watch(() => props.isVisible, (newValue) => {
  console.log('🔍 [DEBUG] 모달 가시성 변경:', newValue)
  if (newValue) {
    console.log('🔍 [DEBUG] 모달이 열림, 데이터 로드 시작')
    loadPaidProducts()
  } else {
    console.log('🔍 [DEBUG] 모달이 닫힘')
  }
}, { immediate: true })

onMounted(() => {
  console.log('🔍 [DEBUG] DeliveryStartModal 컴포넌트 마운트됨')
  if (props.isVisible) {
    console.log('🔍 [DEBUG] 마운트 시 모달이 이미 열려있음, 데이터 로드')
    loadPaidProducts()
  }
})
</script>

<style scoped>
.modal-overlay {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: rgba(0, 0, 0, 0.5);
  display: flex;
  justify-content: center;
  align-items: center;
  z-index: 9999;
}

.modal-container {
  background: white;
  border-radius: 16px;
  width: 90%;
  max-width: 500px;
  max-height: 80vh;
  overflow: hidden;
  box-shadow: 0 10px 30px rgba(0, 0, 0, 0.3);
}

.modal-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 20px 24px;
  border-bottom: 1px solid #eee;
}

.modal-header h2 {
  margin: 0;
  font-size: 18px;
  font-weight: 600;
  color: #2c3e50;
}

.close-button {
  background: none;
  border: none;
  font-size: 24px;
  cursor: pointer;
  padding: 0;
  width: 32px;
  height: 32px;
  display: flex;
  align-items: center;
  justify-content: center;
  color: #666;
}

.close-button:hover {
  color: #4682B4;
}

.modal-body {
  padding: 24px;
  max-height: 400px;
  overflow-y: auto;
}

.loading, .no-products {
  text-align: center;
  padding: 40px 20px;
  color: #666;
}

.product-list h3 {
  margin: 0 0 16px 0;
  font-size: 16px;
  font-weight: 500;
  color: #333;
}

.product-items {
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.product-item {
  display: block;
  padding: 16px;
  border: 2px solid #eee;
  border-radius: 12px;
  cursor: pointer;
  transition: all 0.2s ease;
}

.product-item:hover {
  border-color: #5A9BD6;
  background-color: #EFF6FB;
}

.product-item.selected {
  border-color: #4682B4;
  background-color: #EAF3FB;
}

.product-item input[type="radio"] {
  display: none;
}

.product-info {
  display: flex;
  gap: 12px;
  align-items: center;
}

.product-image {
  width: 60px;
  height: 60px;
  object-fit: cover;
  border-radius: 8px;
  flex-shrink: 0;
}

.product-details {
  flex: 1;
}

.product-details h4 {
  margin: 0 0 4px 0;
  font-size: 14px;
  font-weight: 500;
  color: #2c3e50;
}

.product-price {
  margin: 0 0 4px 0;
  font-size: 14px;
  font-weight: 600;
  color: #4682B4;
}

.product-buyer {
  margin: 0;
  font-size: 12px;
  color: #6b7280;
}

.delivery-status {
  margin: 4px 0 0 0;
  font-size: 12px;
  color: #16a34a;
  font-weight: 600;
}

.product-item.disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.product-item.disabled input {
  cursor: not-allowed;
}

.product-item.disabled .product-info {
  pointer-events: none;
}

.modal-footer {
  display: flex;
  gap: 12px;
  padding: 20px 24px;
  border-top: 1px solid #eee;
}

.cancel-button, .start-button {
  flex: 1;
  padding: 12px 24px;
  border: none;
  border-radius: 8px;
  font-size: 14px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s ease;
}

.cancel-button {
  background-color: #EEF3F8;
  color: #5f6b78;
}

.cancel-button:hover {
  background-color: #E2EBF4;
}

.start-button {
  /* App header/login button gradient */
  background: linear-gradient(90deg, #4682B4, #6EC6CA);
  color: white;
}

.start-button:hover:not(:disabled) {
  background: linear-gradient(90deg, #5A9BD6, #7FD7DA);
}

.start-button:disabled {
  background-color: #b7c7d6;
  cursor: not-allowed;
}

/* 배송 시작 성공 화면 스타일 */
.delivery-started-screen {
  text-align: center;
  padding: 20px;
}

.success-animation {
  margin-bottom: 32px;
}

.robot-icon {
  font-size: 80px;
  margin-bottom: 20px;
  animation: bounce 2s infinite;
}

@keyframes bounce {
  0%, 20%, 50%, 80%, 100% {
    transform: translateY(0);
  }
  40% {
    transform: translateY(-20px);
  }
  60% {
    transform: translateY(-10px);
  }
}

.success-message h3 {
  margin: 0 0 8px 0;
  font-size: 24px;
  font-weight: 700;
  color: #16a34a;
}

.success-message p {
  margin: 0;
  font-size: 16px;
  color: #6b7280;
}

.delivery-progress {
  display: flex;
  flex-direction: column;
  gap: 16px;
  max-width: 300px;
  margin: 0 auto;
}

.progress-step {
  display: flex;
  align-items: center;
  gap: 16px;
  padding: 12px;
  border-radius: 8px;
  transition: all 0.3s ease;
}

.progress-step.active {
  background-color: #ecfdf5;
  border: 2px solid #16a34a;
}

.progress-step:not(.active) {
  background-color: #f9fafb;
  border: 2px solid #e5e7eb;
}

.step-icon {
  font-size: 24px;
  width: 40px;
  height: 40px;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: 50%;
  flex-shrink: 0;
}

.progress-step.active .step-icon {
  background-color: #16a34a;
  color: white;
}

.progress-step:not(.active) .step-icon {
  background-color: #e5e7eb;
  color: #9ca3af;
}

.step-text {
  flex: 1;
  text-align: left;
}

.step-title {
  font-weight: 600;
  font-size: 14px;
  margin-bottom: 2px;
}

.progress-step.active .step-title {
  color: #16a34a;
}

.progress-step:not(.active) .step-title {
  color: #6b7280;
}

.step-desc {
  font-size: 12px;
  color: #9ca3af;
}

.confirm-button {
  flex: 1;
  padding: 12px 24px;
  border: none;
  border-radius: 8px;
  font-size: 14px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s ease;
  background: linear-gradient(90deg, #4682B4, #6EC6CA);
  color: white;
}

.confirm-button:hover {
  background: linear-gradient(90deg, #5A9BD6, #7FD7DA);
}
</style>