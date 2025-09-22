<template>
  <div v-if="isVisible" class="modal-overlay" @click="closeModal">
    <div class="modal-container" @click.stop>
      <div class="modal-header">
        <h2>나르고 시작하기</h2>
        <button class="close-button" @click="closeModal">×</button>
      </div>

      <div class="modal-body">
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
              :class="{ 'selected': selectedProduct?.id === product.id }"
            >
              <input
                type="radio"
                :value="product.id"
                v-model="selectedProductId"
                @change="selectProduct(product)"
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
                </div>
              </div>
            </label>
          </div>
        </div>
      </div>

      <div class="modal-footer">
        <button class="cancel-button" @click="closeModal">취소</button>
        <button
          class="start-button"
          @click="startDelivery"
          :disabled="!selectedProduct || deliveryStarting"
        >
          {{ deliveryStarting ? '배송 시작 중...' : '나르고 시작하기' }}
        </button>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, watch } from 'vue'
import { getPaidSalesProducts, getRobotStatus, startDelivery as startDeliveryAPI } from '../api/delivery/delivery.js'

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

    const robotStatus = await getRobotStatus(1)

    if (robotStatus.status === 'INVALID') {
      alert('배송 불가: 로봇이 현재 사용할 수 없는 상태입니다.')
      return
    }

    if (robotStatus.status === 'VALID') {
      const deliveryData = {
        productId: selectedProduct.value.id,
        buyerId: selectedProduct.value.buyerId,
        sellerId: getUserId(),
        robotId: 1
      }

      await startDeliveryAPI(deliveryData)
      alert('배송을 시작했습니다!')
      emit('delivery-started', selectedProduct.value)
      closeModal()
    }
  } catch (error) {
    console.error('배송 시작 실패:', error)
    alert('배송 시작에 실패했습니다. 다시 시도해주세요.')
  } finally {
    deliveryStarting.value = false
  }
}

const closeModal = () => {
  selectedProductId.value = null
  selectedProduct.value = null
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
</style>