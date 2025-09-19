<template>
  <div class="delivery-cards">
    <!-- 섹션 제목 -->
    <div class="section-title">나르고 위치 조회</div>

    <div class="cards-container">
      <!-- 택배 현황 카드 (API 데이터 기반) -->
      <div v-for="delivery in activeDeliveries" :key="delivery.deliveryId" class="delivery-card" @click="openDetailModal(delivery)">
        <div class="card-header">
          <div class="delivery-icon">📦</div>
          <div class="delivery-info">
            <div class="delivery-company">{{ delivery.title || '나르고' }}</div>
            <div class="delivery-number">#{{ delivery.trackingNumber }}</div>
          </div>
        </div>
        <div class="card-content">
          <div class="status-info">
            <div class="status-text">{{ getStatusText(delivery.status) }}</div>
            <div class="status-time">{{ formatDateTime(getStatusTime(delivery)) }}</div>
          </div>
          <div class="progress-bar">
            <!-- 접수완료 -->
            <div :class="['progress-step', getStepClass('RECEIPT_COMPLETED', delivery.status)]">
              <div v-if="delivery.status === 'RECEIPT_COMPLETED'" class="step-time">
                {{ formatDateTime(delivery.completeTime) }}
              </div>
              <div class="step-dot"></div>
              <div class="step-label">접수완료</div>
            </div>
            <div :class="['progress-line', getLineClass('RECEIPT_COMPLETED', delivery.status)]"></div>

            <!-- 배달중 -->
            <div :class="['progress-step', getStepClass('IN_DELIVERY', delivery.status)]">
              <div v-if="delivery.status === 'IN_DELIVERY'" class="step-time">
                {{ formatDateTime(delivery.completeTime) }}
              </div>
              <div class="step-dot"></div>
              <div class="step-label">배달중</div>
            </div>
            <div :class="['progress-line', getLineClass('IN_DELIVERY', delivery.status)]"></div>

            <!-- 완료 -->
            <div :class="['progress-step', getStepClass('DELIVERY_COMPLETED', delivery.status)]">
              <div v-if="delivery.status === 'DELIVERY_COMPLETED'" class="step-time">
                {{ formatDateTime(delivery.completeTime) }}
              </div>
              <div class="step-dot"></div>
              <div class="step-label">완료</div>
            </div>
          </div>
        </div>
      </div>

      <!-- 데이터가 없을 때 표시 -->
      <div v-if="activeDeliveries.length === 0" class="no-deliveries">
        <div class="no-data-icon">📦</div>
        <div class="no-data-text">현재 진행 중인 배송이 없습니다</div>
      </div>
    </div>

    <!-- 최근 배달 내역 섹션 -->
    <div class="delivery-history-section">
      <div class="history-title">최근 배달 내역</div>
      
      <div class="history-cards">
        <!-- 배달 완료 카드 1 -->
        <div class="history-card">
          <div class="card-header">
            <div class="delivery-icon">📦</div>
            <div class="delivery-info">
              <div class="delivery-number">#S65F1ES51F2FD</div>
              <div class="delivery-date">09/02 10:16</div>
            </div>
          </div>
        </div>

        <!-- 배달 완료 카드 2 -->
        <div class="history-card">
          <div class="card-header">
            <div class="delivery-icon">📦</div>
            <div class="delivery-info">
              <div class="delivery-number">#GB1BF56T1BTF2</div>
              <div class="delivery-date">08/22 08:35</div>
            </div>
          </div>
        </div>

        <!-- 배달 완료 카드 3 -->
        <div class="history-card">
          <div class="card-header">
            <div class="delivery-icon">📦</div>
            <div class="delivery-info">
              <div class="delivery-number">#G1R56DG156DR</div>
              <div class="delivery-date">08/20 17:45</div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 택배 상세 모달 -->
    <DeliveryDetailModal
      :isVisible="isModalVisible"
      :delivery="selectedDelivery || {}"
      @close="closeDetailModal"
      @inquiry="handleInquiry"
      @change-address="handleAddressChange"
    />
  </div>
</template>

<script setup>
import { ref, onMounted, computed } from 'vue';
import { useAuthStore } from '@/stores/auth';
import { getDeliveries } from '@/api/delivery/delivery';
import DeliveryDetailModal from './DeliveryDetailModal.vue';

const authStore = useAuthStore();
const deliveries = ref([]);
const isLoading = ref(false);

// 모달 관련 상태
const isModalVisible = ref(false);
const selectedDelivery = ref(null);

// 배송 상태별 순서 정의 (3단계로 수정)
const statusOrder = {
  'RECEIPT_COMPLETED': 0,
  'IN_DELIVERY': 1,
  'DELIVERY_COMPLETED': 2,
  'CANCELLED': -1
};

// 배송 상태 텍스트 매핑 (DELIVERY_BEGIN 제거)
const statusTexts = {
  'RECEIPT_COMPLETED': '접수완료',
  'IN_DELIVERY': '배달중',
  'DELIVERY_COMPLETED': '배송완료',
  'CANCELLED': '취소됨'
};

// 컴포넌트 마운트 시 배송 데이터 조회
onMounted(async () => {
  await loadDeliveries();
});

// 배송 데이터 조회 함수
const loadDeliveries = async () => {
  if (!authStore.user?.userId) {
    console.log('사용자 정보가 없습니다.');
    return;
  }

  try {
    isLoading.value = true;
    const data = await getDeliveries(authStore.user.userId);
    deliveries.value = data || [];
    console.log('배송 데이터 조회 성공:', deliveries.value);
    console.log('첫 번째 배송 데이터 구조:', deliveries.value[0]);
  } catch (error) {
    console.error('배송 데이터 조회 실패:', error);
    deliveries.value = [];
  } finally {
    isLoading.value = false;
  }
};

// 상태 텍스트 가져오기
const getStatusText = (status) => {
  return statusTexts[status] || status;
};

// 상태에 따른 시간 가져오기 (현재 상태의 완료 시간)
const getStatusTime = (delivery) => {
  return delivery.completeTime || delivery.requestTime;
};

// 단계별 클래스 결정 (completed, active, 또는 빈 문자열)
const getStepClass = (stepStatus, currentStatus) => {
  const stepOrder = statusOrder[stepStatus];
  const currentOrder = statusOrder[currentStatus];

  if (currentOrder === -1) return ''; // 취소된 경우

  if (stepOrder < currentOrder) {
    return 'completed'; // 현재 상태보다 이전 단계
  } else if (stepOrder === currentOrder) {
    return 'active'; // 현재 상태
  }
  return ''; // 아직 진행되지 않은 단계
};

// 진행선 클래스 결정
const getLineClass = (stepStatus, currentStatus) => {
  const stepOrder = statusOrder[stepStatus];
  const currentOrder = statusOrder[currentStatus];

  if (currentOrder === -1) return ''; // 취소된 경우

  return stepOrder < currentOrder ? 'completed' : '';
};

// 날짜 시간 포맷팅
const formatDateTime = (dateTime) => {
  if (!dateTime) return '';

  try {
    const date = new Date(dateTime);
    const month = String(date.getMonth() + 1).padStart(2, '0');
    const day = String(date.getDate()).padStart(2, '0');
    const hours = String(date.getHours()).padStart(2, '0');
    const minutes = String(date.getMinutes()).padStart(2, '0');

    return `${month}/${day} ${hours}:${minutes}`;
  } catch (error) {
    console.error('날짜 포맷팅 오류:', error);
    return '';
  }
};

// 진행 중인 배송만 필터링 (취소되지 않은 배송)
const activeDeliveries = computed(() => {
  return deliveries.value.filter(delivery => delivery.status !== 'CANCELLED');
});

// 모달 관련 함수들
const openDetailModal = (delivery) => {
  selectedDelivery.value = delivery;
  isModalVisible.value = true;
};

const closeDetailModal = () => {
  isModalVisible.value = false;
  selectedDelivery.value = null;
};

const handleInquiry = (delivery) => {
  alert('문의하기 기능이 곧 추가될 예정입니다.');
  closeDetailModal();
};

const handleAddressChange = (delivery) => {
  alert('배송지 변경 기능이 곧 추가될 예정입니다.');
  closeDetailModal();
};
</script>

<style scoped>
.delivery-cards {
  padding: 24px 20px;
  background-color: white;
  margin-top: -10px;
  border-radius: 24px 24px 0 0;
  min-height: calc(100vh - 320px);
}

.section-title {
  font-size: 16px;
  font-weight: 600;
  color: #2c3e50;
  margin-bottom: 20px;
  margin-top: 20px;
  text-align: left;
}

.cards-container {
  display: flex;
  flex-direction: column;
  gap: 20px;
  margin-bottom: 40px;
}

.delivery-card {
  background-color: white;
  border: 1px solid #f1f3f4;
  border-radius: 16px;
  padding: 20px;
  box-shadow: 0 2px 12px rgba(0, 0, 0, 0.08);
  transition: all 0.3s ease;
  cursor: pointer;
}

.delivery-card:hover {
  transform: translateY(-2px);
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.12);
}

.card-header {
  display: flex;
  align-items: center;
  margin-bottom: 20px;
}

.delivery-icon {
  font-size: 24px;
  margin-right: 16px;
}

.delivery-info {
  flex: 1;
}

.delivery-company {
  font-size: 16px;
  font-weight: 600;
  color: #2c3e50;
  margin-bottom: 6px;
}

.delivery-number {
  font-size: 14px;
  color: #6c757d;
  font-family: 'Courier New', monospace;
  font-weight: 500;
}

.card-content {
  margin-bottom: 0;
}

.status-info {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 20px;
}

.status-text {
  font-size: 15px;
  font-weight: 600;
  color: #007bff;
}

.status-time {
  font-size: 13px;
  color: #6c757d;
  font-weight: 500;
}

.progress-bar {
  display: flex;
  align-items: center;
  justify-content: space-between;
  position: relative;
}

.progress-step {
  display: flex;
  flex-direction: column;
  align-items: center;
  flex: 1;
  position: relative;
}

.step-dot {
  width: 12px;
  height: 12px;
  border-radius: 50%;
  background-color: #e9ecef;
  border: 2px solid #e9ecef;
  margin-bottom: 8px;
  transition: all 0.3s ease;
}

.progress-step.completed .step-dot {
  background-color: #007bff;
  border-color: #007bff;
}

.progress-step.active .step-dot {
  background-color: #007bff;
  border-color: #007bff;
  box-shadow: 0 0 0 4px rgba(0, 123, 255, 0.2);
  animation: pulse 1s infinite;
}

@keyframes pulse {
  0% {
    box-shadow: 0 0 0 0 rgba(0, 123, 255, 0.7);
    transform: scale(1);
  }
  50% {
    box-shadow: 0 0 0 8px rgba(0, 123, 255, 0);
    transform: scale(1.1);
  }
  100% {
    box-shadow: 0 0 0 0 rgba(0, 123, 255, 0);
    transform: scale(1);
  }
}

.step-label {
  font-size: 10px;
  color: #6c757d;
  font-weight: 500;
  text-align: center;
  line-height: 1.1;
  white-space: nowrap;
}

.step-time {
  font-size: 9px;
  color: #007bff;
  font-weight: 500;
  text-align: center;
  margin-bottom: 4px;
  white-space: nowrap;
}

.progress-step.completed .step-label {
  color: #007bff;
  font-weight: 600;
}

.progress-step.active .step-label {
  color: #007bff;
  font-weight: 600;
}

.progress-line {
  height: 2px;
  background-color: #e9ecef;
  flex: 1;
  margin: 0 8px;
  margin-top: -16px;
  transition: all 0.3s ease;
}

.progress-line.completed {
  background-color: #007bff;
}

/* 최근 배달 내역 섹션 */
.delivery-history-section {
  margin-top: 20px;
}

.history-title {
  font-size: 16px;
  font-weight: 600;
  color: #2c3e50;
  margin-bottom: 16px;
  text-align: left;
}

.history-cards {
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.history-card {
  background-color: white;
  border: 1px solid #f1f3f4;
  border-radius: 12px;
  padding: 16px;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.08);
  transition: all 0.3s ease;
}

.history-card:hover {
  transform: translateY(-2px);
  box-shadow: 0 4px 16px rgba(0, 0, 0, 0.12);
}

.history-card .card-header {
  margin-bottom: 0;
  gap: 12px;
}

.history-card .delivery-icon {
  font-size: 20px;
  margin-right: 0;
}

.history-card .delivery-number {
  font-size: 14px;
  color: #2c3e50;
  font-family: 'Courier New', monospace;
  font-weight: 500;
  margin-bottom: 4px;
}

.history-card .delivery-date {
  font-size: 12px;
  color: #6c757d;
  font-weight: 400;
}

/* 데이터 없음 스타일 */
.no-deliveries {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 60px 20px;
  text-align: center;
}

.no-data-icon {
  font-size: 48px;
  margin-bottom: 16px;
  opacity: 0.3;
}

.no-data-text {
  font-size: 14px;
  color: #6c757d;
  font-weight: 500;
}
</style>
