<template>
  <div v-if="isVisible" class="modal-overlay" @click="closeModal">
    <div class="modal-container" @click.stop>
      <!-- 모달 헤더 -->
      <div class="modal-header">
        <h2 class="modal-title">배송 상세 정보</h2>
        <button class="close-button" @click="closeModal">
          <span>&times;</span>
        </button>
      </div>

      <!-- 모달 콘텐츠 -->
      <div class="modal-content">
        <!-- 1. 핵심 정보 섹션 -->
        <div class="key-info-section">
          <div class="status-badge" :class="getStatusBadgeClass(delivery.status)">
            {{ getStatusText(delivery.status) }}
          </div>

          <div class="delivery-estimate">
            <div class="estimate-title">예상 도착 시간</div>
            <div class="estimate-time">{{ getEstimatedArrival() }}</div>
          </div>

          <div class="tracking-number">
            <div class="tracking-label">운송장 번호</div>
            <div class="tracking-value">
              <span class="tracking-text">#{{ delivery.trackingNumber }}</span>
              <button class="copy-button" @click="copyTrackingNumber">
                <span class="copy-icon">📋</span>
                복사
              </button>
            </div>
          </div>
        </div>

        <!-- 2. 실시간 배송 추적 타임라인 -->
        <div class="timeline-section">
          <h3 class="section-title">배송 추적</h3>
          <div class="timeline">
            <!-- 접수완료 -->
            <div :class="['timeline-item', getTimelineClass('RECEIPT_COMPLETED')]">
              <div class="timeline-dot"></div>
              <div class="timeline-content">
                <div class="timeline-status">접수완료</div>
                <div class="timeline-description">상품 접수가 완료되었습니다</div>
                <div v-if="getTimelineTime('RECEIPT_COMPLETED')" class="timeline-time">
                  {{ formatDetailDateTime(getTimelineTime('RECEIPT_COMPLETED')) }}
                </div>
                <div class="timeline-location">나르고 출고시작</div>
              </div>
            </div>

            <!-- 배달시작 -->
            <div :class="['timeline-item', getTimelineClass('DELIVERY_BEGIN')]">
              <div class="timeline-dot"></div>
              <div class="timeline-content">
                <div class="timeline-status">배달시작</div>
                <div class="timeline-description">배송이 시작되었습니다</div>
                <div v-if="getTimelineTime('DELIVERY_BEGIN')" class="timeline-time">
                  {{ formatDetailDateTime(getTimelineTime('DELIVERY_BEGIN')) }}
                </div>
                <div class="timeline-location">나르고 배송센터</div>
              </div>
            </div>

            <!-- 배달중 -->
            <div :class="['timeline-item', getTimelineClass('IN_DELIVERY')]">
              <div class="timeline-dot"></div>
              <div class="timeline-content">
                <div class="timeline-status">배달중</div>
                <div class="timeline-description">나르고봇이 배송 중입니다</div>
                <div v-if="getTimelineTime('IN_DELIVERY')" class="timeline-time">
                  {{ formatDetailDateTime(getTimelineTime('IN_DELIVERY')) }}
                </div>
                <div class="timeline-location">{{ delivery.destination || '배송 목적지로 이동 중' }}</div>
              </div>
            </div>

            <!-- 배송완료 -->
            <div :class="['timeline-item', getTimelineClass('DELIVERY_COMPLETED')]">
              <div class="timeline-dot"></div>
              <div class="timeline-content">
                <div class="timeline-status">배송완료</div>
                <div class="timeline-description">배송이 완료되었습니다</div>
                <div v-if="getTimelineTime('DELIVERY_COMPLETED')" class="timeline-time">
                  {{ formatDetailDateTime(getTimelineTime('DELIVERY_COMPLETED')) }}
                </div>
                <div class="timeline-location">목적지 도착</div>
              </div>
            </div>
          </div>
        </div>

        <!-- 3. 주문 정보 섹션 -->
        <div class="order-info-section">
          <h3 class="section-title">주문 정보</h3>
          <div class="order-details">
            <div class="order-item">
              <div class="item-image">📦</div>
              <div class="item-info">
                <div class="item-name">{{ delivery.title || '나르고 배송 상품' }}</div>
                <div class="item-description">수량: 1개</div>
              </div>
            </div>
          </div>

          <div class="recipient-info">
            <h4 class="info-subtitle">수령인 정보</h4>
            <div class="recipient-details">
              <div class="recipient-item">
                <span class="label">받는 분:</span>
                <span class="value">{{ maskName(delivery.recipientName) }}</span>
              </div>
              <div class="recipient-item">
                <span class="label">연락처:</span>
                <span class="value">{{ maskPhoneNumber(delivery.recipientPhone) }}</span>
              </div>
              <div class="recipient-item">
                <span class="label">배송지:</span>
                <span class="value">{{ maskAddress(delivery.destination) }}</span>
              </div>
            </div>
          </div>
        </div>

        <!-- 4. 고객 지원 섹션 -->
        <div class="support-section">
          <h3 class="section-title">고객 지원</h3>

          <div v-if="delivery.status === 'IN_DELIVERY'" class="driver-contact">
            <div class="contact-info">
              <div class="contact-icon">📞</div>
              <div class="contact-details">
                <div class="contact-title">배송 기사 연락처</div>
                <div class="contact-number">010-****-1234</div>
              </div>
            </div>
          </div>

          <div class="support-buttons">
            <button class="support-btn inquiry-btn">
              <span class="btn-icon">💬</span>
              문의하기
            </button>

            <button
              v-if="canChangeAddress"
              class="support-btn address-btn"
              @click="changeAddress"
            >
              <span class="btn-icon">📍</span>
              배송지 변경
            </button>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { computed, defineProps, defineEmits } from 'vue';

const props = defineProps({
  isVisible: {
    type: Boolean,
    default: false
  },
  delivery: {
    type: Object,
    default: () => ({})
  }
});

const emit = defineEmits(['close', 'inquiry', 'change-address']);

// 배송 상태 텍스트 매핑
const statusTexts = {
  'RECEIPT_COMPLETED': '접수완료',
  'DELIVERY_BEGIN': '배달시작',
  'IN_DELIVERY': '배달중',
  'DELIVERY_COMPLETED': '배송완료',
  'CANCELLED': '취소됨'
};

// 배송 상태별 순서
const statusOrder = {
  'RECEIPT_COMPLETED': 0,
  'DELIVERY_BEGIN': 1,
  'IN_DELIVERY': 2,
  'DELIVERY_COMPLETED': 3,
  'CANCELLED': -1
};

// 모달 닫기
const closeModal = () => {
  emit('close');
};

// 상태 텍스트 가져오기
const getStatusText = (status) => {
  return statusTexts[status] || status;
};

// 상태 뱃지 클래스
const getStatusBadgeClass = (status) => {
  const classes = {
    'RECEIPT_COMPLETED': 'status-pending',
    'DELIVERY_BEGIN': 'status-in-progress',
    'IN_DELIVERY': 'status-in-progress',
    'DELIVERY_COMPLETED': 'status-completed',
    'CANCELLED': 'status-cancelled'
  };
  return classes[status] || 'status-pending';
};

// 타임라인 클래스
const getTimelineClass = (stepStatus) => {
  const stepOrder = statusOrder[stepStatus];
  const currentOrder = statusOrder[props.delivery.status];

  if (currentOrder === -1) return 'timeline-cancelled';

  if (stepOrder < currentOrder) {
    return 'timeline-completed';
  } else if (stepOrder === currentOrder) {
    return 'timeline-active';
  }
  return 'timeline-pending';
};

// 타임라인 시간 가져오기
const getTimelineTime = (stepStatus) => {
  if (stepStatus === props.delivery.status) {
    return props.delivery.completeTime || props.delivery.requestTime;
  }
  return null;
};

// 예상 도착 시간 계산
const getEstimatedArrival = () => {
  if (props.delivery.status === 'DELIVERY_COMPLETED') {
    return '배송이 완료되었습니다';
  }

  const now = new Date();
  const estimatedTime = new Date(now.getTime() + 4 * 60 * 60 * 1000); // 4시간 후

  const year = estimatedTime.getFullYear();
  const month = estimatedTime.getMonth() + 1;
  const day = estimatedTime.getDate();
  const hours = estimatedTime.getHours();
  const period = hours < 12 ? '오전' : '오후';
  const displayHours = hours <= 12 ? hours : hours - 12;

  return `${year}년 ${month}월 ${day}일 ${period} ${displayHours}시 도착 예정`;
};

// 상세 날짜 포맷팅
const formatDetailDateTime = (dateTime) => {
  if (!dateTime) return '';

  try {
    const date = new Date(dateTime);
    const year = date.getFullYear();
    const month = String(date.getMonth() + 1).padStart(2, '0');
    const day = String(date.getDate()).padStart(2, '0');
    const hours = String(date.getHours()).padStart(2, '0');
    const minutes = String(date.getMinutes()).padStart(2, '0');

    return `${year}.${month}.${day} ${hours}:${minutes}`;
  } catch (error) {
    return '';
  }
};

// 운송장 번호 복사
const copyTrackingNumber = async () => {
  try {
    await navigator.clipboard.writeText(props.delivery.trackingNumber);
    alert('운송장 번호가 복사되었습니다!');
  } catch (err) {
    console.error('복사 실패:', err);
    alert('복사에 실패했습니다.');
  }
};

// 개인정보 마스킹 함수들
const maskName = (name) => {
  if (!name) return '정보 없음';
  if (name.length <= 2) return name;
  return name.charAt(0) + '*'.repeat(name.length - 2) + name.charAt(name.length - 1);
};

const maskPhoneNumber = (phone) => {
  if (!phone) return '정보 없음';
  return phone.replace(/(\d{3})-?(\d{4})-?(\d{4})/, '$1-****-$3');
};

const maskAddress = (address) => {
  if (!address) return '정보 없음';
  const parts = address.split(' ');
  if (parts.length >= 2) {
    return parts[0] + ' ' + parts[1] + ' ****';
  }
  return address;
};

// 배송지 변경 가능 여부
const canChangeAddress = computed(() => {
  return ['RECEIPT_COMPLETED', 'DELIVERY_BEGIN'].includes(props.delivery.status);
});

// 배송지 변경
const changeAddress = () => {
  emit('change-address', props.delivery);
};
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
  z-index: 1000;
  padding: 20px;
}

.modal-container {
  background-color: white;
  border-radius: 20px;
  width: 100%;
  max-width: 480px;
  max-height: 90vh;
  overflow-y: auto;
  position: relative;
  box-shadow: 0 20px 40px rgba(0, 0, 0, 0.1);
}

.modal-header {
  display: flex;
  justify-content: between;
  align-items: center;
  padding: 24px;
  border-bottom: 1px solid #f1f3f4;
  position: sticky;
  top: 0;
  background-color: white;
  border-radius: 20px 20px 0 0;
}

.modal-title {
  font-size: 20px;
  font-weight: 700;
  color: #2c3e50;
  margin: 0;
  flex: 1;
}

.close-button {
  background: none;
  border: none;
  font-size: 24px;
  color: #6c757d;
  cursor: pointer;
  padding: 0;
  width: 32px;
  height: 32px;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: 50%;
  transition: all 0.2s ease;
}

.close-button:hover {
  background-color: #f8f9fa;
  color: #2c3e50;
}

.modal-content {
  padding: 0 24px 24px;
}

/* 핵심 정보 섹션 */
.key-info-section {
  padding: 24px 0;
  border-bottom: 1px solid #f1f3f4;
}

.status-badge {
  display: inline-block;
  padding: 8px 16px;
  border-radius: 20px;
  font-size: 14px;
  font-weight: 600;
  margin-bottom: 20px;
}

.status-pending {
  background-color: #fff3cd;
  color: #856404;
}

.status-in-progress {
  background-color: #d1ecf1;
  color: #0c5460;
}

.status-completed {
  background-color: #d4edda;
  color: #155724;
}

.status-cancelled {
  background-color: #f8d7da;
  color: #721c24;
}

.delivery-estimate {
  margin-bottom: 20px;
}

.estimate-title {
  font-size: 14px;
  color: #6c757d;
  margin-bottom: 4px;
}

.estimate-time {
  font-size: 16px;
  font-weight: 600;
  color: #2c3e50;
}

.tracking-number {
  margin-bottom: 0;
}

.tracking-label {
  font-size: 14px;
  color: #6c757d;
  margin-bottom: 8px;
}

.tracking-value {
  display: flex;
  align-items: center;
  gap: 12px;
}

.tracking-text {
  font-family: 'Courier New', monospace;
  font-size: 16px;
  font-weight: 600;
  color: #2c3e50;
}

.copy-button {
  display: flex;
  align-items: center;
  gap: 4px;
  padding: 6px 12px;
  background-color: #f8f9fa;
  border: 1px solid #e9ecef;
  border-radius: 8px;
  font-size: 12px;
  color: #6c757d;
  cursor: pointer;
  transition: all 0.2s ease;
}

.copy-button:hover {
  background-color: #e9ecef;
  color: #495057;
}

/* 타임라인 섹션 */
.timeline-section {
  padding: 24px 0;
  border-bottom: 1px solid #f1f3f4;
}

.section-title {
  font-size: 18px;
  font-weight: 600;
  color: #2c3e50;
  margin: 0 0 20px 0;
}

.timeline {
  position: relative;
}

.timeline::before {
  content: '';
  position: absolute;
  left: 32px;
  top: 0;
  bottom: 0;
  width: 2px;
  background-color: #e9ecef;
}

.timeline-item {
  position: relative;
  padding-left: 75px;
  margin-bottom: 24px;
}

.timeline-item:last-child {
  margin-bottom: 0;
}

.timeline-dot {
  position: absolute;
  left: 20px;
  top: 4px;
  width: 24px;
  height: 24px;
  border-radius: 50%;
  background-color: #e9ecef;
  border: 4px solid white;
  box-shadow: 0 0 0 2px #e9ecef;
}

.timeline-completed .timeline-dot {
  background-color: #28a745;
  box-shadow: 0 0 0 2px #28a745;
}

.timeline-active .timeline-dot {
  background-color: #007bff;
  box-shadow: 0 0 0 2px #007bff;
  animation: pulse-timeline 1.5s infinite;
}

@keyframes pulse-timeline {
  0% {
    box-shadow: 0 0 0 2px #007bff;
  }
  50% {
    box-shadow: 0 0 0 8px rgba(0, 123, 255, 0.3);
  }
  100% {
    box-shadow: 0 0 0 2px #007bff;
  }
}

.timeline-content {
  padding: 4px 0;
}

.timeline-status {
  font-size: 16px;
  font-weight: 600;
  color: #2c3e50;
  margin-bottom: 4px;
}

.timeline-description {
  font-size: 14px;
  color: #6c757d;
  margin-bottom: 4px;
}

.timeline-time {
  font-size: 12px;
  color: #007bff;
  font-weight: 500;
  margin-bottom: 4px;
}

.timeline-location {
  font-size: 12px;
  color: #6c757d;
  font-style: italic;
}

/* 주문 정보 섹션 */
.order-info-section {
  padding: 24px 0;
  border-bottom: 1px solid #f1f3f4;
}

.order-details {
  margin-bottom: 20px;
}

.order-item {
  display: flex;
  align-items: center;
  gap: 16px;
  padding: 16px;
  background-color: #f8f9fa;
  border-radius: 12px;
}

.item-image {
  font-size: 32px;
}

.item-info {
  flex: 1;
}

.item-name {
  font-size: 16px;
  font-weight: 600;
  color: #2c3e50;
  margin-bottom: 4px;
}

.item-description {
  font-size: 14px;
  color: #6c757d;
}

.info-subtitle {
  font-size: 16px;
  font-weight: 600;
  color: #2c3e50;
  margin: 0 0 12px 0;
}

.recipient-details {
  background-color: #f8f9fa;
  border-radius: 12px;
  padding: 16px;
}

.recipient-item {
  display: flex;
  margin-bottom: 8px;
}

.recipient-item:last-child {
  margin-bottom: 0;
}

.label {
  font-size: 14px;
  color: #6c757d;
  width: 80px;
  flex-shrink: 0;
}

.value {
  font-size: 14px;
  color: #2c3e50;
  font-weight: 500;
}

/* 고객 지원 섹션 */
.support-section {
  padding: 24px 0 0;
}

.driver-contact {
  margin-bottom: 20px;
}

.contact-info {
  display: flex;
  align-items: center;
  gap: 16px;
  padding: 16px;
  background-color: #e3f2fd;
  border-radius: 12px;
}

.contact-icon {
  font-size: 24px;
}

.contact-title {
  font-size: 14px;
  color: #1976d2;
  font-weight: 600;
  margin-bottom: 4px;
}

.contact-number {
  font-size: 16px;
  color: #1976d2;
  font-weight: 700;
}

.support-buttons {
  display: flex;
  gap: 12px;
}

.support-btn {
  flex: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  padding: 14px;
  border: none;
  border-radius: 12px;
  font-size: 14px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s ease;
}

.inquiry-btn {
  background-color: #007bff;
  color: white;
}

.inquiry-btn:hover {
  background-color: #0056b3;
}

.address-btn {
  background-color: #28a745;
  color: white;
}

.address-btn:hover {
  background-color: #1e7e34;
}

.btn-icon {
  font-size: 16px;
}

/* 반응형 디자인 */
@media (max-width: 480px) {
  .modal-container {
    margin: 10px;
    max-height: calc(100vh - 20px);
  }

  .modal-header, .modal-content {
    padding: 16px;
  }

  .support-buttons {
    flex-direction: column;
  }

  .tracking-value {
    flex-direction: column;
    align-items: flex-start;
    gap: 8px;
  }
}
</style>