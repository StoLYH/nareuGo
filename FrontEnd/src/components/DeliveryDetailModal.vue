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
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M8 4V16C8 17.1046 8.89543 18 10 18H18C19.1046 18 20 17.1046 20 16V7.24264C20 6.44699 19.6839 5.68393 19.1213 5.12132L16.8787 2.87868C16.3161 2.31607 15.553 2 14.7574 2H10C8.89543 2 8 2.89543 8 4Z" stroke="currentColor" stroke-width="2"/>
                  <path d="M16 18V20C16 21.1046 15.1046 22 14 22H6C4.89543 22 4 21.1046 4 20V9C4 7.89543 4.89543 7 6 7H8" stroke="currentColor" stroke-width="2"/>
                </svg>
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
            <div class="contact-card">
              <div class="contact-header">
                <div class="driver-info">
                  <div class="driver-avatar">
                    <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                      <path d="M12 12C14.7614 12 17 9.76142 17 7C17 4.23858 14.7614 2 12 2C9.23858 2 7 4.23858 7 7C7 9.76142 9.23858 12 12 12Z" fill="currentColor"/>
                      <path d="M12 14C7.58172 14 4 17.5817 4 22H20C20 17.5817 16.4183 14 12 14Z" fill="currentColor"/>
                    </svg>
                  </div>
                  <div class="driver-details">
                    <div class="driver-name">나르고봇 #R001</div>
                    <div class="driver-status">
                      <div class="status-dot"></div>
                      배송 중
                    </div>
                  </div>
                </div>
                <div class="contact-actions">
                  <button class="contact-btn call-btn" @click="callDriver">
                    <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                      <path d="M22 16.92V19.92C22.0011 20.4897 21.8064 21.0449 21.4447 21.4894C21.083 21.9339 20.5771 22.2398 20.0116 22.3567C19.4462 22.4736 18.8562 22.3942 18.3393 22.131C17.8225 21.8679 17.4106 21.4364 17.17 20.9C16.1 19.07 14.97 17.35 13.72 15.72C13.4154 15.3394 13.2644 14.8672 13.2916 14.3879C13.3188 13.9087 13.5226 13.4555 13.87 13.11L15.17 11.81C15.3945 11.5756 15.5673 11.2952 15.6775 10.9876C15.7877 10.68 15.8329 10.3521 15.81 10.025C15.7871 9.69787 15.6967 9.37975 15.5451 9.09082C15.3936 8.8019 15.1844 8.54981 14.93 8.35L12.58 6.58C12.2149 6.30885 11.7846 6.16228 11.3419 6.15651C10.8993 6.15074 10.4654 6.28599 10.094 6.546C9.72262 6.80601 9.43043 7.18026 9.25659 7.61502C9.08275 8.04978 9.03541 8.52745 9.12 8.99L9.86 12.73C9.91 13 10 13.26 10.12 13.5L12 17C12.4652 17.9435 13.1945 18.7327 14.1001 19.2671C15.0057 19.8015 16.048 20.0573 17.1 19.9992C17.3667 19.9854 17.6333 19.9854 17.9 19.9992C18.6 20.0292 19.31 19.9592 19.9892 19.7892C20.6684 19.6192 21.3168 19.3492 21.9 18.9892C22.483 18.6292 22.9863 18.1559 23.3788 17.5892C23.7713 17.0225 24.043 16.3738 24.1763 15.6892C24.3096 15.0046 24.3015 14.2996 24.1525 13.6192C24.0035 12.9388 23.717 12.2976 23.31 11.7392C22.903 11.1808 22.3843 10.719 21.7892 10.3792C21.1941 10.0394 20.5348 9.82916 19.86 9.76L18.37 9.64C18.1226 9.61878 17.8726 9.61878 17.625 9.64C17.5 9.65 17.38 9.67 17.26 9.7C17.14 9.73 17.03 9.77 16.92 9.82L15.82 10.32C15.5467 10.4467 15.32 10.64 15.16 10.88C15 11.12 14.91 11.4 14.9 11.69C14.89 11.98 14.96 12.27 15.1 12.52C15.24 12.77 15.45 12.97 15.7 13.1L16.8 13.65C17.05 13.78 17.35 13.82 17.63 13.76C17.91 13.7 18.16 13.55 18.34 13.33L18.84 12.75C19.02 12.53 19.27 12.38 19.55 12.32C19.83 12.26 20.12 12.3 20.37 12.43L21.47 12.98C21.72 13.11 21.92 13.32 22.04 13.58C22.16 13.84 22.19 14.13 22.13 14.41L21.88 15.51C21.82 15.79 21.67 16.04 21.45 16.22L20.87 16.72C20.65 16.9 20.37 17 20.08 17C19.79 17 19.51 16.9 19.29 16.72L18.71 16.22" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                    </svg>
                  </button>
                  <button class="contact-btn message-btn" @click="messageDriver">
                    <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                      <path d="M21 15C21 15.5304 20.7893 16.0391 20.4142 16.4142C20.0391 16.7893 19.5304 17 19 17H7L3 21V5C3 4.46957 3.21071 3.96086 3.58579 3.58579C3.96086 3.21071 4.46957 3 5 3H19C19.5304 3 20.0391 3.21071 20.4142 3.58579C20.7893 3.96086 21 4.46957 21 5V15Z" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                    </svg>
                  </button>
                </div>
              </div>
              <div class="location-info">
                <div class="location-text">
                  <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                    <path d="M21 10C21 17 12 23 12 23S3 17 3 10C3 7.61305 3.94821 5.32387 5.63604 3.63604C7.32387 1.94821 9.61305 1 12 1C14.3869 1 16.6761 1.94821 18.3639 3.63604C20.0518 5.32387 21 7.61305 21 10Z" stroke="currentColor" stroke-width="2"/>
                    <path d="M12 13C13.6569 13 15 11.6569 15 10C15 8.34315 13.6569 7 12 7C10.3431 7 9 8.34315 9 10C9 11.6569 10.3431 13 12 13Z" stroke="currentColor" stroke-width="2"/>
                  </svg>
                  {{ delivery.destination || '고객님 배송지로 이동 중입니다' }}
                </div>
                <div class="eta-info">예상 도착: 약 {{ getEstimatedMinutes() }}분 후</div>
              </div>
            </div>
          </div>

          <div class="support-buttons">
            <button class="support-btn inquiry-btn" @click="handleInquiry">
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M21 15C21 15.5304 20.7893 16.0391 20.4142 16.4142C20.0391 16.7893 19.5304 17 19 17H7L3 21V5C3 4.46957 3.21071 3.96086 3.58579 3.58579C3.96086 3.21071 4.46957 3 5 3H19C19.5304 3 20.0391 3.21071 20.4142 3.58579C20.7893 3.96086 21 4.46957 21 5V15Z" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
              </svg>
              고객센터 문의
            </button>

            <button
              v-if="canChangeAddress"
              class="support-btn address-btn"
              @click="changeAddress"
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M21 10C21 17 12 23 12 23S3 17 3 10C3 7.61305 3.94821 5.32387 5.63604 3.63604C7.32387 1.94821 9.61305 1 12 1C14.3869 1 16.6761 1.94821 18.3639 3.63604C20.0518 5.32387 21 7.61305 21 10Z" stroke="currentColor" stroke-width="2"/>
                <path d="M12 13C13.6569 13 15 11.6569 15 10C15 8.34315 13.6569 7 12 7C10.3431 7 9 8.34315 9 10C9 11.6569 10.3431 13 12 13Z" stroke="currentColor" stroke-width="2"/>
              </svg>
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

// 배송 상태 텍스트 매핑 (3단계로 수정)
const statusTexts = {
  'RECEIPT_COMPLETED': '접수완료',
  'IN_DELIVERY': '배달중',
  'DELIVERY_COMPLETED': '배송완료',
  'CANCELLED': '취소됨'
};

// 배송 상태별 순서 (3단계로 수정)
const statusOrder = {
  'RECEIPT_COMPLETED': 0,
  'IN_DELIVERY': 1,
  'DELIVERY_COMPLETED': 2,
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

// 상태 뱃지 클래스 (3단계로 수정)
const getStatusBadgeClass = (status) => {
  const classes = {
    'RECEIPT_COMPLETED': 'status-pending',
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
    await navigator.clipboard.writeText(props.delivery.trackingNumber || 'NRG-2025-001');
    // 실제 서비스에서는 토스트 알림을 사용
    const originalText = document.querySelector('.copy-button').textContent;
    document.querySelector('.copy-button').innerHTML = `
      <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M20 6L9 17l-5-5" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
      </svg>
      복사됨!
    `;
    setTimeout(() => {
      document.querySelector('.copy-button').innerHTML = `
        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M8 4V16C8 17.1046 8.89543 18 10 18H18C19.1046 18 20 17.1046 20 16V7.24264C20 6.44699 19.6839 5.68393 19.1213 5.12132L16.8787 2.87868C16.3161 2.31607 15.553 2 14.7574 2H10C8.89543 2 8 2.89543 8 4Z" stroke="currentColor" stroke-width="2"/>
          <path d="M16 18V20C16 21.1046 15.1046 22 14 22H6C4.89543 22 4 21.1046 4 20V9C4 7.89543 4.89543 7 6 7H8" stroke="currentColor" stroke-width="2"/>
        </svg>
        복사
      `;
    }, 2000);
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

// 배송지 변경 가능 여부 (3단계로 수정)
const canChangeAddress = computed(() => {
  return ['RECEIPT_COMPLETED'].includes(props.delivery.status);
});

// 배송지 변경
const changeAddress = () => {
  emit('change-address', props.delivery);
};

// 예상 도착 시간 (분 단위)
const getEstimatedMinutes = () => {
  if (props.delivery.status === 'DELIVERY_COMPLETED') {
    return 0;
  }
  // 실제로는 GPS 정보를 기반으로 계산
  return Math.floor(Math.random() * 30) + 15; // 15-45분 사이
};

// 배송기사 전화걸기
const callDriver = () => {
  // 실제 서비스에서는 VoIP 연결 또는 전화번호 제공
  alert('나르고봇은 자율주행 로봇으로 직접 통화는 불가하지만, 실시간 위치를 확인할 수 있습니다.');
};

// 배송기사 메시지
const messageDriver = () => {
  // 실제 서비스에서는 채팅 인터페이스 열기
  alert('나르고봇과의 실시간 채팅을 지원 예정입니다.');
};

// 문의하기
const handleInquiry = () => {
  emit('inquiry', props.delivery);
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
  justify-content: space-between;
  align-items: center;
  padding: 24px;
  border-bottom: 1px solid #f1f3f4;
  position: sticky;
  top: 0;
  background-color: white;
  border-radius: 20px 20px 0 0;
  z-index: 10;
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
  gap: 6px;
  padding: 8px 16px;
  background-color: #007bff;
  border: none;
  border-radius: 8px;
  font-size: 13px;
  font-weight: 500;
  color: white;
  cursor: pointer;
  transition: all 0.2s ease;
}

.copy-button:hover {
  background-color: #0056b3;
  transform: translateY(-1px);
  box-shadow: 0 2px 8px rgba(0, 123, 255, 0.3);
}

.copy-button:active {
  transform: translateY(0);
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
  margin-bottom: 24px;
}

.contact-card {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  border-radius: 16px;
  padding: 20px;
  color: white;
  box-shadow: 0 8px 32px rgba(102, 126, 234, 0.3);
}

.contact-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 16px;
}

.driver-info {
  display: flex;
  align-items: center;
  gap: 12px;
}

.driver-avatar {
  width: 48px;
  height: 48px;
  background: rgba(255, 255, 255, 0.2);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  color: white;
}

.driver-details {
  flex: 1;
}

.driver-name {
  font-size: 16px;
  font-weight: 700;
  margin-bottom: 4px;
}

.driver-status {
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 14px;
  opacity: 0.9;
}

.status-dot {
  width: 8px;
  height: 8px;
  background-color: #4ade80;
  border-radius: 50%;
  animation: pulse-dot 2s infinite;
}

@keyframes pulse-dot {
  0%, 100% {
    opacity: 1;
    transform: scale(1);
  }
  50% {
    opacity: 0.7;
    transform: scale(1.2);
  }
}

.contact-actions {
  display: flex;
  gap: 8px;
}

.contact-btn {
  width: 44px;
  height: 44px;
  background: rgba(255, 255, 255, 0.2);
  border: 1px solid rgba(255, 255, 255, 0.3);
  border-radius: 12px;
  display: flex;
  align-items: center;
  justify-content: center;
  color: white;
  cursor: pointer;
  transition: all 0.2s ease;
  backdrop-filter: blur(10px);
}

.contact-btn:hover {
  background: rgba(255, 255, 255, 0.3);
  transform: translateY(-2px);
}

.location-info {
  padding-top: 16px;
  border-top: 1px solid rgba(255, 255, 255, 0.2);
}

.location-text {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 14px;
  margin-bottom: 8px;
  opacity: 0.9;
}

.eta-info {
  font-size: 13px;
  font-weight: 600;
  color: #4ade80;
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
  padding: 16px;
  border: none;
  border-radius: 12px;
  font-size: 14px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s ease;
  position: relative;
  overflow: hidden;
}

.support-btn::before {
  content: '';
  position: absolute;
  top: 0;
  left: -100%;
  width: 100%;
  height: 100%;
  background: linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.2), transparent);
  transition: left 0.5s;
}

.support-btn:hover::before {
  left: 100%;
}

.inquiry-btn {
  background: linear-gradient(135deg, #007bff 0%, #0056b3 100%);
  color: white;
  box-shadow: 0 4px 15px rgba(0, 123, 255, 0.3);
}

.inquiry-btn:hover {
  transform: translateY(-2px);
  box-shadow: 0 6px 20px rgba(0, 123, 255, 0.4);
}

.address-btn {
  background: linear-gradient(135deg, #28a745 0%, #1e7e34 100%);
  color: white;
  box-shadow: 0 4px 15px rgba(40, 167, 69, 0.3);
}

.address-btn:hover {
  transform: translateY(-2px);
  box-shadow: 0 6px 20px rgba(40, 167, 69, 0.4);
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