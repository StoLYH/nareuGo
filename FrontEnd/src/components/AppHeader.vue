<template>
  <header class="header">
    <div class="location">
      <span class="location-text">{{ location }}</span>
    </div>
    <div class="header-icons">
      <button class="icon-btn" @click="$emit('edit')">
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
          <path d="M3 17.25V21h3.75L17.81 9.94l-3.75-3.75L3 17.25zM20.71 7.04c.39-.39.39-1.02 0-1.41l-2.34-2.34c-.39-.39-1.02-.39-1.41 0l-1.83 1.83 3.75 3.75 1.83-1.83z" fill="currentColor"/>
        </svg>
      </button>
      <button class="icon-btn" @click="$emit('search')">
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
          <path d="M15.5 14h-.79l-.28-.27C15.41 12.59 16 11.11 16 9.5 16 5.91 13.09 3 9.5 3S3 5.91 3 9.5 5.91 16 9.5 16c1.61 0 3.09-.59 4.23-1.57l.27.28v.79l5 4.99L20.49 19l-4.99-5zm-6 0C7.01 14 5 11.99 5 9.5S7.01 5 9.5 5 14 7.01 14 9.5 11.99 14 9.5 14z" fill="currentColor"/>
        </svg>
      </button>
      <button class="icon-btn notification-btn" @click="handleNotificationClick">
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
          <path d="M12 22c1.1 0 2-.9 2-2h-4c0 1.1.89 2 2 2zm6-6v-5c0-3.07-1.64-5.64-4.5-6.32V4c0-.83-.67-1.5-1.5-1.5s-1.5.67-1.5 1.5v.68C7.63 5.36 6 7.92 6 11v5l-2 2v1h16v-1l-2-2z" fill="currentColor"/>
        </svg>
        <div v-if="notificationStore.unreadCount > 0" class="notification-badge">{{ notificationStore.unreadCount }}</div>
      </button>
    </div>
  </header>

  <!-- 실시간 알림 팝업 -->
  <div v-if="showNotificationAlert" class="notification-alert" @click="handleRobotArrivedClick">
    <div class="alert-content">
      <div class="alert-icon">
        🤖
      </div>
      <div class="alert-text">
        <div class="alert-title">나르고가 도착했습니다!</div>
        <div class="alert-message">{{ lastNotificationMessage }}</div>
      </div>
      <button class="alert-close" @click.stop="dismissAlert">
        ×
      </button>
    </div>
  </div>

  <!-- 물건 넣기 모달창 -->
  <SellerPickupModal
    :isVisible="showPickupModal"
    :deliveryData="pickupData"
    @close="closePickupModal"
    @pickup-confirmed="handlePickupConfirmed"
  />
</template>

<script setup>
import { ref, onMounted, onUnmounted } from 'vue'
import { useNotificationStore } from '@/stores/notification'
import { useAuthStore } from '@/stores/auth'
import SellerPickupModal from './SellerPickupModal.vue'

defineProps({
  location: {
    type: String,
    default: 'OO마을 12단지'
  }
})
defineEmits(['edit', 'search', 'notification'])

const notificationStore = useNotificationStore()
const authStore = useAuthStore()
const showNotificationAlert = ref(false)
const lastNotificationMessage = ref('')
const showPickupModal = ref(false)
const pickupData = ref(null)
const lastRobotArrivedData = ref(null)

// 알림 팝업 닫기
const dismissAlert = () => {
  showNotificationAlert.value = false
}

// 헤더 알림 버튼 클릭 처리
const handleNotificationClick = () => {
  console.log('🔔 [헤더] 알림 버튼 클릭됨')
  console.log('🔔 [헤더] 알림 목록:', notificationStore.notifications)
  console.log('🔔 [헤더] lastRobotArrivedData:', lastRobotArrivedData.value)

  // 가장 최근 로봇 도착 관련 알림이 있으면 모달창 열기
  const latestRobotArrivalNotification = notificationStore.notifications.find(n => n.type === 'ROBOT_ARRIVAL' && !n.isRead)
  const latestBuyerArrivalNotification = notificationStore.notifications.find(n => n.type === 'DELIVERY_ARRIVAL' && !n.isRead)

  console.log('🔔 [헤더] 최근 판매자 로봇 도착 알림:', latestRobotArrivalNotification)
  console.log('🔔 [헤더] 최근 구매자 배송 도착 알림:', latestBuyerArrivalNotification)

  if (latestRobotArrivalNotification) {
    // 판매자용 알림에서 배송 데이터 추출
    const modalData = {
      deliveryId: latestRobotArrivalNotification.deliveryId || '1',
      productTitle: latestRobotArrivalNotification.productTitle || '상품명 없음',
      buyerName: latestRobotArrivalNotification.buyerName || '구매자명 없음'
    }
    console.log('🔔 [헤더] 판매자 모달창 열기 데이터:', modalData)
    openPickupModal(modalData)
    notificationStore.markAsRead(latestRobotArrivalNotification.id)
  } else if (latestBuyerArrivalNotification) {
    // 구매자용 알림인 경우 구매자 모달 열기 이벤트 발송
    const modalData = {
      deliveryId: latestBuyerArrivalNotification.deliveryId || '1',
      productTitle: latestBuyerArrivalNotification.productTitle || '상품명 없음',
      sellerName: latestBuyerArrivalNotification.sellerName || '판매자명 없음'
    }
    console.log('🔔 [헤더] 구매자 모달창 열기 데이터:', modalData)

    // 구매자 모달 열기 이벤트 발송
    const event = new CustomEvent('showBuyerPickupModal', {
      detail: modalData
    })
    window.dispatchEvent(event)
    notificationStore.markAsRead(latestBuyerArrivalNotification.id)
  } else {
    console.log('🔔 [헤더] 읽지 않은 로봇 도착/배송 도착 알림을 찾을 수 없음')
    // 기존 배송 관련 알림도 확인
    const latestDeliveryNotification = notificationStore.notifications.find(n => n.type === 'DELIVERY')
    if (latestDeliveryNotification) {
      const modalData = lastRobotArrivedData.value || {
        deliveryId: '1',
        productTitle: '고래팝니다',
        buyerName: '오세원'
      }
      console.log('🔔 [헤더] 기존 배송 알림으로 모달창 열기:', modalData)
      openPickupModal(modalData)
      notificationStore.markAsRead(latestDeliveryNotification.id)
    }
  }
}

// 실시간 알림 팝업 클릭 처리
const handleRobotArrivedClick = () => {
  console.log('🔔 [4️⃣ 단계] "나르고 도착!" 팝업 클릭됨')
  console.log('🔔 [4️⃣ 단계] lastRobotArrivedData:', lastRobotArrivedData.value)

  dismissAlert()
  console.log('🔔 [4️⃣ 단계] 팝업 닫기 완료')

  if (lastRobotArrivedData.value) {
    console.log('🔔 [4️⃣ 단계] ✅ lastRobotArrivedData 있음, 모달창 열기')
    openPickupModal(lastRobotArrivedData.value)
  } else {
    console.log('🔔 [4️⃣ 단계] ❌ lastRobotArrivedData 없음, 임시 데이터로 모달창 열기')
    // 임시 데이터로 모달창 열기
    const tempData = {
      deliveryId: '1',
      productTitle: '고래팝니다',
      buyerName: '오세원'
    }
    console.log('🔔 [4️⃣ 단계] 임시 데이터:', tempData)
    openPickupModal(tempData)
  }
}

// 물건 넣기 모달창 열기
const openPickupModal = (data) => {
  console.log('🔔 [4️⃣ 단계] openPickupModal 호출됨')
  console.log('🔔 [4️⃣ 단계] 전달받은 데이터:', data)

  pickupData.value = {
    deliveryId: data.deliveryId,
    productTitle: data.productTitle,
    buyerName: data.buyerName
  }
  console.log('🔔 [4️⃣ 단계] pickupData 설정됨:', pickupData.value)

  showPickupModal.value = true
  console.log('🔔 [4️⃣ 단계] ✅ showPickupModal = true 설정됨')
  console.log('🔔 [4️⃣ 단계] 모달창이 표시되어야 함')
}

// 물건 넣기 모달창 닫기
const closePickupModal = () => {
  showPickupModal.value = false
  pickupData.value = null
}

// 물건 넣기 완료 처리
const handlePickupConfirmed = (result) => {
  console.log('🔔 [헤더] 물건 넣기 완료:', result)
  // 필요시 추가 처리
}

// Store의 알림 변화를 감지해서 팝업 표시
const handleNotificationUpdate = () => {
  console.log('🔔 [3️⃣ 단계] 알림 업데이트 감지됨')
  console.log('🔔 [3️⃣ 단계] 전체 알림 목록:', notificationStore.notifications)

  const latestNotification = notificationStore.notifications[0]
  console.log('🔔 [3️⃣ 단계] 최신 알림:', latestNotification)

  if (latestNotification) {
    console.log('🔔 [3️⃣ 단계] 알림 타입:', latestNotification.type)
    console.log('🔔 [3️⃣ 단계] 읽음 상태:', latestNotification.isRead)
  }

  if (latestNotification && !latestNotification.isRead &&
      (latestNotification.type === 'DELIVERY' ||
       latestNotification.type === 'ROBOT_ARRIVAL' ||
       latestNotification.type === 'DELIVERY_ARRIVAL')) {
    console.log('🔔 [3️⃣ 단계] ✅ 로봇 도착/배송 도착 알림 팝업 표시 조건 만족')

    // 알림 타입에 따른 메시지 설정
    let message = '나르고가 도착했습니다!'
    if (latestNotification.type === 'ROBOT_ARRIVAL') {
      message = '나르고가 도착했습니다! 물건을 넣어주세요.'
    } else if (latestNotification.type === 'DELIVERY_ARRIVAL') {
      message = '배송이 도착했습니다! 물건을 수령해주세요.'
    }

    lastNotificationMessage.value = latestNotification.message || message
    showNotificationAlert.value = true
    console.log('🔔 [3️⃣ 단계] ✅ showNotificationAlert = true 설정됨')

    // 5초 후 자동으로 팝업 닫기
    setTimeout(() => {
      console.log('🔔 [3️⃣ 단계] 5초 후 팝업 자동 닫기')
      showNotificationAlert.value = false
    }, 5000)
  } else {
    console.log('🔔 [3️⃣ 단계] ❌ 로봇 도착/배송 도착 알림 팝업 표시 조건 불만족')
  }
}

// 로봇 도착 이벤트 처리
const handleRobotArrivedEvent = (event) => {
  console.log('🔔 [헤더] 로봇 도착 이벤트 수신:', event.detail)
  lastRobotArrivedData.value = event.detail
}

// showPickupModal 이벤트 처리 (FCM 알림 클릭 시)
const handleShowPickupModalEvent = (event) => {
  console.log('🔔 [헤더] showPickupModal 이벤트 수신:', event.detail)
  openPickupModal(event.detail)
}


// 컴포넌트 마운트 시 초기화
onMounted(async () => {
  // 백엔드에서 읽지 않은 알림 개수 조회
  const userId = authStore.user?.userId || 3
  await notificationStore.fetchUnreadCount(userId)

  // Store 변화 감지 (notifications 배열 변화 감지)
  const unwatch = notificationStore.$onAction(({name, after}) => {
    if (name === 'addNotification') {
      after((result) => {
        console.log('🔔 [헤더] 새 알림이 추가됨:', result)
        handleNotificationUpdate()
      })
    }
  })

  // 로봇 도착 이벤트 리스너 등록
  window.addEventListener('robotArrivedAtSeller', handleRobotArrivedEvent)

  // FCM 알림 클릭으로 모달창 열기 이벤트 리스너 등록
  window.addEventListener('showPickupModal', handleShowPickupModalEvent)

  // 언마운트 시 watcher 및 이벤트 리스너 제거
  onUnmounted(() => {
    unwatch?.()
    window.removeEventListener('robotArrivedAtSeller', handleRobotArrivedEvent)
    window.removeEventListener('showPickupModal', handleShowPickupModalEvent)
  })
})
</script>

<style scoped>
.header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 16px 20px;
  background-color: #4682B4;
  border-bottom: none;
  box-shadow: 0 4px 12px rgba(0,0,0,0.12);
  position: relative;
}

.location {
  font-size: 18px;
  font-weight: 700;
  color: #ffffff;
}

.header-icons {
  display: flex;
  gap: 8px;
}

.icon-btn {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 40px;
  height: 40px;
  border-radius: 50%;
  background-color: rgba(255,255,255,0.16);
  color: #ffffff;
  transition: background-color 0.2s, color 0.2s, transform 0.2s;
  position: relative;
}

.icon-btn:hover {
  background-color: rgba(255,255,255,0.28);
  color: #ffffff;
  transform: translateY(-1px);
}

.notification-btn {
  position: relative;
}

.notification-badge {
  position: absolute;
  top: -2px;
  right: -2px;
  background-color: #ff4757;
  color: white;
  border-radius: 50%;
  width: 18px;
  height: 18px;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 11px;
  font-weight: bold;
  border: 2px solid white;
  animation: pulse 2s infinite;
}

@keyframes pulse {
  0% {
    transform: scale(1);
  }
  50% {
    transform: scale(1.1);
  }
  100% {
    transform: scale(1);
  }
}

.header::after {
  content: '';
  position: absolute;
  left: 0;
  right: 0;
  bottom: -8px;
  height: 12px;
  background: linear-gradient(180deg, rgba(70,130,180,0.25), rgba(110,198,202,0));
  pointer-events: none;
}

/* 실시간 알림 팝업 스타일 */
.notification-alert {
  position: fixed;
  top: 80px;
  right: 20px;
  z-index: 1000;
  background: white;
  border-radius: 12px;
  box-shadow: 0 8px 32px rgba(0,0,0,0.2);
  max-width: 320px;
  animation: slideInRight 0.3s ease-out;
  cursor: pointer;
}

@keyframes slideInRight {
  from {
    transform: translateX(100%);
    opacity: 0;
  }
  to {
    transform: translateX(0);
    opacity: 1;
  }
}

.alert-content {
  display: flex;
  align-items: center;
  padding: 16px;
  gap: 12px;
}

.alert-icon {
  font-size: 24px;
  flex-shrink: 0;
}

.alert-text {
  flex: 1;
}

.alert-title {
  font-weight: bold;
  color: #2c3e50;
  margin-bottom: 4px;
}

.alert-message {
  color: #7f8c8d;
  font-size: 14px;
  line-height: 1.4;
}

.alert-close {
  background: none;
  border: none;
  font-size: 20px;
  color: #bdc3c7;
  cursor: pointer;
  padding: 0;
  width: 24px;
  height: 24px;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: 50%;
  transition: background-color 0.2s;
}

.alert-close:hover {
  background-color: #ecf0f1;
}

</style>