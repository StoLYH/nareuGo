<template>
  <div class="notification-page">
    <!-- 헤더 -->
    <header class="notification-header">
      <button class="back-btn" @click="goBack">
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
          <path d="M19 12H5M12 19l-7-7 7-7" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
        </svg>
      </button>
      <h1 class="header-title">알림</h1>
      <div class="header-spacer"></div>
    </header>

    <!-- 알림 목록 -->
    <main class="notification-list">
      <div v-if="notificationStore.notifications.length === 0" class="empty-state">
        <div class="empty-icon">
          <svg width="64" height="64" viewBox="0 0 24 24" fill="none">
            <path d="M12 22c1.1 0 2-.9 2-2h-4c0 1.1.89 2 2 2zm6-6v-5c0-3.07-1.64-5.64-4.5-6.32V4c0-.83-.67-1.5-1.5-1.5s-1.5.67-1.5 1.5v.68C7.63 5.36 6 7.92 6 11v5l-2 2v1h16v-1l-2-2z" fill="#ccc"/>
          </svg>
        </div>
        <p class="empty-message">새로운 알림이 없습니다</p>
        <p class="empty-submessage">알림이 오면 여기에 표시됩니다</p>
      </div>

      <div v-else>
        <div
          v-for="notification in notificationStore.notifications"
          :key="notification.id"
          class="notification-item"
          :class="{ 'unread': !notification.isRead }"
          @click="markAsRead(notification.id)"
        >
          <div class="notification-icon">
            <svg v-if="notification.type === 'LIKE'" width="20" height="20" viewBox="0 0 24 24" fill="none">
              <path d="M20.84 4.61a5.5 5.5 0 0 0-7.78 0L12 5.67l-1.06-1.06a5.5 5.5 0 0 0-7.78 7.78l1.06 1.06L12 21.23l7.78-7.78 1.06-1.06a5.5 5.5 0 0 0 0-7.78z" fill="#FF3B30"/>
            </svg>
            <svg v-else-if="notification.type === 'PURCHASE'" width="20" height="20" viewBox="0 0 24 24" fill="none">
              <path d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" stroke="#34C759" stroke-width="2"/>
            </svg>
            <svg v-else-if="notification.type === 'DELIVERY'" width="20" height="20" viewBox="0 0 24 24" fill="none">
              <path d="M8 6a2 2 0 012-2h4l2 2h6a2 2 0 012 2v6a2 2 0 01-2 2H6a2 2 0 01-2-2V6z" stroke="#FF9500" stroke-width="2" fill="none"/>
              <circle cx="12" cy="11" r="3" stroke="#FF9500" stroke-width="2" fill="none"/>
            </svg>
            <svg v-else-if="notification.type === 'DELIVERY_COMPLETED' || notification.type === 'DELIVERY_ARRIVAL'" width="20" height="20" viewBox="0 0 24 24" fill="none">
              <path d="M8 12l2 2 4-4M3 12a9 9 0 1118 0" stroke="#28a745" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
              <path d="M21 12a9 9 0 11-18 0" stroke="#28a745" stroke-width="2" fill="none"/>
              <path d="M16 8v8" stroke="#28a745" stroke-width="2" stroke-linecap="round"/>
            </svg>
            <svg v-else width="20" height="20" viewBox="0 0 24 24" fill="none">
              <path d="M12 22c1.1 0 2-.9 2-2h-4c0 1.1.89 2 2 2zm6-6v-5c0-3.07-1.64-5.64-4.5-6.32V4c0-.83-.67-1.5-1.5-1.5s-1.5.67-1.5 1.5v.68C7.63 5.36 6 7.92 6 11v5l-2 2v1h16v-1l-2-2z" fill="#007AFF"/>
            </svg>
          </div>
          
          <div class="notification-content">
            <h3 class="notification-title">{{ notification.title }}</h3>
            <p class="notification-message">{{ notification.message }}</p>
            <span class="notification-time">{{ formatTime(notification.createdAt) }}</span>
          </div>
          
          <div v-if="!notification.isRead" class="unread-indicator"></div>
        </div>
      </div>
    </main>

    <!-- 판매자 픽업 모달 -->
    <SellerPickupModal
      :is-visible="showPickupModal"
      :delivery-data="selectedDeliveryData"
      @close="closePickupModal"
      @pickup-confirmed="handlePickupConfirmed"
    />

    <!-- 구매자 픽업 모달 -->
    <BuyerPickupModal
      :is-visible="showBuyerPickupModal"
      :delivery-data="selectedBuyerDeliveryData"
      @close="closeBuyerPickupModal"
      @pickup-confirmed="handleBuyerPickupConfirmed"
    />
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted } from 'vue'
import { useRouter } from 'vue-router'
import { useNotificationStore } from '@/stores/notification'
import SellerPickupModal from '@/components/SellerPickupModal.vue'
import BuyerPickupModal from '@/components/BuyerPickupModal.vue'

const router = useRouter()
const notificationStore = useNotificationStore()

// 판매자 픽업 모달 관련
const showPickupModal = ref(false)
const selectedDeliveryData = ref(null)

// 구매자 픽업 모달 관련
const showBuyerPickupModal = ref(false)
const selectedBuyerDeliveryData = ref(null)

// 샘플 데이터 (실제로는 API에서 가져올 데이터)
const sampleNotifications = [
  {
    id: 1,
    type: 'ROBOT_ARRIVAL',
    title: '나르고가 도착했습니다!',
    message: '물건을 넣어주세요. 배송을 계속하려면 여기를 눌러주세요.',
    createdAt: new Date(Date.now() - 1000 * 60 * 5), // 5분 전
    isRead: false,
    deliveryId: 1,
    productTitle: '맛있는 사과',
    buyerName: '김구매자'
  },
  {
    id: 5,
    type: 'DELIVERY_ARRIVAL',
    title: '🏠 배송 도착!',
    message: '나르고가 도착했습니다! 물건을 수령해 주세요.',
    createdAt: new Date(Date.now() - 1000 * 60 * 2), // 2분 전
    isRead: false,
    deliveryId: 1,
    productTitle: '맛있는 사과',
    sellerName: '김판매자',
    timestamp: new Date().toISOString()
  },
  {
    id: 2,
    type: 'LIKE',
    title: '관심 상품 알림',
    message: '좋아요 표시한 "맛있는 사과" 상품의 가격이 할인되었습니다!',
    createdAt: new Date(Date.now() - 1000 * 60 * 30), // 30분 전
    isRead: false
  },
  {
    id: 3,
    type: 'PURCHASE',
    title: '구매 완료',
    message: '"신선한 바나나" 상품 구매가 완료되었습니다.',
    createdAt: new Date(Date.now() - 1000 * 60 * 60 * 2), // 2시간 전
    isRead: true
  },
  {
    id: 4,
    type: 'GENERAL',
    title: '새로운 상품 등록',
    message: '근처에 새로운 상품이 등록되었습니다. 확인해보세요!',
    createdAt: new Date(Date.now() - 1000 * 60 * 60 * 24), // 1일 전
    isRead: false
  }
]

const goBack = () => {
  router.go(-1)
}

const markAsRead = (notificationId) => {
  const notification = notificationStore.notifications.find(n => n.id === notificationId)

  if (notification && notification.type === 'ROBOT_ARRIVAL') {
    // 로봇 도착 알림인 경우 판매자 픽업 모달 열기
    selectedDeliveryData.value = {
      deliveryId: notification.deliveryId,
      productTitle: notification.productTitle,
      buyerName: notification.buyerName
    }
    showPickupModal.value = true
  } else if (notification && (notification.type === 'DELIVERY_COMPLETED' || notification.type === 'DELIVERY_ARRIVAL')) {
    // 구매자 배송 완료/도착 알림인 경우 구매자 픽업 모달 열기
    selectedBuyerDeliveryData.value = {
      deliveryId: notification.deliveryId,
      productTitle: notification.productTitle,
      sellerName: notification.sellerName,
      timestamp: notification.timestamp
    }
    showBuyerPickupModal.value = true
    console.log('🏠 [알림페이지] 구매자 배송 도착 모달 열기:', selectedBuyerDeliveryData.value)
  }

  notificationStore.markAsRead(notificationId)
  // 여기서 실제로는 API 호출로 읽음 상태를 서버에 업데이트
  console.log('Marked notification as read:', notificationId)
}

const formatTime = (date) => {
  const now = new Date()
  const diff = now - date
  const minutes = Math.floor(diff / (1000 * 60))
  const hours = Math.floor(diff / (1000 * 60 * 60))
  const days = Math.floor(diff / (1000 * 60 * 60 * 24))

  if (minutes < 1) {
    return '방금 전'
  } else if (minutes < 60) {
    return `${minutes}분 전`
  } else if (hours < 24) {
    return `${hours}시간 전`
  } else {
    return `${days}일 전`
  }
}

// 로봇 도착 알림 이벤트 핸들러 (알림 페이지용)
const handleRobotArrivalOnPage = (event) => {
  console.log('🔔 [알림페이지] 로봇 도착 이벤트 수신:', event.detail)

  // Store를 통해 알림 추가 (이미 AppHeader에서도 추가되지만 중복 방지)
  // 실시간으로 화면 업데이트됨

  // 페이지 상단으로 부드럽게 스크롤
  setTimeout(() => {
    window.scrollTo({ top: 0, behavior: 'smooth' })
  }, 100)

  // 새 알림을 하이라이트하는 효과
  setTimeout(() => {
    const newNotificationElement = document.querySelector('.notification-item.unread:first-child')
    if (newNotificationElement) {
      newNotificationElement.classList.add('new-notification')
      // 5초 후 하이라이트 제거
      setTimeout(() => {
        newNotificationElement.classList.remove('new-notification')
      }, 5000)
    }
  }, 200)

  console.log('🔄 [알림페이지] 새 알림으로 인한 화면 업데이트 완료')
}

// 구매자 배송 도착 알림 이벤트 핸들러
const handleBuyerDeliveryArrived = (event) => {
  console.log('🏠 [알림페이지] 구매자 배송 도착 이벤트 수신:', event.detail)

  // Store에 알림 추가
  const deliveryNotification = {
    type: 'DELIVERY_COMPLETED',
    title: '📦 배송완료 알림',
    message: '나르고가 도착했습니다! 물건을 수령해 주세요.',
    deliveryId: event.detail.deliveryId,
    productTitle: event.detail.productTitle,
    sellerName: event.detail.sellerName,
    timestamp: event.detail.timestamp
  }

  notificationStore.addNotification(deliveryNotification)
  console.log('🏠 [알림페이지] 구매자 배송 도착 알림 추가됨:', deliveryNotification)

  // 페이지 상단으로 부드럽게 스크롤
  setTimeout(() => {
    window.scrollTo({ top: 0, behavior: 'smooth' })
  }, 100)

  // 새 알림을 하이라이트하는 효과
  setTimeout(() => {
    const newNotificationElement = document.querySelector('.notification-item.unread:first-child')
    if (newNotificationElement) {
      newNotificationElement.classList.add('new-notification')
      // 5초 후 하이라이트 제거
      setTimeout(() => {
        newNotificationElement.classList.remove('new-notification')
      }, 5000)
    }
  }, 200)

  console.log('🔄 [알림페이지] 구매자 배송 도착으로 인한 화면 업데이트 완료')
}

// 모달 닫기 핸들러들
const closePickupModal = () => {
  showPickupModal.value = false
  selectedDeliveryData.value = null
}

const closeBuyerPickupModal = () => {
  showBuyerPickupModal.value = false
  selectedBuyerDeliveryData.value = null
}

const handlePickupConfirmed = (data) => {
  console.log('✅ 판매자 픽업 완료:', data)
  closePickupModal()
}

const handleBuyerPickupConfirmed = (data) => {
  console.log('✅ 구매자 수령 완료:', data)
  closeBuyerPickupModal()
}

onMounted(() => {
  // 실제로는 여기서 API를 호출해서 알림 목록을 가져옴
  // 샘플 데이터로 초기화 (store에 없을 때만)
  if (notificationStore.notifications.length === 0) {
    sampleNotifications.forEach(notification => {
      notificationStore.addNotification(notification)
    })
  }

  // 로봇 도착 이벤트 리스너 등록 (알림 페이지용)
  window.addEventListener('robotArrivedAtSeller', handleRobotArrivalOnPage)
  window.addEventListener('buyerDeliveryArrived', handleBuyerDeliveryArrived)
  console.log('🔔 [알림페이지] 로봇 도착 및 구매자 배송 도착 이벤트 리스너 등록됨')
})

onUnmounted(() => {
  // 이벤트 리스너 제거
  window.removeEventListener('robotArrivedAtSeller', handleRobotArrivalOnPage)
  window.removeEventListener('buyerDeliveryArrived', handleBuyerDeliveryArrived)
  console.log('🔔 [알림페이지] 로봇 도착 및 구매자 배송 도착 이벤트 리스너 제거됨')
})
</script>

<style scoped>
.notification-page {
  min-height: 100vh;
  background-color: #f8f9fa;
}

.notification-header {
  display: flex;
  align-items: center;
  padding: 16px 20px;
  background-color: white;
  border-bottom: 1px solid #eee;
  position: sticky;
  top: 0;
  z-index: 10;
}

.back-btn {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 40px;
  height: 40px;
  border-radius: 50%;
  background-color: #f8f8f8;
  color: #333;
  transition: background-color 0.2s;
}

.back-btn:hover {
  background-color: #f0f0f0;
}

.header-title {
  flex: 1;
  text-align: center;
  font-size: 18px;
  font-weight: 600;
  color: #333;
  margin: 0;
}

.header-spacer {
  width: 40px;
}

.notification-list {
  padding: 0;
}

.empty-state {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 80px 20px;
  text-align: center;
}

.empty-icon {
  margin-bottom: 16px;
  opacity: 0.5;
}

.empty-message {
  font-size: 18px;
  font-weight: 500;
  color: #666;
  margin-bottom: 8px;
}

.empty-submessage {
  font-size: 14px;
  color: #999;
}

.notification-item {
  display: flex;
  align-items: flex-start;
  padding: 16px 20px;
  background-color: white;
  border-bottom: 1px solid #eee;
  cursor: pointer;
  transition: background-color 0.2s;
  position: relative;
}

.notification-item:hover {
  background-color: #f8f9fa;
}

.notification-item.unread {
  background-color: #f0f8ff;
}

.notification-icon {
  margin-right: 12px;
  margin-top: 2px;
}

.notification-content {
  flex: 1;
}

.notification-title {
  font-size: 16px;
  font-weight: 600;
  color: #333;
  margin: 0 0 4px 0;
}

.notification-message {
  font-size: 14px;
  color: #666;
  line-height: 1.4;
  margin: 0 0 8px 0;
}

.notification-time {
  font-size: 12px;
  color: #999;
}

.unread-indicator {
  width: 8px;
  height: 8px;
  background-color: #007AFF;
  border-radius: 50%;
  margin-left: 8px;
  margin-top: 6px;
  flex-shrink: 0;
}

/* 새 알림 하이라이트 효과 */
.notification-item.new-notification {
  background-color: #e3f2fd !important;
  border-left: 4px solid #2196f3;
  animation: newNotificationGlow 3s ease-in-out;
  transform: scale(1.02);
  box-shadow: 0 4px 12px rgba(33, 150, 243, 0.3);
}

@keyframes newNotificationGlow {
  0% {
    background-color: #2196f3;
    transform: scale(1.05);
  }
  25% {
    background-color: #e3f2fd;
    transform: scale(1.02);
  }
  50% {
    background-color: #2196f3;
    transform: scale(1.03);
  }
  75% {
    background-color: #e3f2fd;
    transform: scale(1.02);
  }
  100% {
    background-color: #e3f2fd;
    transform: scale(1.02);
  }
}

/* 새 알림 아이템 슬라이드 인 효과 */
.notification-item:first-child.unread {
  animation: slideInFromTop 0.5s ease-out;
}

@keyframes slideInFromTop {
  0% {
    transform: translateY(-100%);
    opacity: 0;
  }
  100% {
    transform: translateY(0);
    opacity: 1;
  }
}
</style>
