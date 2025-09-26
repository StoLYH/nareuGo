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
      <button class="icon-btn notification-btn" @click="$emit('notification')">
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
          <path d="M12 22c1.1 0 2-.9 2-2h-4c0 1.1.89 2 2 2zm6-6v-5c0-3.07-1.64-5.64-4.5-6.32V4c0-.83-.67-1.5-1.5-1.5s-1.5.67-1.5 1.5v.68C7.63 5.36 6 7.92 6 11v5l-2 2v1h16v-1l-2-2z" fill="currentColor"/>
        </svg>
        <div v-if="notificationCount > 0" class="notification-badge">{{ notificationCount }}</div>
      </button>
    </div>
  </header>

  <!-- 실시간 알림 팝업 -->
  <div v-if="showNotificationAlert" class="notification-alert" @click="dismissAlert">
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
</template>

<script setup>
import { ref, onMounted, onUnmounted } from 'vue'

defineProps({
  location: {
    type: String,
    default: 'OO마을 12단지'
  }
})
defineEmits(['edit', 'search', 'notification'])

const notificationCount = ref(0)
const showNotificationAlert = ref(false)
const lastNotificationMessage = ref('')

// 알림 이벤트 리스너
const handleRobotArrival = (event) => {
  console.log('🔔 [헤더] 로봇 도착 알림 수신:', event.detail)

  // 알림 카운트 증가
  notificationCount.value += 1

  // 알림 메시지 설정
  lastNotificationMessage.value = event.detail.message

  // 알림 팝업 표시
  showNotificationAlert.value = true

  // 5초 후 자동으로 팝업 닫기
  setTimeout(() => {
    showNotificationAlert.value = false
  }, 5000)
}

// 알림 팝업 닫기
const dismissAlert = () => {
  showNotificationAlert.value = false
}

// 컴포넌트 마운트 시 이벤트 리스너 등록
onMounted(() => {
  window.addEventListener('robotArrivedAtSeller', handleRobotArrival)

  // 백엔드에서 읽지 않은 알림 개수 조회
  fetchUnreadNotificationCount()
})

// 컴포넌트 언마운트 시 이벤트 리스너 제거
onUnmounted(() => {
  window.removeEventListener('robotArrivedAtSeller', handleRobotArrival)
})

// 읽지 않은 알림 개수 조회
const fetchUnreadNotificationCount = async () => {
  try {
    const baseURL = window.location.hostname === 'localhost' ? 'http://localhost:8080' : 'https://j13a501.p.ssafy.io/api'
    const response = await fetch(`${baseURL}/notifications/unread-count?userId=3`)
    if (response.ok) {
      const contentType = response.headers.get('content-type')
      if (contentType && contentType.includes('application/json')) {
        const data = await response.json()
        if (data.success) {
          notificationCount.value = data.unreadCount
        }
      } else {
        console.warn('응답이 JSON이 아닙니다:', contentType)
      }
    }
  } catch (error) {
    console.error('알림 개수 조회 실패:', error)
  }
}
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