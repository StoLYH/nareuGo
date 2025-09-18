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
      <div v-if="notifications.length === 0" class="empty-state">
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
          v-for="notification in notifications" 
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
  </div>
</template>

<script setup>
import { ref, onMounted } from 'vue'
import { useRouter } from 'vue-router'

const router = useRouter()
const notifications = ref([])

// 샘플 데이터 (실제로는 API에서 가져올 데이터)
const sampleNotifications = [
  {
    id: 1,
    type: 'LIKE',
    title: '관심 상품 알림',
    message: '좋아요 표시한 "맛있는 사과" 상품의 가격이 할인되었습니다!',
    createdAt: new Date(Date.now() - 1000 * 60 * 30), // 30분 전
    isRead: false
  },
  {
    id: 2,
    type: 'PURCHASE',
    title: '구매 완료',
    message: '"신선한 바나나" 상품 구매가 완료되었습니다.',
    createdAt: new Date(Date.now() - 1000 * 60 * 60 * 2), // 2시간 전
    isRead: true
  },
  {
    id: 3,
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
  const notification = notifications.value.find(n => n.id === notificationId)
  if (notification) {
    notification.isRead = true
    // 여기서 실제로는 API 호출로 읽음 상태를 서버에 업데이트
    console.log('Marked notification as read:', notificationId)
  }
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

onMounted(() => {
  // 실제로는 여기서 API를 호출해서 알림 목록을 가져옴
  // loadNotifications()
  notifications.value = sampleNotifications
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
</style>
