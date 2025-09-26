import { defineStore } from 'pinia'
import { ref, computed } from 'vue'

export const useNotificationStore = defineStore('notification', () => {
  // 상태
  const notifications = ref([])
  const unreadCount = ref(0)
  const isEventListenerRegistered = ref(false)

  // 계산된 값
  const hasUnreadNotifications = computed(() => unreadCount.value > 0)

  // 액션
  const addNotification = (notification) => {
    const newNotification = {
      id: Date.now(),
      ...notification,
      isRead: false,
      createdAt: new Date()
    }

    notifications.value.unshift(newNotification)
    unreadCount.value += 1

    console.log('🔔 [Store] 새 알림 추가:', newNotification)
    return newNotification
  }

  const markAsRead = (notificationId) => {
    const notification = notifications.value.find(n => n.id === notificationId)
    if (notification && !notification.isRead) {
      notification.isRead = true
      unreadCount.value = Math.max(0, unreadCount.value - 1)
      console.log('🔔 [Store] 알림 읽음 처리:', notificationId)
    }
  }

  const setUnreadCount = (count) => {
    unreadCount.value = count
  }

  const fetchUnreadCount = async (userId) => {
    try {
      const baseURL = window.location.hostname === 'localhost' ? 'http://localhost:8080' : 'https://j13a501.p.ssafy.io/api'
      const response = await fetch(`${baseURL}/notifications/unread-count?userId=${userId}`)
      if (response.ok) {
        const contentType = response.headers.get('content-type')
        if (contentType && contentType.includes('application/json')) {
          const data = await response.json()
          if (data.success) {
            unreadCount.value = data.unreadCount
            return data.unreadCount
          }
        } else {
          console.warn('응답이 JSON이 아닙니다:', contentType)
        }
      }
    } catch (error) {
      console.error('읽지 않은 알림 개수 조회 실패:', error)
    }
    return 0
  }

  // 로봇 도착 알림 처리 (전역에서 한 번만)
  const handleRobotArrival = (event) => {
    console.log('🔔 [Store] 로봇 도착 이벤트 수신:', event.detail)

    const robotNotification = {
      type: 'DELIVERY',
      title: '나르고 도착 알림',
      message: event.detail.message
    }

    return addNotification(robotNotification)
  }

  // 전역 이벤트 리스너 등록
  const initEventListeners = () => {
    if (!isEventListenerRegistered.value) {
      window.addEventListener('robotArrivedAtSeller', handleRobotArrival)
      isEventListenerRegistered.value = true
      console.log('🔔 [Store] 전역 로봇 도착 이벤트 리스너 등록됨')
    }
  }

  // 전역 이벤트 리스너 제거
  const removeEventListeners = () => {
    if (isEventListenerRegistered.value) {
      window.removeEventListener('robotArrivedAtSeller', handleRobotArrival)
      isEventListenerRegistered.value = false
      console.log('🔔 [Store] 전역 로봇 도착 이벤트 리스너 제거됨')
    }
  }

  return {
    // 상태
    notifications,
    unreadCount,
    isEventListenerRegistered,

    // 계산된 값
    hasUnreadNotifications,

    // 액션
    addNotification,
    markAsRead,
    setUnreadCount,
    fetchUnreadCount,
    handleRobotArrival,
    initEventListeners,
    removeEventListeners
  }
})