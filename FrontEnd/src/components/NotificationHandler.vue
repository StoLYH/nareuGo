<template>
  <div v-if="showNotification" class="notification-toast" :class="notificationType">
    <div class="notification-content">
      <h4>{{ notificationTitle }}</h4>
      <p>{{ notificationBody }}</p>
    </div>
    <button @click="closeNotification" class="close-btn">×</button>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted } from 'vue'

const showNotification = ref(false)
const notificationTitle = ref('')
const notificationBody = ref('')
const notificationType = ref('default')

let notificationTimeout = null

const showInAppNotification = (event) => {
  const { title, body, type } = event.detail

  notificationTitle.value = title || '새 알림'
  notificationBody.value = body || ''
  notificationType.value = type || 'default'
  showNotification.value = true

  // 5초 후 자동으로 닫기
  if (notificationTimeout) {
    clearTimeout(notificationTimeout)
  }
  notificationTimeout = setTimeout(() => {
    closeNotification()
  }, 5000)
}

const closeNotification = () => {
  showNotification.value = false
  if (notificationTimeout) {
    clearTimeout(notificationTimeout)
    notificationTimeout = null
  }
}

onMounted(() => {
  window.addEventListener('show-notification', showInAppNotification)
})

onUnmounted(() => {
  window.removeEventListener('show-notification', showInAppNotification)
  if (notificationTimeout) {
    clearTimeout(notificationTimeout)
  }
})
</script>

<style scoped>
.notification-toast {
  position: fixed;
  top: 20px;
  right: 20px;
  background: white;
  border-radius: 8px;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  padding: 16px;
  max-width: 300px;
  z-index: 1000;
  display: flex;
  align-items: flex-start;
  gap: 12px;
  border-left: 4px solid #007AFF;
  animation: slideIn 0.3s ease-out;
}

.notification-toast.LIKE {
  border-left-color: #FF3B30;
}

.notification-toast.PURCHASE {
  border-left-color: #34C759;
}

.notification-toast.SALE_CONFIRMATION {
  border-left-color: #FF9500;
}

.notification-content {
  flex: 1;
}

.notification-content h4 {
  margin: 0 0 4px 0;
  font-size: 14px;
  font-weight: 600;
  color: #1D1D1F;
}

.notification-content p {
  margin: 0;
  font-size: 12px;
  color: #86868B;
  line-height: 1.4;
}

.close-btn {
  background: none;
  border: none;
  font-size: 18px;
  color: #86868B;
  cursor: pointer;
  padding: 0;
  width: 20px;
  height: 20px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.close-btn:hover {
  color: #1D1D1F;
}

@keyframes slideIn {
  from {
    transform: translateX(100%);
    opacity: 0;
  }
  to {
    transform: translateX(0);
    opacity: 1;
  }
}
</style>