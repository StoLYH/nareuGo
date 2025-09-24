<template>
  <div class="fcm-quick-setup">
    <div class="setup-card">
      <h3>🔔 알림 설정</h3>
      <div class="status-info">
        <span class="label">알림 상태:</span>
        <span :class="['status', statusClass]">{{ statusText }}</span>
      </div>
      <button
        @click="setupFCM"
        :disabled="isLoading"
        :class="['setup-btn', statusClass]"
      >
        {{ isLoading ? '설정 중...' : buttonText }}
      </button>
      <div v-if="fcmToken" class="token-display">
        <small>토큰: {{ fcmToken.substring(0, 20) }}...</small>
      </div>
    </div>
  </div>
</template>

<script>
import fcmService from '@/api/fcmService.js'

export default {
  name: 'FCMQuickSetup',
  data() {
    return {
      isLoading: false,
      fcmToken: null,
      isSetup: false
    }
  },
  computed: {
    statusText() {
      if (this.isSetup) return '알림 활성화됨'
      return '알림 비활성화됨'
    },
    statusClass() {
      return this.isSetup ? 'active' : 'inactive'
    },
    buttonText() {
      return this.isSetup ? '알림 재설정' : '알림 활성화'
    }
  },
  async mounted() {
    await this.checkStatus()
  },
  methods: {
    async checkStatus() {
      const tokenInfo = fcmService.getTokenInfo()
      if (tokenInfo.token && tokenInfo.isInitialized) {
        this.fcmToken = tokenInfo.token
        this.isSetup = true
      }
    },

    async setupFCM() {
      this.isLoading = true
      try {
        // 테스트용 사용자 ID (실제로는 로그인된 사용자 ID 사용)
        const testUserId = 1

        const success = await fcmService.initialize(testUserId)

        if (success) {
          const tokenInfo = fcmService.getTokenInfo()
          this.fcmToken = tokenInfo.token
          this.isSetup = true

          // 성공 메시지
          this.showMessage('알림 설정이 완료되었습니다!', 'success')
        } else {
          this.showMessage('알림 설정에 실패했습니다. 권한을 허용해주세요.', 'error')
        }
      } catch (error) {
        console.error('FCM 설정 오류:', error)
        this.showMessage('설정 중 오류가 발생했습니다.', 'error')
      } finally {
        this.isLoading = false
      }
    },

    showMessage(text, type) {
      // 간단한 메시지 표시
      const message = document.createElement('div')
      message.textContent = text
      message.style.cssText = `
        position: fixed;
        top: 20px;
        right: 20px;
        padding: 12px 20px;
        border-radius: 4px;
        color: white;
        background: ${type === 'success' ? '#28a745' : '#dc3545'};
        z-index: 9999;
        font-size: 14px;
      `
      document.body.appendChild(message)

      setTimeout(() => {
        document.body.removeChild(message)
      }, 3000)
    }
  }
}
</script>

<style scoped>
.fcm-quick-setup {
  margin-bottom: 20px;
}

.setup-card {
  background: white;
  border: 1px solid #e9ecef;
  border-radius: 8px;
  padding: 20px;
  box-shadow: 0 2px 4px rgba(0,0,0,0.1);
}

.setup-card h3 {
  margin: 0 0 15px 0;
  color: #333;
}

.status-info {
  display: flex;
  align-items: center;
  margin-bottom: 15px;
}

.label {
  font-weight: 600;
  margin-right: 10px;
  color: #555;
}

.status {
  padding: 4px 8px;
  border-radius: 4px;
  font-size: 12px;
  font-weight: 500;
}

.status.active {
  background: #d4edda;
  color: #155724;
}

.status.inactive {
  background: #f8d7da;
  color: #721c24;
}

.setup-btn {
  padding: 10px 20px;
  border: none;
  border-radius: 5px;
  cursor: pointer;
  font-weight: 500;
  transition: all 0.2s;
  width: 100%;
}

.setup-btn:disabled {
  opacity: 0.6;
  cursor: not-allowed;
}

.setup-btn.active {
  background: #28a745;
  color: white;
}

.setup-btn.active:hover:not(:disabled) {
  background: #218838;
}

.setup-btn.inactive {
  background: #007bff;
  color: white;
}

.setup-btn.inactive:hover:not(:disabled) {
  background: #0056b3;
}

.token-display {
  margin-top: 10px;
  font-family: monospace;
  color: #666;
}
</style>