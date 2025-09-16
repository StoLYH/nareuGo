<template>                                                                                                                                                                        
    <div class="error-page">                                                                                                                                                        
      <div class="error-container">
        <div class="error-icon">⚠️</div>
        <h2>로그인 실패</h2>
        <p class="error-message">{{ errorMessage }}</p>
        <div class="error-actions">
          <button @click="goToLogin" class="retry-button">
            다시 시도
          </button>
          <button @click="goToHome" class="home-button">
            홈으로 가기
          </button>
        </div>
      </div>
    </div>
</template>

<script setup>                                                                                                                                                                    
import { ref, onMounted } from 'vue'
import { useRoute, useRouter } from 'vue-router'

const route = useRoute()
const router = useRouter()

const errorMessage = ref('알 수 없는 오류가 발생했습니다.')

onMounted(() => {
const message = route.query.message
if (message) {
    errorMessage.value = decodeURIComponent(message)
}
})

const goToLogin = () => {
router.push('/login')
}

const goToHome = () => {
router.push('/')
}
</script>                                                                                                                                                                         

<style scoped>                                                                                                                                                                    
.error-page {
display: flex;
justify-content: center;
align-items: center;
min-height: 100vh;
background-color: #f7fafc;
}

.error-container {
background: white;
padding: 48px;
border-radius: 16px;
box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
text-align: center;
max-width: 500px;
}

.error-icon {
font-size: 64px;
margin-bottom: 24px;
}

.error-message {
color: #e53e3e;
margin-bottom: 32px;
font-size: 16px;
}

.error-actions {
display: flex;
gap: 16px;
justify-content: center;
}

.retry-button {
background-color: #3182ce;
color: white;
border: none;
padding: 12px 24px;
border-radius: 8px;
cursor: pointer;
font-size: 16px;
}

.home-button {
background-color: #718096;
color: white;
border: none;
padding: 12px 24px;
border-radius: 8px;
cursor: pointer;
font-size: 16px;
}
</style>