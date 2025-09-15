<template>                                                                                                                                                                        
    <div class="oauth-callback">                                                                                                                                                    
        <div v-if="isLoading" class="loading">
        <div class="spinner"></div>
        <p>로그인 처리 중...</p>
        </div>

        <div v-else-if="error" class="error">
        <h2>로그인 실패</h2>
        <p>{{ error }}</p>
        <button @click="goToLogin">다시 시도</button>
        </div>
    </div>
</template>

<script setup>                                                                                                                                                                    
import { ref, onMounted } from 'vue'
import { useRoute, useRouter } from 'vue-router'
import { useAuthStore } from '@/stores/auth'

const route = useRoute()
const router = useRouter()
const authStore = useAuthStore()

const isLoading = ref(true)
const error = ref('')

onMounted(async () => {
try {
    // URL에서 토큰 추출 (백엔드에서 리다이렉트 시 포함)
    const token = route.query.token || route.query.access_token
    const errorParam = route.query.error

    if (errorParam) {
    throw new Error(decodeURIComponent(errorParam))
    }

    if (!token) {
    throw new Error('인증 토큰이 없습니다.')
    }

    // 토큰 저장
    authStore.setTokens(token)

    // 사용자 정보 조회
    await authStore.getUserInfo()

    // 메인 페이지로 리다이렉트
    const redirectTo = route.query.redirect || '/'
    router.replace(redirectTo)

} catch (err) {
    console.error('OAuth2 콜백 처리 실패:', err)
    error.value = err.message || '로그인 처리 중 오류가 발생했습니다.'
} finally {
    isLoading.value = false
}
})

const goToLogin = () => {
router.push('/login')
}
</script>                                                                                                                                                                         

<style scoped>                                                                                                                                                                    
.oauth-callback {
display: flex;
justify-content: center;
align-items: center;
min-height: 100vh;
text-align: center;
}

.loading {
display: flex;
flex-direction: column;
align-items: center;
gap: 16px;
}

.spinner {
width: 32px;
height: 32px;
border: 3px solid #f3f3f3;
border-top: 3px solid #3498db;
border-radius: 50%;
animation: spin 1s linear infinite;
}

@keyframes spin {
0% { transform: rotate(0deg); }
100% { transform: rotate(360deg); }
}

.error {
color: #e74c3c;
}

.error button {
margin-top: 16px;
padding: 8px 16px;
background-color: #3498db;
color: white;
border: none;
border-radius: 4px;
cursor: pointer;
}
</style>