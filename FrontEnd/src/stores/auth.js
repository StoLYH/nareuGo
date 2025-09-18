import { defineStore } from 'pinia'
import { ref, computed } from 'vue'

export const useAuthStore = defineStore('auth', () => {
// State                                                                                                                                                                        
const user = ref(null)
const accessToken = ref(localStorage.getItem('access_token') || null)
const isLoading = ref(false)

// Getters                                                                                                                                                                      
const isAuthenticated = computed(() => {
    return !!accessToken.value && !!user.value
})

// Actions                                                                                                                                                                      
const setTokens = (access_token) => {
    accessToken.value = access_token
    localStorage.setItem('access_token', access_token)
}

const setUser = (userData) => {
    user.value = userData
}

const logout = async () => {
    try {
    // 백엔드 로그아웃 API 호출
    // await authAPI.logout()
    } catch (error) {
    console.error('로그아웃 에러:', error)
    } finally {
    // 클라이언트 측 정리
    clearAuth()
    }
}

const clearAuth = () => {
    user.value = null
    accessToken.value = null
    localStorage.removeItem('access_token')
    localStorage.removeItem('user')
}

const getUserInfo = async () => {
    if (!accessToken.value) return

    try {
        isLoading.value = true
        const userData = JSON.parse(localStorage.getItem('user') || '{}')
        if (userData && userData.userId) {
            setUser(userData)
        }
    } catch (error) {
        console.error('사용자 정보 조회 실패:', error)
        clearAuth()
    } finally {
        isLoading.value = false
    }
}


return {
    // State
    user,
    accessToken,
    isLoading,

    // Getters
    isAuthenticated,

    // Actions
    setTokens,
    setUser,
    logout,
    clearAuth,
    getUserInfo
}
})
