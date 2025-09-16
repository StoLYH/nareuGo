import { defineStore } from 'pinia'                                                                                                                                               
import { ref, computed } from 'vue'                                                                                                                                               
import Cookies from 'js-cookie'                                                                                                                                                   
import { authAPI } from '@/api/auth'                                                                                                                                              

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
    console.log('저장되는 사용자 데이터:', userData)
    console.log('사용자 ID:', userData?.id)
    user.value = userData
}

const logout = async () => {
    try {
    // 백엔드 로그아웃 API 호출                                                                                                                                                 
    await authAPI.logout()
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
    Cookies.remove('refresh_token')
    // NareuGo 쿠키도 제거 (백엔드에서 설정한 쿠키)
    Cookies.remove('NareuGo')
}

const getUserInfo = async () => {
    if (!accessToken.value) return                                                                                                                                                

    try {
    isLoading.value = true                                                                                                                                                      
    const response = await authAPI.getUserInfo()
    setUser(response.data)
    } catch (error) {
    console.error('사용자 정보 조회 실패:', error)
    clearAuth()
    } finally {
    isLoading.value = false                                                                                                                                                     
    }
}

const refreshAccessToken = async () => {
    try {
    const response = await authAPI.refreshToken()
    if (!response?.data?.accessToken) {
        throw new Error('No accessToken in refresh response')
    }
    setTokens(response.data.accessToken)
    return response.data.accessToken
    } catch (error) {
    console.error('토큰 갱신 실패:', error)
    clearAuth()
    throw error
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
    getUserInfo,
    refreshAccessToken
}
})