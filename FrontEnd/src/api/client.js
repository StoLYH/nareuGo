import axios from 'axios'
import { useAuthStore } from '@/stores/auth'
import router from '@/router'

// API 기본 설정
const apiClient = axios.create({
  baseURL: import.meta.env.VITE_API_BASE_URL || 'http://localhost:8080',
  timeout: 10000,
  withCredentials: true
})


// 요청 인터셉터
apiClient.interceptors.request.use(
  (config) => {
    const authStore = useAuthStore()

    const isRefreshRequest = config?.url?.includes('/api/v1/auth/refresh')
    if (authStore.accessToken && !isRefreshRequest) {
      config.headers.Authorization = `Bearer ${authStore.accessToken}`
    }

    return config
  },
  (error) => {
    return Promise.reject(error)
  }
)

// 응답 인터셉터
apiClient.interceptors.response.use(
  (response) => {
    return response
  },
  async (error) => {
    const originalRequest = error.config

    if (error.response?.status === 401 && !originalRequest._retry) {
      originalRequest._retry = true

      const authStore = useAuthStore()
      
      // 리프레시 요청 자체가 401이면 즉시 로그아웃
      if (originalRequest?.url?.includes('/api/v1/auth/refresh')) {
        authStore.clearAuth()
        if (!originalRequest?.url?.includes('/login')) {
          router.push('/login')
        }
        return Promise.reject(error)
      }

      try {
        await authStore.refreshAccessToken()
        // 토큰 갱신 성공 시 원래 요청 재시도
        return apiClient(originalRequest)
      } catch (refreshError) {
        // 토큰 갱신 실패 시 로그인 페이지로 리다이렉트
        authStore.clearAuth()
        if (!originalRequest?.url?.includes('/login')) {
          router.push('/login')
        }
        return Promise.reject(refreshError)
      }
    }

    return Promise.reject(error)
  }
)

export default apiClient
