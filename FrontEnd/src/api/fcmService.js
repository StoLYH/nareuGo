// FCM 토큰 관리 서비스
import axios from 'axios'
import { getFCMToken, onMessageListener } from '@/config/firebase.js'
import { handleFCMMessage } from '@/utils/fcmMessageHandler.js'

class FCMService {
  constructor() {
    this.apiBaseUrl = import.meta.env.VITE_API_BASE_URL || 'http://localhost:8080'
    this.userId = null
    this.fcmToken = null
    this.isInitialized = false
  }

  // FCM 서비스 초기화
  async initialize(userId) {
    try {
      this.userId = userId

      // Service Worker 등록
      await this.registerServiceWorker()

      // FCM 토큰 생성 및 등록
      await this.generateAndRegisterToken()

      // 메시지 리스너 설정
      this.setupMessageListener()

      this.isInitialized = true
      console.log('FCM 서비스 초기화 완료')

      return true
    } catch (error) {
      console.error('FCM 서비스 초기화 실패:', error)
      return false
    }
  }

  // Service Worker 등록
  async registerServiceWorker() {
    if (!('serviceWorker' in navigator)) {
      throw new Error('Service Worker를 지원하지 않는 브라우저입니다.')
    }

    try {
      const registration = await navigator.serviceWorker.register('/firebase-messaging-sw.js')
      console.log('Service Worker 등록 성공:', registration)
      return registration
    } catch (error) {
      console.error('Service Worker 등록 실패:', error)
      throw error
    }
  }

  // FCM 토큰 생성 및 백엔드 등록
  async generateAndRegisterToken() {
    try {
      // FCM 토큰 생성
      const token = await getFCMToken()

      if (!token) {
        console.warn('FCM 토큰 생성 실패 (HTTP 환경 또는 권한 거부)')
        return null
      }

      this.fcmToken = token
      console.log('FCM 토큰 생성 성공:', token)

      // 백엔드에 토큰 등록
      await this.registerTokenToBackend(token)

      // 로컬 스토리지에 토큰 저장
      localStorage.setItem('fcmToken', token)
      localStorage.setItem('fcmTokenUserId', this.userId)

      return token
    } catch (error) {
      console.error('FCM 토큰 생성/등록 실패:', error)
      return null
    }
  }

  // 백엔드에 FCM 토큰 등록
  async registerTokenToBackend(token) {
    try {
      const response = await axios.post(`${this.apiBaseUrl}/fcm/token`, {
        userId: this.userId,
        token: token,
        deviceType: 'WEB'
      })

      console.log('백엔드 토큰 등록 성공:', response.data)
      return true
    } catch (error) {
      console.error('백엔드 토큰 등록 실패:', error)
      throw error
    }
  }

  // 백엔드에서 FCM 토큰 제거
  async removeTokenFromBackend(token = this.fcmToken) {
    try {
      await axios.delete(`${this.apiBaseUrl}/fcm/token`, {
        data: {
          userId: this.userId,
          token: token
        }
      })

      console.log('백엔드 토큰 제거 성공')
      return true
    } catch (error) {
      console.error('백엔드 토큰 제거 실패:', error)
      return false
    }
  }

  // 포그라운드 메시지 리스너 설정
  setupMessageListener() {
    onMessageListener().then((payload) => {
      if (payload) {
        console.log('🔔 [FCM Service] 포그라운드 메시지 수신:', payload)

        // 새로운 통합 핸들러로 전달
        handleFCMMessage(payload)

        // 기존 처리도 유지 (호환성)
        this.handleForegroundMessage(payload)
      }
    }).catch((error) => {
      console.error('❌ [FCM Service] 메시지 리스너 설정 실패:', error)
    })
  }

  // 포그라운드 메시지 처리
  handleForegroundMessage(payload) {
    const title = payload.notification?.title || 'NareuGO 알림'
    const body = payload.notification?.body || '새로운 알림이 있습니다.'
    const icon = payload.notification?.icon || '/icons/icon-192x192.png'

    // 브라우저 알림 표시 (포그라운드 상태)
    if (Notification.permission === 'granted') {
      const notification = new Notification(title, {
        body: body,
        icon: icon,
        tag: payload.data?.type || 'default',
        data: payload.data || {}
      })

      // 알림 클릭 이벤트
      notification.onclick = () => {
        window.focus()
        notification.close()
        this.handleNotificationClick(payload.data)
      }

      // 3초 후 자동 닫기
      setTimeout(() => {
        notification.close()
      }, 3000)
    }

    // 앱 내 알림 처리 (UI 업데이트 등)
    this.handleInAppNotification(payload)
  }

  // 앱 내 알림 처리
  handleInAppNotification(payload) {
    // 여기서 앱 내 알림 UI 업데이트
    // 예: 알림 배지 업데이트, 알림 목록 추가 등
    const event = new CustomEvent('fcmNotificationReceived', {
      detail: payload
    })
    window.dispatchEvent(event)

    // "물건을 넣어주세요" 알림인 경우 특별 처리
    if (payload.data?.type === 'DELIVERY_ROBOT_ARRIVED') {
      const robotArrivedEvent = new CustomEvent('robotArrivedAtSeller', {
        detail: {
          message: payload.notification?.body || '나르고가 도착했습니다! 물건을 넣어주세요.',
          deliveryId: payload.data?.deliveryId,
          productTitle: payload.data?.productTitle,
          buyerName: payload.data?.buyerName,
          data: payload.data
        }
      })
      window.dispatchEvent(robotArrivedEvent)
    }

    // "물건을 가져가세요" 알림인 경우 특별 처리
    if (payload.data?.type === 'DELIVERY_ROBOT_ARRIVED_BUYER') {
      const robotArrivedBuyerEvent = new CustomEvent('robotArrivedAtBuyer', {
        detail: {
          message: payload.notification?.body || '나르고가 도착했습니다! 물건을 가져가세요.',
          deliveryId: payload.data?.deliveryId,
          productTitle: payload.data?.productTitle,
          sellerName: payload.data?.sellerName,
          data: payload.data
        }
      })
      window.dispatchEvent(robotArrivedBuyerEvent)
    }
  }

  // 알림 클릭 처리
  handleNotificationClick(data) {
    if (data?.type) {
      switch (data.type) {
        case 'DELIVERY_ROBOT_ARRIVED':
          // 물건 넣기 모달창 열기 이벤트 발생
          const pickupEvent = new CustomEvent('showPickupModal', {
            detail: {
              deliveryId: data.deliveryId,
              productTitle: data.productTitle,
              buyerName: data.buyerName,
              data: data
            }
          })
          window.dispatchEvent(pickupEvent)
          break
        case 'DELIVERY_ROBOT_ARRIVED_BUYER':
          // 물건 가져가기 모달창 열기 이벤트 발생
          const buyerPickupEvent = new CustomEvent('showBuyerPickupModal', {
            detail: {
              deliveryId: data.deliveryId,
              productTitle: data.productTitle,
              sellerName: data.sellerName,
              data: data
            }
          })
          window.dispatchEvent(buyerPickupEvent)
          break
        case 'LIKE':
          // 상품 상세 페이지로 이동
          if (data.productId) {
            window.location.href = `/product/${data.productId}`
          }
          break
        case 'PURCHASE':
          // 판매 관리 페이지로 이동
          window.location.href = '/my/sales'
          break
        case 'SALE_CONFIRMATION':
          // 구매 내역 페이지로 이동
          window.location.href = '/my/purchases'
          break
        case 'CHAT':
          // 채팅 페이지로 이동
          window.location.href = '/chat'
          break
        default:
          // 기본 홈페이지로 이동
          window.location.href = '/'
      }
    }
  }

  // FCM 토큰 새로고침
  async refreshToken() {
    try {
      await this.generateAndRegisterToken()
      return this.fcmToken
    } catch (error) {
      console.error('FCM 토큰 새로고침 실패:', error)
      return null
    }
  }

  // 서비스 정리
  async cleanup() {
    try {
      if (this.fcmToken) {
        await this.removeTokenFromBackend()
      }

      localStorage.removeItem('fcmToken')
      localStorage.removeItem('fcmTokenUserId')

      this.fcmToken = null
      this.userId = null
      this.isInitialized = false

      console.log('FCM 서비스 정리 완료')
    } catch (error) {
      console.error('FCM 서비스 정리 실패:', error)
    }
  }

  // 현재 토큰 상태 확인
  getTokenInfo() {
    return {
      token: this.fcmToken,
      userId: this.userId,
      isInitialized: this.isInitialized,
      storedToken: localStorage.getItem('fcmToken'),
      storedUserId: localStorage.getItem('fcmTokenUserId')
    }
  }
}

// 싱글톤 인스턴스
const fcmService = new FCMService()

export default fcmService