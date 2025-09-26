// FCM 메시지 처리 핸들러

import { useNotificationStore } from '@/stores/notification'
import { showSellerRobotArrivedModal } from './sellerNotification.js'
import { showBuyerDeliveryModal } from './buyerNotification.js'

// FCM 메시지 핸들러 초기화
export const initFCMMessageHandler = () => {
  console.log('🔔 [FCM Handler] FCM 메시지 핸들러 초기화')

  // Service Worker에서 백그라운드 메시지 처리
  if ('serviceWorker' in navigator) {
    navigator.serviceWorker.addEventListener('message', (event) => {
      console.log('🔔 [FCM Handler] Service Worker 메시지 수신:', event.data)

      if (event.data && event.data.type === 'FCM_MESSAGE') {
        console.log('🔔 [FCM Handler] Service Worker에서 FCM 메시지 처리 시작')
        handleFCMMessage(event.data.payload)
      } else if (event.data && (event.data.notification || event.data.data)) {
        // 직접 FCM 데이터가 온 경우
        console.log('🔔 [FCM Handler] Service Worker에서 직접 FCM 데이터 처리 시작')
        handleFCMMessage(event.data)
      }
    })
  }

  // 포그라운드 메시지 처리 (Firebase SDK 사용 시)
  // 실제 환경에서는 Firebase의 onMessage()를 사용
  console.log('🔔 [FCM Handler] 포그라운드 메시지 리스너 준비 완료')
}

// FCM 메시지 처리 함수
export const handleFCMMessage = (payload) => {
  console.log('🔔 [FCM Handler] FCM 메시지 처리:', payload)

  const notificationStore = useNotificationStore()

  try {
    const { notification, data } = payload

    // 판매자에게 로봇 도착 알림 처리
    if (data && (data.type === 'ROBOT_ARRIVAL_SELLER' || data.type === 'DELIVERY_ROBOT_ARRIVED')) {
      console.log('🤖 [FCM Handler] 판매자 로봇 도착 FCM 메시지 처리')

      // 알림 스토어에 추가
      notificationStore.addNotification({
        type: 'ROBOT_ARRIVAL',
        title: '나르고가 도착했습니다!',
        message: '물건을 넣어주세요. 배송을 계속하려면 여기를 눌러주세요.',
        deliveryId: data.deliveryId,
        productTitle: data.productTitle || '상품명 없음',
        buyerName: data.buyerName || '구매자명 없음'
      })

      // 판매자 모달 자동 표시
      showSellerRobotArrivedModal({
        deliveryId: data.deliveryId,
        productTitle: data.productTitle,
        buyerName: data.buyerName,
        timestamp: new Date().toISOString()
      })

      console.log('✅ [FCM Handler] 판매자 로봇 도착 FCM 메시지 처리 완료')
    }

    // 구매자에게 로봇 도착 알림 처리
    else if (data && (data.type === 'ROBOT_ARRIVAL_BUYER' || data.type === 'DELIVERY_ROBOT_ARRIVED_BUYER')) {
      console.log('🏠 [FCM Handler] 구매자 로봇 도착 FCM 메시지 처리')

      // 알림 스토어에 추가
      notificationStore.addNotification({
        type: 'DELIVERY_ARRIVAL',
        title: '배송이 도착했습니다!',
        message: '나르고가 도착했습니다. 물건을 수령해주세요.',
        deliveryId: data.deliveryId,
        productTitle: data.productTitle || '상품명 없음',
        sellerName: data.sellerName || '판매자명 없음'
      })

      // 구매자 모달 자동 표시
      showBuyerDeliveryModal({
        deliveryId: data.deliveryId,
        productTitle: data.productTitle,
        sellerName: data.sellerName,
        buyerName: data.buyerName,
        timestamp: new Date().toISOString(),
        sellerAddress: data.sellerAddress,
        buyerAddress: data.buyerAddress
      })

      console.log('✅ [FCM Handler] 구매자 로봇 도착 FCM 메시지 처리 완료')
    }

    // 일반 로봇 도착 알림 (호환성 유지)
    else if (data && data.type === 'ROBOT_ARRIVAL') {
      console.log('🤖 [FCM Handler] 일반 로봇 도착 FCM 메시지 처리 (판매자로 처리)')

      // 판매자 모달 자동 표시 (기본값)
      showSellerRobotArrivedModal({
        deliveryId: data.deliveryId,
        productTitle: data.productTitle,
        buyerName: data.buyerName,
        timestamp: new Date().toISOString()
      })

      // 로봇 도착 이벤트 발송 (추가 처리용)
      const robotArrivedEvent = new CustomEvent('robotArrivedAtSeller', {
        detail: {
          deliveryId: data.deliveryId,
          productTitle: data.productTitle,
          buyerName: data.buyerName,
          timestamp: new Date().toISOString(),
          message: '나르고가 도착했습니다! 물건을 넣어주세요.'
        }
      })
      window.dispatchEvent(robotArrivedEvent)

      // 브라우저 알림 표시 (포그라운드 상태일 때)
      if (document.hasFocus() && 'Notification' in window && Notification.permission === 'granted') {
        const browserNotification = new Notification('🤖 나르고 도착!', {
          body: '나르고가 도착했습니다! 물건을 넣어주세요.',
          icon: '/icons/icon-192x192.png',
          tag: 'robot-arrival',
          requireInteraction: true
        })

        // 알림 클릭 시 모달 열기
        browserNotification.onclick = () => {
          const event = new CustomEvent('showPickupModal', {
            detail: {
              deliveryId: data.deliveryId,
              productTitle: data.productTitle,
              buyerName: data.buyerName
            }
          })
          window.dispatchEvent(event)
          browserNotification.close()
        }

        // 5초 후 자동 닫기
        setTimeout(() => {
          browserNotification.close()
        }, 5000)
      }

      console.log('✅ [FCM Handler] 로봇 도착 FCM 메시지 처리 완료')
    }

    // 배송 상태 업데이트 알림 처리
    else if (data && data.type === 'DELIVERY_STATUS') {
      console.log('📦 [FCM Handler] 배송 상태 FCM 메시지 처리')

      let title = '📦 배송 상태 업데이트'
      let message = notification?.body || '배송 상태가 업데이트되었습니다.'

      switch (data.status) {
        case 'STARTED':
          title = '🚀 배송 시작!'
          message = '나르고가 배송을 시작했습니다.'
          break
        case 'PICKUP_COMPLETED':
          title = '📦 픽업 완료!'
          message = '물건이 픽업되었습니다. 구매자에게 배송 중입니다.'
          break
        case 'ARRIVED_AT_BUYER':
          title = '🏠 배송 도착!'
          message = '나르고가 구매자 집에 도착했습니다.'
          break
        case 'COMPLETED':
          title = '✅ 배송 완료!'
          message = '배송이 성공적으로 완료되었습니다.'
          break
      }

      // 알림 스토어에 추가
      notificationStore.addNotification({
        type: 'DELIVERY_STATUS',
        title: title,
        message: message,
        deliveryId: data.deliveryId,
        status: data.status
      })

      console.log('✅ [FCM Handler] 배송 상태 FCM 메시지 처리 완료')
    }

    // 일반 알림 처리
    else {
      console.log('📢 [FCM Handler] 일반 FCM 메시지 처리')

      notificationStore.addNotification({
        type: 'GENERAL',
        title: notification?.title || '알림',
        message: notification?.body || '새로운 알림이 있습니다.'
      })

      console.log('✅ [FCM Handler] 일반 FCM 메시지 처리 완료')
    }

  } catch (error) {
    console.error('❌ [FCM Handler] FCM 메시지 처리 실패:', error)
  }
}

// 테스트용 함수들
if (typeof window !== 'undefined') {
  // 판매자 로봇 도착 FCM 메시지 시뮬레이션
  window.testSellerRobotArrivalFCM = (deliveryId = '1', productTitle = '테스트 상품', buyerName = '테스트 구매자') => {
    console.log('🧪 [TEST] 판매자 로봇 도착 FCM 메시지 시뮬레이션')

    const testPayload = {
      notification: {
        title: '나르고 도착!',
        body: '나르고가 도착했습니다! 물건을 넣어주세요.'
      },
      data: {
        type: 'ROBOT_ARRIVAL_SELLER',
        deliveryId: deliveryId.toString(),
        productTitle: productTitle,
        buyerName: buyerName
      }
    }

    handleFCMMessage(testPayload)
    return testPayload
  }

  // 구매자 로봇 도착 FCM 메시지 시뮬레이션
  window.testBuyerRobotArrivalFCM = (deliveryId = '1', productTitle = '테스트 상품', sellerName = '테스트 판매자', buyerName = '테스트 구매자') => {
    console.log('🧪 [TEST] 구매자 로봇 도착 FCM 메시지 시뮬레이션')

    const testPayload = {
      notification: {
        title: '배송 도착!',
        body: '나르고가 도착했습니다! 물건을 수령해주세요.'
      },
      data: {
        type: 'ROBOT_ARRIVAL_BUYER',
        deliveryId: deliveryId.toString(),
        productTitle: productTitle,
        sellerName: sellerName,
        buyerName: buyerName,
        sellerAddress: '1동 301호',
        buyerAddress: '2동 201호'
      }
    }

    handleFCMMessage(testPayload)
    return testPayload
  }

  // 기존 함수 (호환성 유지)
  window.testRobotArrivalFCM = window.testSellerRobotArrivalFCM

  // 배송 상태 FCM 메시지 시뮬레이션
  window.testDeliveryStatusFCM = (status = 'PICKUP_COMPLETED', deliveryId = '1') => {
    console.log('🧪 [TEST] 배송 상태 FCM 메시지 시뮬레이션')

    const testPayload = {
      notification: {
        title: '배송 상태 업데이트',
        body: '배송 상태가 업데이트되었습니다.'
      },
      data: {
        type: 'DELIVERY_STATUS',
        deliveryId: deliveryId.toString(),
        status: status
      }
    }

    handleFCMMessage(testPayload)
    return testPayload
  }

  console.log('🌐 [FCM Handler] 전역 테스트 함수들이 등록되었습니다:')
  console.log('- window.testSellerRobotArrivalFCM(deliveryId, productTitle, buyerName)')
  console.log('- window.testBuyerRobotArrivalFCM(deliveryId, productTitle, sellerName, buyerName)')
  console.log('- window.testRobotArrivalFCM(deliveryId, productTitle, buyerName) [기존 함수]')
  console.log('- window.testDeliveryStatusFCM(status, deliveryId)')
}

export default {
  initFCMMessageHandler,
  handleFCMMessage
}