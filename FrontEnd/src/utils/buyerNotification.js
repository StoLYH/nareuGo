// 구매자 배송 도착 알림 전역 유틸리티

let buyerModalInstance = null
let buyerModalCallback = null

// 구매자 모달 콜백 등록 (컴포넌트에서 호출)
export const registerBuyerModalCallback = (callback) => {
  buyerModalCallback = callback
  console.log('🔔 [BuyerNotification] 구매자 모달 콜백 등록됨')
}

// 구매자 모달 콜백 해제
export const unregisterBuyerModalCallback = () => {
  buyerModalCallback = null
  console.log('🔔 [BuyerNotification] 구매자 모달 콜백 해제됨')
}

// 구매자 배송 도착 모달 표시 (전역에서 호출 가능)
export const showBuyerDeliveryModal = (deliveryData) => {
  console.log('🏠 [BuyerNotification] 구매자 배송 도착 모달 표시 요청:', deliveryData)

  const modalData = {
    deliveryId: deliveryData.deliveryId,
    productTitle: deliveryData.productTitle || deliveryData.title || '상품명 없음',
    sellerName: deliveryData.sellerName || deliveryData.seller || '판매자명 없음',
    buyerName: deliveryData.buyerName || deliveryData.buyer || '구매자명 없음',
    timestamp: deliveryData.timestamp || new Date().toISOString(),
    sellerAddress: deliveryData.sellerAddress,
    buyerAddress: deliveryData.buyerAddress
  }

  console.log('🏠 [BuyerNotification] 변환된 모달 데이터:', modalData)

  // 등록된 콜백이 있으면 호출
  if (buyerModalCallback) {
    console.log('🏠 [BuyerNotification] ✅ 등록된 콜백으로 모달 표시')
    buyerModalCallback(modalData)
  } else {
    console.warn('🏠 [BuyerNotification] ⚠️ 구매자 모달 콜백이 등록되지 않음')

    // 브라우저 알림으로 대체
    if ('Notification' in window && Notification.permission === 'granted') {
      new Notification('📦 배송 도착!', {
        body: `배송 ID ${modalData.deliveryId}번 상품이 도착했습니다. 물건을 수령해 주세요.`,
        icon: '/icons/icon-192x192.png',
        badge: '/icons/badge-72x72.png',
        tag: 'buyer-delivery-arrival',
        data: modalData,
        requireInteraction: true
      })
    }

    // 콘솔에 안내 메시지 출력
    console.log('🏠 [BuyerNotification] 📱 구매자 모달을 표시하려면 해당 컴포넌트가 마운트되어야 합니다.')
  }

  // 커스텀 이벤트 발송 (다른 컴포넌트에서 처리 가능)
  const event = new CustomEvent('buyerDeliveryArrived', {
    detail: modalData
  })
  window.dispatchEvent(event)
  console.log('🏠 [BuyerNotification] buyerDeliveryArrived 커스텀 이벤트 발송 완료')
}

// 구매자 수령 완료 처리
export const handleBuyerPickupComplete = (deliveryId) => {
  console.log('✅ [BuyerNotification] 구매자 수령 완료 처리:', deliveryId)

  // 커스텀 이벤트 발송
  const event = new CustomEvent('buyerPickupCompleted', {
    detail: { deliveryId }
  })
  window.dispatchEvent(event)
}

// 테스트용 함수들을 window 객체에 등록
if (typeof window !== 'undefined') {
  // 구매자 배송 도착 모달 테스트 (window에서 호출 가능)
  window.showBuyerDeliveryModal = showBuyerDeliveryModal

  // 테스트 데이터로 모달 표시
  window.testBuyerModal = () => {
    const testData = {
      deliveryId: 1,
      productTitle: '테스트 상품',
      sellerName: '테스트 판매자',
      buyerName: '테스트 구매자',
      timestamp: new Date().toISOString(),
      sellerAddress: '1동 301호',
      buyerAddress: '2동 201호'
    }
    showBuyerDeliveryModal(testData)
  }

  // FCM 알림 시뮬레이션
  window.testBuyerFCM = () => {
    if ('Notification' in window) {
      if (Notification.permission === 'granted') {
        new Notification('📦 배송 도착!', {
          body: '나르고가 귀하의 집 앞에 도착했습니다. 물건을 수령해 주세요.',
          icon: '/icons/icon-192x192.png',
          badge: '/icons/badge-72x72.png',
          tag: 'buyer-delivery-test',
          requireInteraction: true
        })
        console.log('✅ 구매자 FCM 테스트 알림 전송 완료')
      } else if (Notification.permission !== 'denied') {
        Notification.requestPermission().then(function(permission) {
          if (permission === 'granted') {
            window.testBuyerFCM()
          }
        })
      } else {
        console.warn('⚠️ 알림 권한이 거부되었습니다.')
      }
    } else {
      console.warn('⚠️ 브라우저가 알림을 지원하지 않습니다.')
    }
  }

  // 구매자 도착 API 호출 시뮬레이션
  window.testBuyerArrivalAPI = async () => {
    const deliveryId = '1'

    try {
      console.log('🏠 [TEST] 구매자 도착 API 호출 시뮬레이션...')

      const response = await fetch(`http://localhost:8080/robot/delivery/${deliveryId}/buyer/arrived`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('access_token') || 'test_token'}`
        },
        body: JSON.stringify({
          timestamp: new Date().toISOString()
        })
      })

      if (response.ok) {
        const data = await response.json()
        console.log('✅ [TEST] 구매자 도착 API 호출 성공:', data)

        // 모달 자동 표시
        setTimeout(() => {
          window.testBuyerModal()
        }, 1000)

        return data
      } else {
        console.error('❌ [TEST] 구매자 도착 API 호출 실패:', response.status)
      }
    } catch (error) {
      console.error('❌ [TEST] 구매자 도착 API 네트워크 오류:', error)
    }
  }

  console.log('🌐 [BuyerNotification] 전역 테스트 함수들이 등록되었습니다:')
  console.log('- window.showBuyerDeliveryModal(deliveryData)')
  console.log('- window.testBuyerModal()')
  console.log('- window.testBuyerFCM()')
  console.log('- window.testBuyerArrivalAPI()')
}

export default {
  showBuyerDeliveryModal,
  registerBuyerModalCallback,
  unregisterBuyerModalCallback,
  handleBuyerPickupComplete
}