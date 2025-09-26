// 판매자 로봇 도착 알림 전역 유틸리티

let sellerModalInstance = null
let sellerModalCallback = null

// 판매자 모달 콜백 등록 (컴포넌트에서 호출)
export const registerSellerModalCallback = (callback) => {
  sellerModalCallback = callback
  console.log('🔔 [SellerNotification] 판매자 모달 콜백 등록됨')
}

// 판매자 모달 콜백 해제
export const unregisterSellerModalCallback = () => {
  sellerModalCallback = null
  console.log('🔔 [SellerNotification] 판매자 모달 콜백 해제됨')
}

// 판매자 로봇 도착 모달 표시 (전역에서 호출 가능)
export const showSellerRobotArrivedModal = (deliveryData) => {
  console.log('🤖 [SellerNotification] 판매자 로봇 도착 모달 표시 요청:', deliveryData)

  const modalData = {
    deliveryId: deliveryData.deliveryId,
    productTitle: deliveryData.productTitle || deliveryData.title || '상품명 없음',
    buyerName: deliveryData.buyerName || deliveryData.buyer || '구매자명 없음',
    timestamp: deliveryData.timestamp || new Date().toISOString()
  }

  console.log('🤖 [SellerNotification] 변환된 모달 데이터:', modalData)

  // 등록된 콜백이 있으면 호출
  if (sellerModalCallback) {
    console.log('🤖 [SellerNotification] ✅ 등록된 콜백으로 모달 표시')
    sellerModalCallback(modalData)
  } else {
    console.warn('🤖 [SellerNotification] ⚠️ 판매자 모달 콜백이 등록되지 않음')

    // 브라우저 알림으로 대체
    if ('Notification' in window && Notification.permission === 'granted') {
      new Notification('🤖 나르고 도착!', {
        body: `나르고가 도착했습니다! 물건을 넣어주세요.`,
        icon: '/icons/icon-192x192.png',
        badge: '/icons/badge-72x72.png',
        tag: 'seller-robot-arrival',
        data: modalData,
        requireInteraction: true
      })
    }

    // 콘솔에 안내 메시지 출력
    console.log('🤖 [SellerNotification] 📱 판매자 모달을 표시하려면 해당 컴포넌트가 마운트되어야 합니다.')
  }

  // 커스텀 이벤트 발송 (다른 컴포넌트에서 처리 가능)
  const event = new CustomEvent('robotArrivedAtSeller', {
    detail: modalData
  })
  window.dispatchEvent(event)
  console.log('🤖 [SellerNotification] robotArrivedAtSeller 커스텀 이벤트 발송 완료')
}

// 판매자 물건 넣기 완료 처리
export const handleSellerPickupComplete = (deliveryId) => {
  console.log('✅ [SellerNotification] 판매자 물건 넣기 완료 처리:', deliveryId)

  // 커스텀 이벤트 발송
  const event = new CustomEvent('sellerPickupCompleted', {
    detail: { deliveryId }
  })
  window.dispatchEvent(event)
}

// 가게 가능 여부 확인 함수
export const checkStoreAvailability = async () => {
  try {
    console.log('🏪 [SellerNotification] 가게 가능 여부 확인 시작')

    // 현재 시간 확인
    const now = new Date()
    const hour = now.getHours()
    const minute = now.getMinutes()

    console.log(`🕐 [SellerNotification] 현재 시간: ${hour}:${minute.toString().padStart(2, '0')}`)

    // 영업 시간 확인 (제한 없음으로 변경)
    const isBusinessHour = true
    console.log('🏪 [SellerNotification] 영업 시간 내:', isBusinessHour)

    // 기타 가게 상태 확인 (예: 휴무일, 재고 등)
    // TODO: 실제 백엔드 API 호출로 대체
    const isStoreOpen = true // 임시로 항상 열림으로 설정
    const hasStock = true   // 임시로 항상 재고 있음으로 설정

    const availability = {
      isAvailable: isBusinessHour && isStoreOpen && hasStock,
      reason: !isBusinessHour ? '영업시간이 아닙니다' :
              !isStoreOpen ? '가게가 휴무입니다' :
              !hasStock ? '재고가 부족합니다' : '배송 가능합니다',
      businessHours: '24시간 운영',
      currentTime: `${hour}:${minute.toString().padStart(2, '0')}`
    }

    console.log('🏪 [SellerNotification] 가게 가능 여부:', availability)
    return availability

  } catch (error) {
    console.error('❌ [SellerNotification] 가게 가능 여부 확인 실패:', error)
    return {
      isAvailable: false,
      reason: '가게 상태 확인 중 오류가 발생했습니다',
      error: error.message
    }
  }
}

// 테스트용 함수들을 window 객체에 등록
if (typeof window !== 'undefined') {
  // 판매자 로봇 도착 모달 테스트 (window에서 호출 가능)
  window.showSellerRobotArrivedModal = showSellerRobotArrivedModal

  // 테스트 데이터로 모달 표시
  window.testSellerModal = () => {
    const testData = {
      deliveryId: 1,
      productTitle: '테스트 상품',
      buyerName: '테스트 구매자',
      timestamp: new Date().toISOString()
    }
    showSellerRobotArrivedModal(testData)
  }

  // FCM 알림 시뮬레이션
  window.testSellerFCM = () => {
    if ('Notification' in window) {
      if (Notification.permission === 'granted') {
        new Notification('🤖 나르고 도착!', {
          body: '나르고가 판매자 집 앞에 도착했습니다. 물건을 넣어주세요.',
          icon: '/icons/icon-192x192.png',
          badge: '/icons/badge-72x72.png',
          tag: 'seller-robot-test',
          requireInteraction: true
        })
        console.log('✅ 판매자 FCM 테스트 알림 전송 완료')
      } else if (Notification.permission !== 'denied') {
        Notification.requestPermission().then(function(permission) {
          if (permission === 'granted') {
            window.testSellerFCM()
          }
        })
      } else {
        console.warn('⚠️ 알림 권한이 거부되었습니다.')
      }
    } else {
      console.warn('⚠️ 브라우저가 알림을 지원하지 않습니다.')
    }
  }

  // 가게 가능 여부 테스트
  window.testStoreAvailability = async () => {
    const availability = await checkStoreAvailability()
    console.log('🏪 [TEST] 가게 가능 여부 결과:', availability)
    alert(`가게 가능 여부: ${availability.isAvailable ? '가능' : '불가능'}\n사유: ${availability.reason}`)
    return availability
  }

  // 판매자 도착 API 호출 시뮬레이션
  window.testSellerArrivalAPI = async () => {
    const deliveryId = '1'

    try {
      console.log('🤖 [TEST] 판매자 도착 API 호출 시뮬레이션...')

      const response = await fetch(`http://localhost:8080/robot/delivery/${deliveryId}/seller/arrived`, {
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
        console.log('✅ [TEST] 판매자 도착 API 호출 성공:', data)

        // 모달 자동 표시
        setTimeout(() => {
          window.testSellerModal()
        }, 1000)

        return data
      } else {
        console.error('❌ [TEST] 판매자 도착 API 호출 실패:', response.status)
      }
    } catch (error) {
      console.error('❌ [TEST] 판매자 도착 API 네트워크 오류:', error)
    }
  }

  console.log('🌐 [SellerNotification] 전역 테스트 함수들이 등록되었습니다:')
  console.log('- window.showSellerRobotArrivedModal(deliveryData)')
  console.log('- window.testSellerModal()')
  console.log('- window.testSellerFCM()')
  console.log('- window.testStoreAvailability()')
  console.log('- window.testSellerArrivalAPI()')
}

export default {
  showSellerRobotArrivedModal,
  registerSellerModalCallback,
  unregisterSellerModalCallback,
  handleSellerPickupComplete,
  checkStoreAvailability
}