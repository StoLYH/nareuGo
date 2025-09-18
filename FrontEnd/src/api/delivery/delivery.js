import apiClient from '../client'

// 배송 데이터 조회 API
export const getDeliveries = async (userId) => {
  try {
    const response = await apiClient.get(`/mypage/deliveries/${userId}`)
    return response.data
  } catch (error) {
    console.error('배송 데이터 조회 실패:', error)
    throw error
  }
}

// 배송 상세 정보 조회 API (필요시 구현)
export const getDeliveryDetail = async (deliveryId) => {
  try {
    const response = await apiClient.get(`/deliveries/${deliveryId}`)
    return response.data
  } catch (error) {
    console.error('배송 상세 정보 조회 실패:', error)
    throw error
  }
}

// 배송 상태 업데이트 API (필요시 구현)
export const updateDeliveryStatus = async (deliveryId, status) => {
  try {
    const response = await apiClient.patch(`/deliveries/${deliveryId}/status`, {
      status: status
    })
    return response.data
  } catch (error) {
    console.error('배송 상태 업데이트 실패:', error)
    throw error
  }
}

// 배송지 변경 API (필요시 구현)
export const updateDeliveryAddress = async (deliveryId, newAddress) => {
  try {
    const response = await apiClient.patch(`/deliveries/${deliveryId}/address`, {
      destination: newAddress
    })
    return response.data
  } catch (error) {
    console.error('배송지 변경 실패:', error)
    throw error
  }
}

// 배송 문의 API (필요시 구현)
export const createDeliveryInquiry = async (deliveryId, inquiryData) => {
  try {
    const response = await apiClient.post(`/deliveries/${deliveryId}/inquiries`, inquiryData)
    return response.data
  } catch (error) {
    console.error('배송 문의 생성 실패:', error)
    throw error
  }
}

// 배송 추적 정보 조회 API (실시간 위치 등)
export const getDeliveryTracking = async (trackingNumber) => {
  try {
    const response = await apiClient.get(`/deliveries/tracking/${trackingNumber}`)
    return response.data
  } catch (error) {
    console.error('배송 추적 정보 조회 실패:', error)
    throw error
  }
}

// 배송 히스토리 조회 API
export const getDeliveryHistory = async (userId, page = 1, limit = 10) => {
  try {
    const response = await apiClient.get(`/mypage/deliveries/${userId}/history`, {
      params: {
        page,
        limit
      }
    })
    return response.data
  } catch (error) {
    console.error('배송 히스토리 조회 실패:', error)
    throw error
  }
}
