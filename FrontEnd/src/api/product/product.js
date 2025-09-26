import axios from 'axios'

// API 기본 URL
const BASE_URL = import.meta.env.VITE_API_BASE_URL || 'http://localhost:8080'

// 상품 등록 API (S3 presigned URL 받기)
export const createProduct = async (productData) => {
  try {
    const response = await axios.post(`${BASE_URL}/products`, productData, {
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${localStorage.getItem('access_token')}`
      }
    })
    return response.data
  } catch (error) {
    console.error('상품 등록 실패:', error)
    throw error
  }
}

// S3 직접 업로드 함수
export const uploadToS3 = async (presignedUrl, file) => {
  try {
    const response = await fetch(presignedUrl, {
      method: 'PUT',
      body: file,
      headers: {
        'Content-Type': file.type,
      },
    })

    if (!response.ok) {
      throw new Error(`S3 업로드 실패: ${response.status} ${response.statusText}`)
    }

    return response
  } catch (error) {
    console.error('S3 업로드 에러:', error)
    throw error
  }
}

// 상품 목록 조회 API - 사용자 ID로 같은 위치의 상품들 조회
export const getProducts = async (userId) => {
  try {
    const response = await axios.get(`${BASE_URL}/products/${userId}`, {
      headers: {
        'Authorization': `Bearer ${localStorage.getItem('access_token')}`
      }
    })
    return response.data
  } catch (error) {
    console.error('상품 목록 조회 실패:', error)
    throw error
  }
}

// 상품 상세 조회 API
export const getProductDetail = async (productId) => {
  try {
    console.log('상품 상세 조회 시도:', productId, `${BASE_URL}/products/item/${productId}`)
    const response = await axios.get(`${BASE_URL}/products/item/${productId}`, {
      headers: {
        'Authorization': `Bearer ${localStorage.getItem('access_token')}`
      }
    })
    console.log('상품 상세 조회 성공:', response.data)
    return response.data
  } catch (error) {
    console.error('상품 상세 조회 실패:', error.response?.status, error.response?.data, error.message)
    throw error
  }
}
