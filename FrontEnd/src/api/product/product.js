import axios from 'axios'

// API 기본 URL
const BASE_URL = import.meta.env.VITE_BASE_URL || 'http://localhost:8080'

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

// 상품 목록 조회 API (추후 구현)
export const getProducts = async () => {
  try {
    const response = await axios.get(`${BASE_URL}/products`, {
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

// 상품 상세 조회 API (추후 구현)
export const getProductDetail = async (productId) => {
  try {
    const response = await axios.get(`${BASE_URL}/products/${productId}`, {
      headers: {
        'Authorization': `Bearer ${localStorage.getItem('access_token')}`
      }
    })
    return response.data
  } catch (error) {
    console.error('상품 상세 조회 실패:', error)
    throw error
  }
}
