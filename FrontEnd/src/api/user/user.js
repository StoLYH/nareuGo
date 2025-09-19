import apiClient from '../client.js'

/**
 * 사용자 정보 관련 API 함수들
 */

/**
 * 사용자 프로필 정보 조회
 * @param {number} userId - 사용자 ID
 * @returns {Promise} API 응답
 */
export const getUserProfile = async (userId) => {
  try {
    const response = await apiClient.get(`/user/profile/${userId}`)
    return response.data
  } catch (error) {
    console.error('사용자 프로필 조회 실패:', error)
    throw error
  }
}

/**
 * 결제용 사용자 정보 조회
 * @param {number} userId - 사용자 ID
 * @returns {Promise} API 응답
 */
export const getUserForPayment = async (userId) => {
  try {
    const response = await apiClient.get(`/payment/users/${userId}`)
    return response.data
  } catch (error) {
    console.error('결제용 사용자 정보 조회 실패:', error)
    throw error
  }
}

/**
 * 일반 로그인용 사용자 정보 조회 (UserController)
 * @param {number} userId - 사용자 ID
 * @returns {Promise} API 응답
 */
export const getUserInfo = async (userId) => {
  try {
    const response = await apiClient.get(`/general-login/users/${userId}`)
    return response.data
  } catch (error) {
    console.error('사용자 정보 조회 실패:', error)
    throw error
  }
}

/**
 * 현재 로그인된 사용자의 정보 조회 (localStorage 또는 auth store에서)
 * @returns {Object|null} 사용자 정보 객체 또는 null
 */
export const getCurrentUser = () => {
  try {
    // localStorage에서 사용자 정보 가져오기
    const userData = localStorage.getItem('user')
    if (userData) {
      return JSON.parse(userData)
    }
    return null
  } catch (error) {
    console.error('현재 사용자 정보 조회 실패:', error)
    return null
  }
}

/**
 * 현재 로그인된 사용자의 userId 조회
 * @returns {number|null} 사용자 ID 또는 null
 */
export const getCurrentUserId = () => {
  try {
    const userData = getCurrentUser()
    return userData?.userId || null
  } catch (error) {
    console.error('현재 사용자 ID 조회 실패:', error)
    return null
  }
}

/**
 * 현재 로그인된 사용자의 프로필 정보 조회
 * @returns {Promise} API 응답
 */
export const getCurrentUserProfile = async () => {
  const userId = getCurrentUserId()
  if (!userId) {
    throw new Error('로그인된 사용자가 없습니다.')
  }
  return await getUserProfile(userId)
}

/**
 * 마이페이지 사용자 정보 조회 (기존 컴포넌트에서 사용하던 엔드포인트)
 * @param {number} userId - 사용자 ID
 * @returns {Promise} API 응답
 */
export const getMyPageUserInfo = async (userId) => {
  try {
    const response = await apiClient.get(`/mypage/user/${userId}`)
    return response.data
  } catch (error) {
    console.error('마이페이지 사용자 정보 조회 실패:', error)
    throw error
  }
}

/**
 * 현재 로그인된 사용자의 마이페이지 정보 조회
 * @returns {Promise} API 응답
 */
export const getCurrentUserMyPageInfo = async () => {
  const userId = getCurrentUserId()
  if (!userId) {
    throw new Error('로그인된 사용자가 없습니다.')
  }
  return await getMyPageUserInfo(userId)
}

// 기본 export로 모든 함수들을 객체로 제공
export default {
  getUserProfile,
  getUserForPayment,
  getUserInfo,
  getCurrentUser,
  getCurrentUserId,
  getCurrentUserProfile,
  getMyPageUserInfo,
  getCurrentUserMyPageInfo
}
