import axios from 'axios'

const BASE_URL = import.meta.env.VITE_BASE_URL || 'http://localhost:8080'

// 채팅방 생성 또는 기존 채팅방 찾기
export const findOrCreateChatRoom = async (user1Id, user2Id) => {
  try {
    const response = await axios.post(`${BASE_URL}/chat/rooms`, null, {
      params: {
        user1Id: user1Id,
        user2Id: user2Id
      }
    })
    return response.data // roomId 반환
  } catch (error) {
    console.error('채팅방 생성/찾기 실패:', error)
    throw error
  }
}

// 사용자의 채팅방 목록 조회
export const getUserChatRooms = async (userId) => {
  try {
    const response = await axios.get(`${BASE_URL}/chat/rooms/${userId}`)
    return response.data
  } catch (error) {
    console.error('채팅방 목록 조회 실패:', error)
    throw error
  }
}

// 특정 채팅방의 메시지 목록 조회
export const getChatMessages = async (roomId) => {
  try {
    const response = await axios.get(`${BASE_URL}/chat/rooms/${roomId}/messages`)
    return response.data
  } catch (error) {
    console.error('채팅 메시지 조회 실패:', error)
    throw error
  }
}
