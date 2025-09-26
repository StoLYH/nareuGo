// ROS2 통신 유틸리티
// WebSocket 또는 ROS Bridge를 통한 ROS2 노드와의 통신

// API 기본 URL
const BASE_URL = import.meta.env.VITE_BASE_URL || 'http://localhost:8080'

class ROS2Communication {
  constructor() {
    this.websocket = null
    this.isConnected = false
    this.reconnectAttempts = 0
    this.maxReconnectAttempts = 5
    this.reconnectDelay = 1000 // 1초
  }

  // ROS2 브릿지 연결
  async connect(url = 'ws://localhost:9090') {
    try {
      console.log('🤖 [ROS2] 연결 시도:', url)

      this.websocket = new WebSocket(url)

      return new Promise((resolve, reject) => {
        this.websocket.onopen = () => {
          console.log('✅ [ROS2] WebSocket 연결 성공')
          this.isConnected = true
          this.reconnectAttempts = 0
          resolve(true)
        }

        this.websocket.onmessage = (event) => {
          console.log('📨 [ROS2] 메시지 수신:', event.data)
          this.handleMessage(JSON.parse(event.data))
        }

        this.websocket.onclose = () => {
          console.log('❌ [ROS2] WebSocket 연결 종료')
          this.isConnected = false
          this.handleReconnect(url)
        }

        this.websocket.onerror = (error) => {
          console.error('❌ [ROS2] WebSocket 에러:', error)
          this.isConnected = false
          reject(error)
        }

        // 연결 타임아웃 설정 (5초)
        setTimeout(() => {
          if (!this.isConnected) {
            reject(new Error('ROS2 연결 타임아웃'))
          }
        }, 5000)
      })
    } catch (error) {
      console.error('❌ [ROS2] 연결 실패:', error)
      throw error
    }
  }

  // 자동 재연결
  handleReconnect(url) {
    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++
      console.log(`🔄 [ROS2] 재연결 시도 ${this.reconnectAttempts}/${this.maxReconnectAttempts}`)

      setTimeout(() => {
        this.connect(url)
      }, this.reconnectDelay * this.reconnectAttempts)
    } else {
      console.error('❌ [ROS2] 최대 재연결 횟수 초과')
    }
  }

  // 메시지 처리
  handleMessage(message) {
    try {
      switch (message.op) {
        case 'publish':
          console.log('📢 [ROS2] 발행된 메시지:', message)
          break
        case 'service_response':
          console.log('🔔 [ROS2] 서비스 응답:', message)
          break
        default:
          console.log('📋 [ROS2] 기타 메시지:', message)
      }
    } catch (error) {
      console.error('❌ [ROS2] 메시지 처리 실패:', error)
    }
  }

  // 배송 시작 명령 전송 - 실제 로봇 HTTP API 사용
  async sendDeliveryStart(addresses) {
    try {
      console.log('🚀 [ROBOT] 배송 시작 명령 전송:', addresses)

      // 실제 로봇 백엔드 API 호출 - URL 쿼리 파라미터 사용
      const params = new URLSearchParams({
        sellerAddress: addresses.sellerAddress,
        buyerAddress: addresses.buyerAddress
      })
      const response = await fetch(`${BASE_URL}/robot/delivery/1/addresses?${params}`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json'
        }
      })

      if (response.ok) {
        console.log('✅ [ROBOT] 배송 시작 명령 전송 완료')
        return { success: true, message: '배송 시작 명령 전송 완료' }
      } else {
        throw new Error(`로봇 API 오류: ${response.status}`)
      }
    } catch (error) {
      console.error('❌ [ROBOT] 배송 시작 명령 전송 실패:', error)
      throw error
    }
  }

  // 로봇 상태 조회
  async getRobotStatus(robotId) {
    if (!this.isConnected) {
      console.warn('⚠️ [ROS2] 연결되지 않음, 기본 상태 반환')
      return { status: 'UNKNOWN', robot_id: robotId }
    }

    try {
      const message = {
        op: 'call_service',
        service: '/robot/get_status',
        args: {
          robot_id: robotId
        }
      }

      console.log('🔍 [ROS2] 로봇 상태 조회:', message)
      this.websocket.send(JSON.stringify(message))

      // 실제 구현에서는 응답을 기다려야 함
      return { success: true, message: '로봇 상태 조회 요청 전송 완료' }
    } catch (error) {
      console.error('❌ [ROS2] 로봇 상태 조회 실패:', error)
      throw error
    }
  }


  // 연결 종료
  disconnect() {
    if (this.websocket) {
      console.log('🔌 [ROS2] 연결 종료')
      this.websocket.close()
      this.websocket = null
      this.isConnected = false
    }
  }
}

// 싱글톤 인스턴스
const ros2Communication = new ROS2Communication()

// 편의 함수들
export const connectToROS2 = async (url) => {
  return await ros2Communication.connect(url)
}

export const sendAddressesToROS2 = async (addresses) => {
  return await ros2Communication.sendDeliveryStart(addresses)
}

export const getRobotStatusFromROS2 = async (robotId) => {
  return await ros2Communication.getRobotStatus(robotId)
}

export const disconnectFromROS2 = () => {
  ros2Communication.disconnect()
}

export default ros2Communication