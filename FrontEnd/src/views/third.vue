<template>
  <div class="chat-page">
    <!-- 헤더 -->
    <div class="chat-header">
      <button class="back-btn">
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
          <path d="M15 18L9 12L15 6" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
        </svg>
      </button>
      <div class="header-info">
        <h2>Flipkart Health+</h2>
        <span>HEALTH ORGANIZATION</span>
      </div>
    </div>

    <!-- 채팅 메시지 영역 -->
    <div class="chat-messages" ref="chatMessages" @scroll="handleScroll">
      <!-- 로딩 인디케이터 -->
      <div v-if="loading" class="loading-indicator">
        <div class="loading-spinner"></div>
        <span>이전 메시지 로딩 중...</span>
      </div>
      
      <!-- 메시지 목록 -->
      <div v-for="message in messages" :key="message.id" class="message" :class="message.type">
        <div class="message-bubble">
          {{ message.text }}
        </div>
      </div>
      
      <!-- 더 이상 메시지가 없을 때 -->
      <div v-if="!hasMoreMessages && messages.length > 0" class="no-more-messages">
        <span>더 이상 메시지가 없습니다</span>
      </div>
    </div>

    <!-- 메시지 입력 영역 -->
    <div class="message-input">
      <button class="emoji-btn">
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
          <path d="M12 22C17.5228 22 22 17.5228 22 12C22 6.47715 17.5228 2 12 2C6.47715 2 2 6.47715 2 12C2 17.5228 6.47715 22 12 22Z" stroke="currentColor" stroke-width="2"/>
          <path d="M8 14S9.5 16 12 16S16 14 16 14" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
          <path d="M9 9H9.01" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
          <path d="M15 9H15.01" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
        </svg>
      </button>
      <input type="text" placeholder="Message" class="message-text" />
      <button class="camera-btn">
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
          <path d="M23 19C23 19.5304 22.7893 20.0391 22.4142 20.4142C22.0391 20.7893 21.5304 21 21 21H3C2.46957 21 1.96086 20.7893 1.58579 20.4142C1.21071 20.0391 1 19.5304 1 19V8C1 7.46957 1.21071 6.96086 1.58579 6.58579C1.96086 6.21071 2.46957 6 3 6H7L9 4H15L17 6H21C21.5304 6 22.0391 6.21071 22.4142 6.58579C22.7893 6.96086 23 7.46957 23 8V19Z" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
          <circle cx="12" cy="13" r="4" stroke="currentColor" stroke-width="2"/>
        </svg>
      </button>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, nextTick } from 'vue'

const chatMessages = ref(null)
const messages = ref([])
const loading = ref(false)
const hasMoreMessages = ref(true)
const currentPage = ref(1)

// 더미 메시지 데이터 생성
const generateMessages = (page, count = 10) => {
  const newMessages = []
  const messageTexts = [
    "Hello,i'm fine,how can i help you?",
    "What is the best programming language?",
    "How are you doing today?",
    "Can you help me with this problem?",
    "Thank you for your assistance!",
    "I have a question about the project",
    "Let's schedule a meeting",
    "The weather is nice today",
    "I'm working on the new feature",
    "Have you seen the latest update?"
  ]
  
  for (let i = 0; i < count; i++) {
    const messageId = (page - 1) * count + i + 1
    const randomText = messageTexts[Math.floor(Math.random() * messageTexts.length)]
    const isReceived = Math.random() > 0.5
    
    newMessages.push({
      id: messageId,
      text: randomText,
      type: isReceived ? 'received' : 'sent'
    })
  }
  
  return newMessages
}

// 초기 메시지 로드
const loadInitialMessages = () => {
  messages.value = generateMessages(1, 5)
  hasMoreMessages.value = true
}

// 이전 메시지 로드
const loadMoreMessages = async () => {
  if (loading.value || !hasMoreMessages.value) return
  
  loading.value = true
  
  // 스크롤 위치 저장
  const scrollHeight = chatMessages.value.scrollHeight
  
  // 새로운 메시지 로드
  await new Promise(resolve => setTimeout(resolve, 1000)) // 로딩 시뮬레이션
  
  const newMessages = generateMessages(currentPage.value + 1, 10)
  
  if (newMessages.length === 0) {
    hasMoreMessages.value = false
  } else {
    messages.value = [...newMessages, ...messages.value]
    currentPage.value++
  }
  
  loading.value = false
  
  // 스크롤 위치 복원
  await nextTick()
  chatMessages.value.scrollTop = chatMessages.value.scrollHeight - scrollHeight
}

// 스크롤 이벤트 처리
const handleScroll = () => {
  if (chatMessages.value.scrollTop <= 50) {
    loadMoreMessages()
  }
}

onMounted(() => {
  loadInitialMessages()
  
  // 채팅 메시지 영역을 맨 아래로 스크롤
  nextTick(() => {
    if (chatMessages.value) {
      chatMessages.value.scrollTop = chatMessages.value.scrollHeight
    }
  })
})
</script>

<style scoped>
.chat-page {
  height: 100vh;
  display: flex;
  flex-direction: column;
  background-color: white;
}

/* 헤더 */
.chat-header {
  display: flex;
  align-items: center;
  padding: 16px;
  border-bottom: 1px solid #f0f0f0;
  background-color: white;
}

.back-btn {
  margin-right: 16px;
  color: #333;
}

.header-info h2 {
  font-size: 18px;
  font-weight: 600;
  margin: 0 0 2px 0;
  color: #333;
}

.header-info span {
  font-size: 12px;
  color: #666;
}

/* 채팅 메시지 영역 */
.chat-messages {
  flex: 1;
  padding: 16px;
  overflow-y: auto;
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.message {
  display: flex;
}

.message.received {
  justify-content: flex-start;
}

.message.sent {
  justify-content: flex-end;
}

.message-bubble {
  max-width: 70%;
  padding: 12px 16px;
  border-radius: 18px;
  font-size: 14px;
  line-height: 1.4;
}

.message.received .message-bubble {
  background-color: #f0f0f0;
  color: #333;
}

.message.sent .message-bubble {
  background-color: #007bff;
  color: white;
}

/* 메시지 입력 영역 */
.message-input {
  display: flex;
  align-items: center;
  padding: 16px;
  background-color: white;
  border-top: 1px solid #f0f0f0;
}

.emoji-btn,
.camera-btn {
  color: #666;
  margin: 0 8px;
}

.message-text {
  flex: 1;
  padding: 12px 16px;
  border: 1px solid #e0e0e0;
  border-radius: 24px;
  font-size: 14px;
  outline: none;
  background-color: #f8f8f8;
}

.message-text:focus {
  border-color: #007bff;
  background-color: white;
}

.message-text::placeholder {
  color: #999;
}

/* 로딩 인디케이터 */
.loading-indicator {
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 16px;
  color: #666;
  font-size: 14px;
  gap: 8px;
}

.loading-spinner {
  width: 16px;
  height: 16px;
  border: 2px solid #e0e0e0;
  border-top: 2px solid #007bff;
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}

/* 더 이상 메시지가 없을 때 */
.no-more-messages {
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 16px;
  color: #999;
  font-size: 12px;
}
</style>
