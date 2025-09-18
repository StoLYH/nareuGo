<template>
  <div class="chat-page">
    <!-- 헤더 -->
    <div class="chat-header">
      <button class="back-btn" @click="goBack">
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
          <path
            d="M15 18L9 12L15 6"
            stroke="currentColor"
            stroke-width="2"
            stroke-linecap="round"
            stroke-linejoin="round"
          />
        </svg>
      </button>
      <div class="header-info">
        <h2>{{ otherUserName }}</h2>
        <span>{{ productTitle }}</span>
      </div>
      <!-- 결제 아이콘 버튼 -->
      <button class="payment-btn" title="결제">
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
          <rect
            x="2"
            y="5"
            width="20"
            height="14"
            rx="2"
            ry="2"
            stroke="currentColor"
            stroke-width="2"
          />
          <line
            x1="2"
            y1="10"
            x2="22"
            y2="10"
            stroke="currentColor"
            stroke-width="2"
          />
        </svg>
        <span>결제</span>
      </button>
    </div>

    <!-- 채팅 메시지 영역 -->
    <div class="chat-messages" ref="chatMessages" @scroll="handleScroll">
      <!-- 로딩 인디케이터 -->
      <div v-if="loading" class="loading-indicator">
        <div class="loading-spinner"></div>
        <span>이전 메시지 로딩 중...</span>
      </div>

      <!-- 메시지 목록 -->
      <div
        v-for="message in messages"
        :key="message.id"
        class="message"
        :class="message.type"
      >
        <div class="message-bubble">
          {{ message.text }}
        </div>
      </div>

      <!-- 더 이상 메시지가 없을 때 -->
      <div
        v-if="!hasMoreMessages && messages.length > 0"
        class="no-more-messages"
      >
        <span>더 이상 메시지가 없습니다</span>
      </div>
    </div>

    <!-- 메시지 입력 영역 -->
    <div class="message-input">
      <button class="emoji-btn">
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
          <path
            d="M12 22C17.5228 22 22 17.5228 22 12C22 6.47715 17.5228 2 12 2C6.47715 2 2 6.47715 2 12C2 17.5228 6.47715 22 12 22Z"
            stroke="currentColor"
            stroke-width="2"
          />
          <path
            d="M8 14S9.5 16 12 16S16 14 16 14"
            stroke="currentColor"
            stroke-width="2"
            stroke-linecap="round"
          />
          <path
            d="M9 9H9.01"
            stroke="currentColor"
            stroke-width="2"
            stroke-linecap="round"
            stroke-linejoin="round"
          />
          <path
            d="M15 9H15.01"
            stroke="currentColor"
            stroke-width="2"
            stroke-linecap="round"
            stroke-linejoin="round"
          />
        </svg>
      </button>
      <input 
        type="text" 
        placeholder="Message" 
        class="message-text" 
        v-model="newMessage"
        @keyup.enter="sendMessage"
      />
      <button class="camera-btn">
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
          <path
            d="M23 19C23 19.5304 22.7893 20.0391 22.4142 20.4142C22.0391 20.7893 21.5304 21 21 21H3C2.46957 21 1.96086 20.7893 1.58579 20.4142C1.21071 20.0391 1 19.5304 1 19V8C1 7.46957 1.21071 6.96086 1.58579 6.58579C1.96086 6.21071 2.46957 6 3 6H7L9 4H15L17 6H21C21.5304 6 22.0391 6.21071 22.4142 6.58579C22.7893 6.96086 23 7.46957 23 8V19Z"
            stroke="currentColor"
            stroke-width="2"
            stroke-linecap="round"
            stroke-linejoin="round"
          />
          <circle
            cx="12"
            cy="13"
            r="4"
            stroke="currentColor"
            stroke-width="2"
          />
        </svg>
      </button>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, nextTick } from "vue";
import { useRoute, useRouter } from "vue-router";
import SockJS from 'sockjs-client';
import { Stomp } from '@stomp/stompjs';
import { getChatMessages } from '@/api/chat/chat.js';

const route = useRoute();
const router = useRouter();

const chatMessages = ref(null);
const messages = ref([]);
const loading = ref(false);
const hasMoreMessages = ref(true);
const currentPage = ref(1);
const newMessage = ref('');

// 채팅 관련 데이터
const roomId = ref(route.params.id);
const otherUserId = ref(route.query.otherUserId);
const otherUserName = ref(route.query.otherUserName || '상대방');
const productTitle = ref(route.query.productTitle || '');
const currentUserId = ref(null);

// WebSocket 관련
let stompClient = null;

// WebSocket 연결
const connectWebSocket = () => {
  // 환경 변수를 사용하여 WebSocket 서버 URL 설정
  const baseUrl = import.meta.env.VITE_BASE_URL || 'http://localhost:8080';
  const socket = new SockJS(`${baseUrl}/ws`);
  stompClient = Stomp.over(socket);
  
  stompClient.connect({}, (frame) => {
    console.log('Connected: ' + frame);
    
    // 자신의 큐를 구독하여 메시지 수신
    stompClient.subscribe(`/queue/user.${currentUserId.value}`, (message) => {
      const receivedMessage = JSON.parse(message.body);
      console.log('받은 메시지:', receivedMessage);
      
      // 현재 채팅방의 메시지인지 확인
      if (receivedMessage.roomId.toString() === roomId.value) {
        addMessageToChat(receivedMessage, 'received');
      }
    });
  }, (error) => {
    console.error('WebSocket 연결 실패:', error);
  });
};

// WebSocket 연결 해제
const disconnectWebSocket = () => {
  if (stompClient && stompClient.connected) {
    stompClient.disconnect();
  }
};

// 메시지 전송
const sendMessage = () => {
  if (!newMessage.value.trim() || !stompClient || !stompClient.connected) {
    return;
  }

  const messageData = {
    roomId: parseInt(roomId.value),
    senderId: currentUserId.value.toString(),
    receiverId: otherUserId.value.toString(),
    content: newMessage.value.trim()
  };

  console.log('메시지 전송:', messageData);

  // WebSocket을 통해 메시지 전송
  stompClient.send('/app/chat.send', {}, JSON.stringify(messageData));

  // 내가 보낸 메시지를 즉시 화면에 추가
  addMessageToChat(messageData, 'sent');
  
  // 입력창 초기화
  newMessage.value = '';
};

// 채팅에 메시지 추가
const addMessageToChat = (messageData, type) => {
  const message = {
    id: Date.now() + Math.random(),
    text: messageData.content,
    type: type,
    timestamp: messageData.timestamp || new Date().toISOString()
  };
  
  messages.value.push(message);
  
  // 스크롤을 맨 아래로
  nextTick(() => {
    if (chatMessages.value) {
      chatMessages.value.scrollTop = chatMessages.value.scrollHeight;
    }
  });
};

// 초기 메시지 로드 (API에서)
const loadInitialMessages = async () => {
  try {
    loading.value = true;
    const chatHistory = await getChatMessages(roomId.value);
    
    messages.value = chatHistory.map(msg => ({
      id: msg.messageId || Date.now() + Math.random(),
      text: msg.content,
      type: msg.senderId === currentUserId.value.toString() ? 'sent' : 'received',
      timestamp: msg.timestamp
    }));
    
    hasMoreMessages.value = false; // 일단 페이징 없이
  } catch (error) {
    console.error('메시지 로드 실패:', error);
    messages.value = []; // 빈 배열로 초기화
  } finally {
    loading.value = false;
  }
};

// 이전 메시지 로드
const loadMoreMessages = async () => {
  if (loading.value || !hasMoreMessages.value) return;

  loading.value = true;

  // 스크롤 위치 저장
  const scrollHeight = chatMessages.value.scrollHeight;

  // 새로운 메시지 로드
  await new Promise((resolve) => setTimeout(resolve, 1000)); // 로딩 시뮬레이션

  const newMessages = generateMessages(currentPage.value + 1, 10);

  if (newMessages.length === 0) {
    hasMoreMessages.value = false;
  } else {
    messages.value = [...newMessages, ...messages.value];
    currentPage.value++;
  }

  loading.value = false;

  // 스크롤 위치 복원
  await nextTick();
  chatMessages.value.scrollTop = chatMessages.value.scrollHeight - scrollHeight;
};

// 스크롤 이벤트 처리
const handleScroll = () => {
  if (chatMessages.value.scrollTop <= 50) {
    loadMoreMessages();
  }
};

onMounted(async () => {
  // 현재 사용자 정보 가져오기
  const userInfo = JSON.parse(localStorage.getItem('user'));
  if (!userInfo || !userInfo.userId) {
    alert('로그인이 필요합니다.');
    router.push('/login');
    return;
  }
  
  currentUserId.value = userInfo.userId;
  
  console.log('채팅 정보:', {
    roomId: roomId.value,
    currentUserId: currentUserId.value,
    otherUserId: otherUserId.value,
    otherUserName: otherUserName.value,
    productTitle: productTitle.value
  });

  // 초기 메시지 로드
  await loadInitialMessages();

  // WebSocket 연결
  connectWebSocket();

  // 채팅 메시지 영역을 맨 아래로 스크롤
  nextTick(() => {
    if (chatMessages.value) {
      chatMessages.value.scrollTop = chatMessages.value.scrollHeight;
    }
  });
});

// 뒤로가기
const goBack = () => {
  router.go(-1);
};

onUnmounted(() => {
  // 컴포넌트 해제 시 WebSocket 연결 해제
  disconnectWebSocket();
});
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
  0% {
    transform: rotate(0deg);
  }
  100% {
    transform: rotate(360deg);
  }
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

.payment-btn {
  margin-left: auto;
  display: flex;
  align-items: center;
  gap: 6px;
  padding: 6px 12px;
  border: none;
  border-radius: 20px;
  background-color: #28a745;
  color: white;
  font-size: 14px;
  font-weight: 500;
  cursor: pointer;
  transition: background-color 0.2s, transform 0.2s;
}

.payment-btn:hover {
  background-color: #218838;
  transform: scale(1.03);
}
</style>
