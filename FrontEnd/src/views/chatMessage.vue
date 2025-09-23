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
      <button
        class="payment-btn"
        title="결제"
        @click="handlePaymentClick"
        :disabled="isCreatingOrder"
      >
        <div v-if="isCreatingOrder" class="loading-spinner-small"></div>
        <svg v-else width="20" height="20" viewBox="0 0 24 24" fill="none">
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
        <span>{{ isCreatingOrder ? "주문 생성 중..." : "결제 승인" }}</span>
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
      <button class="send-btn" @click="sendMessage" title="전송">
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
          <path d="M22 2L11 13" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
          <path d="M22 2L15 22L11 13L2 9L22 2Z" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
        </svg>
      </button>
    </div>
    
    <!-- 결제 확인 모달 -->
    <transition name="fade-zoom">
      <div v-if="showPaymentConfirm" class="modal-overlay" @click.self="cancelPayment">
        <div class="modal-card">
          <div class="modal-header-row">
            <div class="modal-icon" aria-hidden="true">
              <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M18 8a6 6 0 10-12 0c0 7-3 9-3 9h18s-3-2-3-9" stroke="#E3A008" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                <path d="M13.73 21a2 2 0 01-3.46 0" stroke="#E3A008" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
              </svg>
            </div>
            <h3>판매자에게 결제 승인을 요청할까요?</h3>
          </div>
          <p class="modal-desc">*빠른 시연을 위해 결제 페이지로 바로 이동합니다*</p>
          <div class="modal-actions">
            <button class="btn-secondary" @click="cancelPayment">취소</button>
            <button class="btn-primary" @click="confirmPayment" :disabled="isCreatingOrder">
              {{ isCreatingOrder ? '진행 중...' : '요청하기' }}
            </button>
          </div>
        </div>
      </div>
    </transition>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, nextTick } from "vue";
import { useRoute, useRouter } from "vue-router";
import SockJS from "sockjs-client";
import { Stomp } from "@stomp/stompjs";
import { getChatMessages } from "@/api/chat/chat.js";

const route = useRoute();
const router = useRouter();

const chatMessages = ref(null);
const messages = ref([]);
const loading = ref(false);
const hasMoreMessages = ref(true);
const currentPage = ref(1);
const newMessage = ref("");
const isCreatingOrder = ref(false);
const showPaymentConfirm = ref(false);

// ===== 결제 버튼 클릭 처리 =====
const handlePaymentClick = () => {
  if (isCreatingOrder.value) return;
  showPaymentConfirm.value = true;
};

const cancelPayment = () => {
  if (isCreatingOrder.value) return;
  showPaymentConfirm.value = false;
};

const confirmPayment = async () => {
  if (isCreatingOrder.value) return;
  isCreatingOrder.value = true;
  try {
    await proceedPaymentFlow();
  } finally {
    isCreatingOrder.value = false;
    showPaymentConfirm.value = false;
  }
};

// 실제 주문 생성 및 결제 상세로 이동
const proceedPaymentFlow = async () => {
  console.log("결제 버튼 확인 - 주문 생성 시작");

  const pid = Number(productId.value);
  if (!pid || Number.isNaN(pid)) {
    alert("상품 정보 로딩 중입니다. 잠시 후 다시 시도해주세요.");
    return;
  }
  const buyerId = getCurrentUserId();

  const baseUrl = import.meta.env.VITE_BASE_URL || "http://localhost:8080";
  const response = await fetch(`${baseUrl}/orders`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ productId: pid, buyerId })
  });
  if (!response.ok) throw new Error("주문 생성에 실패했습니다.");

  const orderData = await response.json();
  const orderId = orderData.orderId;
  console.log("주문 생성 완료 - orderId:", orderId);

  router.push({ name: "PaymentPending", params: { orderId } });
};

// 현재 로그인한 사용자 ID 가져오기
const getCurrentUserId = () => {
  try {
    const userInfo = JSON.parse(localStorage.getItem("user") || "{}");
    return userInfo.userId || 1; // 기본값 1
  } catch (error) {
    console.error("사용자 정보 로드 실패:", error);
    return 1; // 기본값
  }
};

// 채팅 관련 데이터
const roomId = ref(route.params.id);
const otherUserId = ref(route.query.otherUserId);
const otherUserName = ref(route.query.otherUserName || "상대방");
const productTitle = ref(route.query.productTitle || "");
const currentUserId = ref(null);
const productId = ref(null); // 동적으로 가져올 상품 ID (기존 코드와 충돌 방지용 별도 상태)

// 채팅방의 상품 ID를 백엔드에서 조회하는 함수
// 주의: domain 폴더 및 기존 채팅/상품 코드에는 영향 주지 않도록 이 컴포넌트 내부에서만 사용합니다.
const fetchProductIdForRoom = async () => {
  try {
    const baseUrl = import.meta.env.VITE_BASE_URL || "http://localhost:8080";
    const resp = await fetch(`${baseUrl}/chat/rooms/${roomId.value}/product`);
    if (!resp.ok) throw new Error("상품 ID 조회 실패");
    const pid = await resp.json();
    productId.value = pid;
    console.log("채팅방 상품 ID 로드 완료:", pid);
  } catch (e) {
    console.error("채팅방 상품 ID 조회 중 오류:", e);
    // 실패해도 화면 사용은 가능하도록 하되 결제/메시지 전송 시 가드합니다.
  }
};

// WebSocket 관련
let stompClient = null;

// WebSocket 연결
const connectWebSocket = () => {
  // 환경 변수를 사용하여 WebSocket 서버 URL 설정
  const baseUrl = import.meta.env.VITE_BASE_URL || "http://localhost:8080";
  const socket = new SockJS(`${baseUrl}/ws`);
  stompClient = Stomp.over(socket);

  stompClient.connect(
    {},
    (frame) => {
      console.log("Connected: " + frame);

      // 자신의 큐를 구독하여 메시지 수신
      stompClient.subscribe(`/queue/user.${currentUserId.value}`, (message) => {
        const receivedMessage = JSON.parse(message.body);
        console.log("받은 메시지:", receivedMessage);

        // 현재 채팅방의 메시지인지 확인
        if (receivedMessage.roomId.toString() === roomId.value) {
          addMessageToChat(receivedMessage, "received");
        }
      });
    },
    (error) => {
      console.error("WebSocket 연결 실패:", error);
    }
  );
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
  // 채팅방 상품 ID가 아직 로드되지 않았다면 전송 제한
  const pid = Number(productId.value);
  if (!pid || Number.isNaN(pid)) {
    alert("상품 정보 로딩 중입니다. 잠시 후 다시 시도해주세요.");
    return;
  }

  const messageData = {
    roomId: parseInt(roomId.value),
    senderId: currentUserId.value.toString(),
    receiverId: otherUserId.value.toString(),
    content: newMessage.value.trim(),
    // 백엔드에서 조회한 상품 ID 사용
    productId: pid
  };

  console.log("메시지 전송:", messageData);

  // WebSocket을 통해 메시지 전송
  stompClient.send("/app/chat.send", {}, JSON.stringify(messageData));

  // 내가 보낸 메시지를 즉시 화면에 추가
  addMessageToChat(messageData, "sent");

  // 입력창 초기화
  newMessage.value = "";
};

// 채팅에 메시지 추가
const addMessageToChat = (messageData, type) => {
  const message = {
    id: Date.now() + Math.random(),
    text: messageData.content,
    type: type,
    timestamp: messageData.timestamp || new Date().toISOString(),
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

    messages.value = chatHistory.map((msg) => ({
      id: msg.messageId || Date.now() + Math.random(),
      text: msg.content,
      type:
        msg.senderId === currentUserId.value.toString() ? "sent" : "received",
      timestamp: msg.timestamp,
    }));

    hasMoreMessages.value = false; // 일단 페이징 없이
  } catch (error) {
    console.error("메시지 로드 실패:", error);
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
  const userInfo = JSON.parse(localStorage.getItem("user"));
  if (!userInfo || !userInfo.userId) {
    alert("로그인이 필요합니다.");
    router.push("/login");
    return;
  }

  currentUserId.value = userInfo.userId;

  console.log("채팅 정보:", {
    roomId: roomId.value,
    currentUserId: currentUserId.value,
    otherUserId: otherUserId.value,
    otherUserName: otherUserName.value,
    productTitle: productTitle.value,
  });

  // 채팅방의 상품 ID를 먼저 로드
  await fetchProductIdForRoom();

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
  /* 고정 헤더 높이만큼 여백 확보 */
  padding-top: 64px;
}

/* 헤더 */
.chat-header {
  position: fixed;
  top: 0;
  left: 50%;
  transform: translateX(-50%);
  width: 100%;
  max-width: 390px;
  z-index: 50;
  display: flex;
  justify-content: center;
  align-items: center;
  padding: 12px 16px;
  height: 64px;
  background-color: #4682B4;
  color: #fff;
  box-shadow: 0 4px 12px rgba(0,0,0,0.12);
  border-bottom: none;
}

.back-btn {
  position: absolute;
  left: 12px;
  top: 50%;
  transform: translateY(-50%);
  color: #fff;
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 40px;
  height: 40px;
  border-radius: 50%;
  background-color: rgba(255,255,255,0.18);
}
.back-btn:hover { background-color: rgba(255,255,255,0.28); }

.header-info { text-align: center; }
.header-info h2 {
  font-size: 18px;
  font-weight: 600;
  margin: 0 0 2px 0;
  color: #fff;
}

.header-info span {
  font-size: 12px;
  color: rgba(255,255,255,0.85);
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
  background-color: #f3f6fa;
  color: #2c3e50;
}

.message.sent .message-bubble {
  /* 헤더 블루와 겹치지 않도록, 어울리는 연한 민트/시안 톤 */
  background-color: #E7F6F8;
  color: #2c3e50;
}

/* 메시지 입력 영역 */
.message-input {
  display: flex;
  align-items: center;
  padding: 16px;
  background-color: white;
  border-top: 1px solid #4682B4;
}

.emoji-btn,
.send-btn {
  color: #6b7280;
  margin: 0 8px;
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 36px;
  height: 36px;
  border-radius: 18px;
  border: 1px solid #e5eaf0;
  background: #fff;
  transition: background-color 0.2s ease, color 0.2s ease, border-color 0.2s ease;
}
.emoji-btn:hover,
.send-btn:hover {
  background: #f3f6fa;
  border-color: #d7e3ef;
}

/* Distinct accents (soft tones that complement #4682B4) */
.emoji-btn { color: #4682b4; border-color: #4682b4; }
.emoji-btn:hover { background: #EEF5FA; color: #5E819D; border-color: #B8CBDC; }

.send-btn { color: #4682B4; border-color: #4682B4; }
.send-btn:hover { background: #ECF7F6; color: #5E9E9C; border-color: #BADAD9; }

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
  border-color: #4682b4;
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
  border-top: 2px solid #4682b4;
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


.payment-btn {
  position: absolute;
  right: 12px;
  top: 50%;
  transform: translateY(-50%);
  display: inline-flex;
  align-items: center;
  gap: 6px;
  padding: 8px 8px;
  border-radius: 9999px;
  border: none;
  background-color: rgba(255,255,255,0.18); /* 뒤로가기 버튼과 동일 톤 */
  color: #fff;
  font-size: 13px;
  font-weight: 700;
  cursor: pointer;
  transition: background-color 0.2s ease, box-shadow 0.2s ease, transform 0.15s ease;
}
.payment-btn:hover { background-color: rgba(255,255,255,0.28); box-shadow: 0 2px 8px rgba(0,0,0,0.12); }
.payment-btn:disabled { opacity: 0.7; cursor: not-allowed; box-shadow: none; transform: none; }

.loading-spinner-small {
  width: 16px;
  height: 16px;
  border: 2px solid #e0e0e0;
  border-top: 2px solid #4682b4;
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

/* ===== 모달 (중앙 고정) ===== */
.modal-overlay {
  position: fixed;
  inset: 0;
  background: rgba(0, 0, 0, 0.35);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 100; /* 헤더(50)보다 높게 */
  backdrop-filter: blur(2px);
}

.modal-card {
  width: calc(100% - 40px);
  max-width: 380px;
  background: #ffffff;
  border-radius: 16px;
  box-shadow: 0 12px 30px rgba(0, 0, 0, 0.2);
  padding: 22px;
  border: 1px solid #eef2f6;
}

.modal-card h3 {
  margin: 6px 0 8px 0;
  font-size: 18px;
  font-weight: 700;
  color: #2c3e50;
}

.modal-header-row {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-bottom: 6px;
}

.modal-desc {
  margin: 0 0 18px 0;
  font-size: 14px;
  color: #334155;
  line-height: 1.5;
  background: #f3f6fa;
  border: 1px solid #e5eaf0;
  border-radius: 10px;
  padding: 12px 14px;
}

.modal-actions {
  display: flex;
  justify-content: flex-end;
  gap: 10px;
  padding-top: 4px;
}

.btn-secondary,
.btn-primary {
  padding: 10px 14px;
  border-radius: 12px;
  font-size: 14px;
  font-weight: 700;
  border: none;
}

.btn-secondary {
  background: #f3f6fa;
  color: #334155;
}
.btn-secondary:hover { background: #e8eef6; }

.btn-primary {
  background: #4682B4;
  color: #fff;
}
.btn-primary:hover { background: #3b709a; }
</style>
