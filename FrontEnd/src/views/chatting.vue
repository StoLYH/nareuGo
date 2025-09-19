<template>
  <div class="second-page">
    <!-- 헤더 -->
    <AppHeader
      location="채팅"
      @edit="handleEdit"
      @search="handleSearch"
      @notification="handleNotification"
    />

    <!-- 검색창 -->
    <div class="search-container">
      <div class="search-box">
        <input type="text" placeholder="search" class="search-input" />
        <button class="search-icon">
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none">
            <path
              d="M15.5 14h-.79l-.28-.27C15.41 12.59 16 11.11 16 9.5 16 5.91 13.09 3 9.5 3S3 5.91 3 9.5 5.91 16 9.5 16c1.61 0 3.09-.59 4.23-1.57l.27.28v.79l5 4.99L20.49 19l-4.99-5zm-6 0C7.01 14 5 11.99 5 9.5S7.01 5 9.5 5 14 7.01 14 9.5 11.99 14 9.5 14z"
              fill="currentColor"
            />
          </svg>
        </button>
      </div>
    </div>

    <!-- 채팅 목록 -->
    <main class="chat-list">
      <!-- 로딩 상태 -->
      <div v-if="loading" class="loading-container">
        <div class="loading-spinner"></div>
        <p>채팅방 목록을 불러오는 중...</p>
      </div>

      <!-- 빈 목록 상태 -->
      <div v-else-if="chats.length === 0" class="empty-container">
        <p>아직 채팅방이 없습니다.</p>
        <p class="empty-subtitle">
          상품에서 "채팅하기"를 눌러 대화를 시작해보세요!
        </p>
      </div>

      <!-- 채팅방 목록 -->
      <div v-else>
        <div
          class="chat-item"
          v-for="chat in chats"
          :key="chat.id"
          @click="handleChatClick(chat.id)"
        >
          <div class="profile-image">
            <img :src="chat.profileImage" :alt="chat.name" />
          </div>
          <div class="chat-info">
            <h3 class="chat-name">{{ chat.name }}</h3>
            <p class="chat-location">{{ chat.location }}</p>
            <p class="last-message">{{ chat.lastMessage }}</p>
          </div>
          <div class="chat-meta">
            <span class="chat-time">{{ formatTime(chat.lastMessageAt) }}</span>
            <button class="more-menu" @click.stop="handleMoreMenu(chat.id)">
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none">
                <circle cx="12" cy="5" r="2" fill="currentColor" />
                <circle cx="12" cy="12" r="2" fill="currentColor" />
                <circle cx="12" cy="19" r="2" fill="currentColor" />
              </svg>
            </button>
          </div>
        </div>
      </div>
    </main>

    <!-- 하단 네비게이션 -->
    <BottomNavigation active-tab="chat" @navigate="handleNavigation" />
  </div>
</template>

<script setup>
import { ref, onMounted } from "vue";
import { useRouter } from "vue-router";
import AppHeader from "../components/AppHeader.vue";
import BottomNavigation from "../components/BottomNavigation.vue";
import { getUserChatRooms } from "@/api/chat/chat.js";

const router = useRouter();
const chats = ref([]);
const loading = ref(false);

// 채팅방 목록 로드
const loadChatRooms = async () => {
  try {
    loading.value = true;

    // 현재 사용자 정보 가져오기
    const userInfo = JSON.parse(localStorage.getItem("user"));
    if (!userInfo || !userInfo.userId) {
      console.error("사용자 정보가 없습니다.");
      router.push("/login");
      return;
    }

    console.log("채팅방 목록 로드 중... userId:", userInfo.userId);

    // API에서 채팅방 목록 가져오기
    const chatRooms = await getUserChatRooms(userInfo.userId.toString());

    console.log("받은 채팅방 목록:", chatRooms);

    // 채팅방 데이터를 UI 형식으로 변환
    chats.value = chatRooms.map((room) => {
      // 상대방 ID 찾기 (내가 user1이면 user2가 상대방, 반대도 마찬가지)
      const otherUserId =
        room.user1Id === userInfo.userId.toString()
          ? room.user2Id
          : room.user1Id;

      return {
        id: room.roomId,
        roomId: room.roomId,
        name: `사용자 ${otherUserId}`, // 실제로는 사용자 정보를 조회해야 함
        location: "위치 정보", // 실제로는 사용자 위치 정보를 조회해야 함
        profileImage: "/images/social/사용자더미.png", // 기본 프로필 이미지
        lastMessage: room.lastMessage || "메시지가 없습니다.",
        lastMessageAt: room.lastMessageAt || room.createdAt,
        otherUserId: otherUserId,
        hasUnread: false, // 추후 읽음 상태 관리 추가 가능
      };
    });
  } catch (error) {
    console.error("채팅방 목록 로드 실패:", error);
    chats.value = [];
  } finally {
    loading.value = false;
  }
};

// 시간 포맷팅 함수
const formatTime = (timeString) => {
  if (!timeString) return "";

  try {
    const messageTime = new Date(timeString);
    const now = new Date();
    const diffInMinutes = Math.floor((now - messageTime) / (1000 * 60));

    if (diffInMinutes < 1) return "방금 전";
    if (diffInMinutes < 60) return `${diffInMinutes}분 전`;

    const diffInHours = Math.floor(diffInMinutes / 60);
    if (diffInHours < 24) return `${diffInHours}시간 전`;

    const diffInDays = Math.floor(diffInHours / 24);
    if (diffInDays < 7) return `${diffInDays}일 전`;

    // 일주일 이상이면 날짜 표시
    return messageTime.toLocaleDateString("ko-KR", {
      month: "short",
      day: "numeric",
    });
  } catch (error) {
    return "";
  }
};

onMounted(() => {
  loadChatRooms();
});

// 이벤트 핸들러들
const handleEdit = () => {
  router.push("/item/register");
};

const handleSearch = () => {
  router.push("/search");
};

const handleNotification = () => {
  router.push("/notifications");
};

const handleChatClick = (chatId) => {
  // 채팅방 정보 찾기
  const chatRoom = chats.value.find((chat) => chat.id === chatId);
  if (chatRoom) {
    router.push({
      path: `/chat/${chatRoom.roomId}`,
      query: {
        otherUserId: chatRoom.otherUserId,
        otherUserName: chatRoom.name,
        productTitle: "", // 채팅방 목록에서는 상품 정보가 없을 수 있음
      },
    });
  }
};

const handleMoreMenu = (chatId) => {
  console.log("More menu clicked for chat:", chatId);
  // TODO: 채팅 옵션 메뉴 구현
};

const handleNavigation = (tab) => {
  switch (tab) {
    case "home":
      router.push("/items");
      break;
    case "chat":
      router.push("/chat");
      break;
    case "profile":
      router.push("/profile");
      break;
    default:
      console.log("Unknown navigation tab:", tab);
  }
};
</script>

<style scoped>
.second-page {
  width: 100%;
  height: 100vh;
  display: flex;
  flex-direction: column;
  background-color: white;
}

/* 검색창 */
.search-container {
  padding: 16px 20px;
  background-color: white;
}

.search-box {
  position: relative;
  display: flex;
  align-items: center;
}

.search-input {
  width: 100%;
  padding: 12px 40px 12px 16px;
  border: none;
  border-radius: 20px;
  background-color: #f5f5f5;
  font-size: 14px;
  outline: none;
}

.search-input::placeholder {
  color: #999;
}

.search-icon {
  position: absolute;
  right: 12px;
  color: #999;
  background: none;
  border: none;
  padding: 4px;
}

/* 채팅 목록 */
.chat-list {
  flex: 1;
  overflow-y: auto;
  padding: 0 20px;
  /* 스크롤바 숨기기 */
  scrollbar-width: none; /* Firefox */
  -ms-overflow-style: none; /* IE and Edge */
}

.chat-list::-webkit-scrollbar {
  display: none; /* Chrome, Safari, Opera */
}

.chat-item {
  display: flex;
  align-items: center;
  padding: 12px 0;
  gap: 12px;
  cursor: pointer;
  transition: background-color 0.2s;
}

.chat-item:hover {
  background-color: #f8f8f8;
}

.profile-image {
  width: 50px;
  height: 50px;
  border-radius: 50%;
  overflow: hidden;
  background-color: #f8f8f8;
  flex-shrink: 0;
}

.profile-image img {
  width: 100%;
  height: 100%;
  object-fit: cover;
}

.chat-info {
  flex: 1;
  display: flex;
  flex-direction: column;
  justify-content: center;
  min-width: 0;
}

.chat-name {
  font-size: 16px;
  font-weight: 600;
  color: #000;
  margin: 0 0 2px 0;
  line-height: 1.2;
  letter-spacing: -0.01em;
}

.chat-location {
  font-size: 13px;
  color: #666;
  margin: 0;
  font-weight: 400;
  line-height: 1.3;
  letter-spacing: -0.005em;
}

/* 로딩 및 빈 상태 */
.loading-container,
.empty-container {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 60px 20px;
  text-align: center;
}

.loading-spinner {
  width: 32px;
  height: 32px;
  border: 3px solid #f0f0f0;
  border-top: 3px solid #007bff;
  border-radius: 50%;
  animation: spin 1s linear infinite;
  margin-bottom: 16px;
}

@keyframes spin {
  0% {
    transform: rotate(0deg);
  }
  100% {
    transform: rotate(360deg);
  }
}

.empty-container p {
  margin: 0;
  color: #666;
  font-size: 16px;
}

.empty-subtitle {
  font-size: 14px !important;
  color: #999 !important;
  margin-top: 8px !important;
}

/* 채팅 아이템 레이아웃 수정 */
.chat-item {
  display: flex;
  align-items: flex-start;
  padding: 16px 0;
  gap: 12px;
  cursor: pointer;
  transition: background-color 0.2s;
  border-bottom: 1px solid #f5f5f5;
}

.chat-info {
  flex: 1;
  display: flex;
  flex-direction: column;
  justify-content: flex-start;
  min-width: 0;
  gap: 4px;
}

.last-message {
  font-size: 14px;
  color: #666;
  margin: 0;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.chat-meta {
  display: flex;
  flex-direction: column;
  align-items: flex-end;
  gap: 8px;
  flex-shrink: 0;
}

.chat-time {
  font-size: 12px;
  color: #999;
  white-space: nowrap;
}

.more-menu {
  color: #007bff;
  background: none;
  border: none;
  padding: 4px;
  cursor: pointer;
  border-radius: 4px;
  transition: background-color 0.2s;
}

.more-menu:hover {
  background-color: #f0f0f0;
}
</style>
