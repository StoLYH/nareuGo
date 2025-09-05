<template>
  <div class="notification-view">
    <!-- 상단 헤더: 페이지 제목, 설정 아이콘 -->
    <header class="notification-header">
      <div class="placeholder"></div>
      <!-- 제목 중앙 정렬을 위한 빈 공간 -->
      <h1>알림</h1>
      <svg
        @click="goToSettings"
        class="settings-btn"
        xmlns="http://www.w3.org/2000/svg"
        width="24"
        height="24"
        viewBox="0 0 24 24"
        fill="none"
        stroke="currentColor"
        stroke-width="2"
        stroke-linecap="round"
        stroke-linejoin="round"
      >
        <path
          d="M19.14,12.94c0.04-0.3,0.06-0.61,0.06-0.94c0-0.32-0.02-0.64-0.07-0.94l2.03-1.58c0.18-0.14,0.23-0.41,0.12-0.61 l-1.92-3.32c-0.12-0.22-0.37-0.29-0.59-0.22l-2.39,0.96c-0.5-0.38-1.03-0.7-1.62-0.94L14.4,2.81c-0.04-0.24-0.24-0.41-0.48-0.41 h-3.84c-0.24,0-0.44,0.17-0.48,0.41L9.2,5.77C8.61,6.01,8.08,6.33,7.58,6.71L5.19,5.75C4.97,5.68,4.72,5.75,4.6,5.97L2.68,9.29 c-0.11,0.2-0.06,0.47,0.12,0.61l2.03,1.58C4.78,11.76,4.76,12.24,4.76,12s0.02,0.24,0.07,0.46l-2.03,1.58 c-0.18,0.14-0.23,0.41-0.12,0.61l1.92,3.32c0.12,0.22,0.37,0.29,0.59,0.22l2.39-0.96c0.5,0.38,1.03,0.7,1.62,0.94l0.36,2.96 c0.04,0.24,0.24,0.41,0.48,0.41h3.84c0.24,0,0.44-0.17,0.48-0.41l0.36-2.96c0.59-0.24,1.12-0.56,1.62-0.94l2.39,0.96 c0.22,0.08,0.47,0.01,0.59-0.22l1.92-3.32c0.12-0.2,0.07-0.47-0.12-0.61L19.14,12.94z"
        />
        <circle cx="12" cy="12" r="3.5" />
      </svg>
    </header>

    <!-- 탭 네비게이션 -->
    <nav class="tabs" :class="`tab-${activeTab}-active`">
      <button
        @click="activeTab = 'news'"
        :class="{ active: activeTab === 'news' }"
      >
        새 소식
      </button>
      <button
        @click="activeTab = 'keywords'"
        :class="{ active: activeTab === 'keywords' }"
      >
        키워드
      </button>
      <div class="tab-indicator"></div>
    </nav>

    <!-- 알림 목록 (스크롤 영역) -->
    <main class="notification-content">
      <!-- '새 소식' 탭 내용 -->
      <ul v-if="activeTab === 'news'" class="notification-list">
        <li v-for="item in newsNotifications" :key="item.id" class="news-item">
          <div class="icon-wrapper">
            <!-- 아이콘 타입에 따라 다른 아이콘 표시 -->
            <svg
              v-if="item.type === 'delivery'"
              xmlns="http://www.w3.org/2000/svg"
              width="24"
              height="24"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              stroke-width="2"
              stroke-linecap="round"
              stroke-linejoin="round"
            >
              <rect x="1" y="3" width="15" height="13"></rect>
              <polygon points="16 8 20 8 23 11 23 16 16 16 16 8"></polygon>
              <circle cx="5.5" cy="18.5" r="2.5"></circle>
              <circle cx="18.5" cy="18.5" r="2.5"></circle>
            </svg>
            <svg
              v-else-if="item.type === 'gift'"
              xmlns="http://www.w3.org/2000/svg"
              width="24"
              height="24"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              stroke-width="2"
              stroke-linecap="round"
              stroke-linejoin="round"
            >
              <polyline points="20 12 20 22 4 22 4 12"></polyline>
              <rect x="2" y="7" width="20" height="5"></rect>
              <line x1="12" y1="22" x2="12" y2="7"></line>
              <path d="M12 7H7.5a2.5 2.5 0 0 1 0-5C11 2 12 7 12 7z"></path>
              <path d="M12 7h4.5a2.5 2.5 0 0 0 0-5C13 2 12 7 12 7z"></path>
            </svg>
          </div>
          <div class="text-wrapper">
            <span class="item-title">{{ item.title }}</span>
            <span class="item-message">{{ item.message }}</span>
            <span class="item-timestamp">{{ item.timestamp }}</span>
          </div>
        </li>
      </ul>

      <!-- '키워드' 탭 내용 -->
      <ul v-else-if="activeTab === 'keywords'" class="notification-list">
        <li
          v-for="item in keywordNotifications"
          :key="item.id"
          class="keyword-item"
        >
          <img :src="item.imageUrl" :alt="item.title" class="thumbnail" />
          <div class="text-wrapper">
            <span class="item-title">{{ item.title }}</span>
            <span class="item-meta">{{ item.location }} · {{ item.time }}</span>
            <span class="item-price"
              >{{ item.price.toLocaleString("ko-KR") }}원</span
            >
          </div>
        </li>
      </ul>
    </main>
  </div>
</template>

<script>
export default {
  name: "NotificationListView",
  data() {
    return {
      activeTab: "news", // 'news' 또는 'keywords'
      newsNotifications: [
        {
          id: 1,
          type: "delivery",
          title: "#HWDSF776567DS",
          message: "상품의 배송이 시작되었습니다.",
          timestamp: "09/02 10:16",
        },
        {
          id: 2,
          type: "delivery",
          title: "#EHCSF776567BJ",
          message: "상품의 배송이 시작되었습니다.",
          timestamp: "09/01 12:18",
        },
        {
          id: 3,
          type: "gift",
          title: "#ARGDGR61VCB2G",
          message: "상품의 배송이 완료되었습니다.",
          timestamp: "09/03 18:20",
        },
        {
          id: 4,
          type: "gift",
          title: "#YUK1UH56KHUK1",
          message: "상품의 배송이 완료되었습니다.",
          timestamp: "09/02 19:34",
        },
        {
          id: 5,
          type: "delivery",
          title: "#BN9V8N89CN1N2",
          message: "상품의 배송이 시작되었습니다.",
          timestamp: "09/04 13:16",
        },
      ],
      keywordNotifications: [
        {
          id: 1,
          imageUrl: "https://placehold.co/150x150/EFEFEF/333?text=AirPods",
          title: "에어팟 미니",
          location: "군자동",
          time: "3일 전",
          price: 100000,
        },
        {
          id: 2,
          imageUrl: "https://placehold.co/150x150/EFEFEF/333?text=AirPods",
          title: "에어팟 3세대",
          location: "군자동",
          time: "3일 전",
          price: 70000,
        },
        {
          id: 3,
          imageUrl: "https://placehold.co/150x150/EFEFEF/333?text=AirPods",
          title: "에어팟 2세대 오른쪽",
          location: "군자동",
          time: "3일 전",
          price: 30000,
        },
        {
          id: 4,
          imageUrl: "https://placehold.co/150x150/EFEFEF/333?text=AirPods",
          title: "에어팟 프로",
          location: "군자동",
          time: "3일 전",
          price: 220000,
        },
      ],
    };
  },
  methods: {
    goToSettings() {
      alert("설정 페이지로 이동");
    },
  },
};
</script>

<style scoped>
.notification-view {
  display: flex;
  flex-direction: column;
  height: 100%;
  background-color: #fff;
}

/* 상단 헤더 */
.notification-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 16px 16px;
  border-bottom: 1px solid #f0f0f0;
  flex-shrink: 0;
}
.notification-header h1 {
  font-size: 18px;
  font-weight: 600;
}
.placeholder,
.settings-btn {
  width: 24px;
  height: 24px;
}
.settings-btn {
  cursor: pointer;
  color: #555;
}

/* 탭 네비게이션 */
.tabs {
  position: relative;
  display: flex;
  flex-shrink: 0;
  border-bottom: 1px solid #f0f0f0;
}
.tabs button {
  flex: 1;
  padding: 14px 0;
  font-size: 16px;
  color: #888;
  background: none;
  border: none;
  cursor: pointer;
  transition: color 0.3s, font-weight 0.3s;
}
.tabs button.active {
  color: #000;
  font-weight: 600;
}
.tab-indicator {
  position: absolute;
  bottom: -1px;
  height: 2px;
  width: 50%;
  background-color: #000;
  transition: transform 0.3s ease-in-out;
}
/* 탭 활성화 시 인디케이터 이동 */
.tabs.tab-news-active .tab-indicator {
  transform: translateX(0%);
}
.tabs.tab-keywords-active .tab-indicator {
  transform: translateX(100%);
}

/* 알림 목록 영역 */
.notification-content {
  flex-grow: 1;
  overflow-y: auto;
}
.notification-list {
  padding: 0;
  margin: 0;
  list-style: none;
}

/* '새 소식' 아이템 스타일 */
.news-item {
  display: flex;
  align-items: center;
  padding: 16px;
  border-bottom: 1px solid #f0f0f0;
}
.icon-wrapper {
  flex-shrink: 0;
  width: 48px;
  height: 48px;
  border-radius: 50%;
  background-color: #f0f0f0;
  display: flex;
  justify-content: center;
  align-items: center;
  margin-right: 16px;
  color: #555;
}
.news-item .text-wrapper {
  display: flex;
  flex-direction: column;
}
.news-item .item-title {
  font-size: 14px;
  font-weight: 600;
  color: #333;
  margin-bottom: 4px;
}
.news-item .item-message {
  font-size: 14px;
  color: #555;
  margin-bottom: 6px;
}
.news-item .item-timestamp {
  font-size: 12px;
  color: #999;
}

/* '키워드' 아이템 스타일 */
.keyword-item {
  display: flex;
  align-items: center;
  padding: 16px;
  border-bottom: 1px solid #f0f0f0;
}
.keyword-item .thumbnail {
  flex-shrink: 0;
  width: 80px;
  height: 80px;
  border-radius: 8px;
  object-fit: cover;
  margin-right: 16px;
}
.keyword-item .text-wrapper {
  display: flex;
  flex-direction: column;
}
.keyword-item .item-title {
  font-size: 16px;
  color: #333;
  margin-bottom: 6px;
}
.keyword-item .item-meta {
  font-size: 13px;
  color: #888;
  margin-bottom: 8px;
}
.keyword-item .item-price {
  font-size: 16px;
  font-weight: 600;
  color: #4682b4; /* 당근마켓의 가격 색상 */
}

/* 스크롤바 숨기기 */
.notification-content::-webkit-scrollbar {
  display: none;
}
.notification-content {
  -ms-overflow-style: none;
  scrollbar-width: none;
}
</style>
