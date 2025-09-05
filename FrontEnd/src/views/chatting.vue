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
            <path d="M15.5 14h-.79l-.28-.27C15.41 12.59 16 11.11 16 9.5 16 5.91 13.09 3 9.5 3S3 5.91 3 9.5 5.91 16 9.5 16c1.61 0 3.09-.59 4.23-1.57l.27.28v.79l5 4.99L20.49 19l-4.99-5zm-6 0C7.01 14 5 11.99 5 9.5S7.01 5 9.5 5 14 7.01 14 9.5 11.99 14 9.5 14z" fill="currentColor"/>
          </svg>
        </button>
      </div>
    </div>

    <!-- 채팅 목록 -->
    <main class="chat-list">
      <div class="chat-item" v-for="chat in chats" :key="chat.id">
        <div class="profile-image">
          <img :src="chat.profileImage" :alt="chat.name" />
        </div>
        <div class="chat-info">
          <h3 class="chat-name">{{ chat.name }}</h3>
          <p class="chat-location">{{ chat.location }}</p>
        </div>
        <button class="more-menu">
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none">
            <circle cx="12" cy="5" r="2" fill="currentColor"/>
            <circle cx="12" cy="12" r="2" fill="currentColor"/>
            <circle cx="12" cy="19" r="2" fill="currentColor"/>
          </svg>
        </button>
      </div>
    </main>

    <!-- 하단 네비게이션 -->
    <BottomNavigation 
      active-tab="chat"
      @navigate="handleNavigation"
    />
  </div>
</template>

<script setup>
import { ref } from 'vue'
import AppHeader from '../components/AppHeader.vue'
import BottomNavigation from '../components/BottomNavigation.vue'

const chats = ref([
  {
    id: 1,
    name: '김민수',
    location: '111동-101호',
    profileImage: 'https://images.unsplash.com/photo-1507003211169-0a1dd7228f2d?w=150&h=150&fit=crop&crop=face&auto=format&q=90',
    hasUnread: false
  },
  {
    id: 2,
    name: '이지은',
    location: '112동-103호',
    profileImage: 'https://images.unsplash.com/photo-1494790108755-2616b612b786?w=150&h=150&fit=crop&crop=face&auto=format&q=90',
    hasUnread: false
  },
  {
    id: 3,
    name: '박서준',
    location: '113동-205호',
    profileImage: 'https://images.unsplash.com/photo-1472099645785-5658abf4ff4e?w=150&h=150&fit=crop&crop=face&auto=format&q=90',
    hasUnread: true
  },
  {
    id: 4,
    name: '최유진',
    location: '114동-301호',
    profileImage: 'https://images.unsplash.com/photo-1438761681033-6461ffad8d80?w=150&h=150&fit=crop&crop=face&auto=format&q=90',
    hasUnread: true
  },
  {
    id: 5,
    name: '정현우',
    location: '115동-102호',
    profileImage: 'https://images.unsplash.com/photo-1500648767791-00dcc994a43e?w=150&h=150&fit=crop&crop=face&auto=format&q=90',
    hasUnread: true
  },
  {
    id: 6,
    name: '한소영',
    location: '116동-203호',
    profileImage: 'https://images.unsplash.com/photo-1544005313-94ddf0286df2?w=150&h=150&fit=crop&crop=face&auto=format&q=90',
    hasUnread: false
  },
  {
    id: 7,
    name: '강태현',
    location: '117동-304호',
    profileImage: 'https://images.unsplash.com/photo-1506794778202-cad84cf45f1d?w=150&h=150&fit=crop&crop=face&auto=format&q=90',
    hasUnread: true
  },
  {
    id: 8,
    name: '윤서연',
    location: '118동-105호',
    profileImage: 'https://images.unsplash.com/photo-1534528741775-53994a69daeb?w=150&h=150&fit=crop&crop=face&auto=format&q=90',
    hasUnread: false
  }
])

// 이벤트 핸들러들
const handleEdit = () => {
  console.log('편집 버튼 클릭')
}

const handleSearch = () => {
  console.log('검색 버튼 클릭')
}

const handleNotification = () => {
  console.log('알림 버튼 클릭')
}

const handleNavigation = (tab) => {
  console.log('네비게이션:', tab)
}
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

.more-menu {
  color: #007bff;
  background: none;
  border: none;
  padding: 8px;
  cursor: pointer;
}
</style>
