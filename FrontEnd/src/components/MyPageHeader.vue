<template>
  <header class="mypage-header">
    <!-- 뒤로가기 버튼 -->
    <button class="icon-btn back-btn" @click="goBack" title="뒤로가기">
      <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
        <path d="M15 18L9 12L15 6" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
      </svg>
    </button>

    <!-- 타이틀 -->
    <h1 class="header-title">마이나르고</h1>

    <!-- 우측 액션 버튼 -->
    <div class="header-actions">
      <!-- <button class="icon-btn" @click="openCustomerService" title="고객센터">
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
          <path d="M12 1C13.1 1 14 1.9 14 3V7C14 8.1 13.1 9 12 9C10.9 9 10 8.1 10 7V3C10 1.9 10.9 1 12 1ZM19 10V14C19 15.1 18.1 16 17 16H15L13 18V6L15 8H17C18.1 8 19 8.9 19 10ZM5 10V14C5 15.1 5.9 16 7 16H9L11 18V6L9 8H7C5.9 8 5 8.9 5 10Z" fill="currentColor"/>
        </svg>
      </button> -->
      <button class="icon-btn" @click="goNotifications" title="알림">
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
          <path d="M12 22c1.1 0 2-.9 2-2h-4c0 1.1.89 2 2 2zm6-6v-5c0-3.07-1.64-5.64-4.5-6.32V4c0-.83-.67-1.5-1.5-1.5s-1.5.67-1.5 1.5v.68C7.63 5.36 6 7.92 6 11v5l-2 2v1h16v-1l-2-2z" fill="currentColor"/>
        </svg>
      </button>
    </div>
  </header>
</template>

<script setup>
import { useRouter } from 'vue-router';

const router = useRouter();

const goBack = () => router.go(-1);
const openCustomerService = () => console.log('고객센터 열기');
const goNotifications = () => router.push({ name: 'Notifications' });
</script>

<style scoped>
.mypage-header {
  position: fixed;
  top: 0;
  left: 50%;
  transform: translateX(-50%);
  width: 100%;
  max-width: 390px;
  z-index: 50;
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 16px 20px;
  background-color: #4682B4;
  color: #ffffff;
  border-bottom: none;
  box-shadow: 0 4px 12px rgba(0,0,0,0.12);
}

/* 타이틀 */
.header-title {
  font-size: 18px;
  font-weight: 700;
  color: #ffffff;
  margin: 0;
  text-align: center;
  flex: 1;
}

/* 우측 버튼 그룹 */
.header-actions {
  display: flex;
  gap: 12px;
}

/* 버튼 스타일 통일 */
.icon-btn {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 36px;
  height: 36px;
  border-radius: 50%;
  border: none;
  background-color: rgba(255,255,255,0.16);
  color: #ffffff;
  cursor: pointer;
  transition: all 0.2s ease;
}

.icon-btn:hover {
  background-color: rgba(255,255,255,0.28);
  color: #ffffff;
}

.icon-btn:active {
  transform: scale(0.95);
}

/* 뒤로가기 버튼 별도 지정 */
.back-btn {
  margin-right: 8px;
}
</style>
