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
      <button class="icon-btn" @click="openSettings" title="설정">
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
          <path d="M12 15.5A3.5 3.5 0 0 1 8.5 12A3.5 3.5 0 0 1 12 8.5A3.5 3.5 0 0 1 15.5 12A3.5 3.5 0 0 1 12 15.5M19.43 12.98C19.47 12.66 19.5 12.34 19.5 12C19.5 11.66 19.47 11.34 19.43 11.02L21.54 9.37C21.73 9.22 21.78 8.95 21.66 8.73L19.66 5.27C19.54 5.05 19.27 4.96 19.05 5.05L16.56 6.05C16.04 5.65 15.48 5.32 14.87 5.07L14.5 2.42C14.46 2.18 14.25 2 14 2H10C9.75 2 9.54 2.18 9.5 2.42L9.13 5.07C8.52 5.32 7.96 5.66 7.44 6.05L4.95 5.05C4.73 4.96 4.46 5.05 4.34 5.27L2.34 8.73C2.22 8.95 2.27 9.22 2.46 9.37L4.57 11.02C4.53 11.34 4.5 11.67 4.5 12C4.5 12.33 4.53 12.66 4.57 12.98L2.46 14.63C2.27 14.78 2.22 15.05 2.34 15.27L4.34 18.73C4.46 18.95 4.73 19.04 4.95 18.95L7.44 17.95C7.96 18.35 8.52 18.68 9.13 18.93L9.5 21.58C9.54 21.82 9.75 22 10 22H14C14.25 22 14.46 21.82 14.5 21.58L14.87 18.93C15.48 18.68 16.04 18.34 16.56 17.95L19.05 18.95C19.27 19.04 19.54 18.95 19.66 18.73L21.66 15.27C21.78 15.05 21.73 14.78 21.54 14.63L19.43 12.98Z" fill="currentColor"/>
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
const openSettings = () => console.log('설정 열기');
</script>

<style scoped>
.mypage-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 16px 20px;
  background-color: var(--surface, #fff);
  border-bottom: 1px solid rgba(0,0,0,0.06);
}

/* 타이틀 */
.header-title {
  font-size: 16px;
  font-weight: 700;
  color: var(--secondary, #2c3e50);
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
  background-color: transparent;
  color: var(--muted, #6b7280);
  cursor: pointer;
  transition: all 0.2s ease;
}

.icon-btn:hover {
  background-color: var(--surface-2, #f3f6fa);
  color: var(--primary, #4682b4);
}

.icon-btn:active {
  transform: scale(0.95);
}

/* 뒤로가기 버튼 별도 지정 */
.back-btn {
  margin-right: 8px;
}
</style>
