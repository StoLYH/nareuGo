<template>
  <header class="delivery-header">
    <!-- 상단 바: 뒤로가기(좌) + 액션 아이콘(우) -->
    <div class="top-bar">
      <button class="icon-btn back-btn" @click="goBack" aria-label="뒤로가기">
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M15 18L9 12L15 6" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
        </svg>
      </button>
      <div class="header-icons">
        <button class="icon-btn" @click="openSupport" aria-label="고객센터">
          <!-- Headphones Icon -->
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M12 3a9 9 0 0 0-9 9v5a3 3 0 0 0 3 3h1a2 2 0 0 0 2-2v-4a2 2 0 0 0-2-2H5a7 7 0 0 1 14 0h-2a2 2 0 0 0-2 2v4a2 2 0 0 0 2 2h1a3 3 0 0 0 3-3v-5a9 9 0 0 0-9-9Z" fill="currentColor"/>
          </svg>
        </button>
        <!-- <button class="icon-btn" @click="openSettings" aria-label="설정">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M12 15.5A3.5 3.5 0 1 1 12 8.5a3.5 3.5 0 0 1 0 7Zm7.43-2.52c.03-.32.07-.64.07-.98s-.04-.66-.07-.98l2.11-1.65a.5.5 0 0 0 .11-.65l-2-3.46a.5.5 0 0 0-.61-.22l-2.49 1a7.04 7.04 0 0 0-1.7-.98l-.38-2.65A.5.5 0 0 0 13 2h-2a.5.5 0 0 0-.5.42l-.38 2.65c-.6.23-1.17.55-1.7.98l-2.49-1a.5.5 0 0 0-.61.22l-2 3.46a.5.5 0 0 0 .11.65L5.94 11c-.03.32-.07.64-.07.98s.04.66.07.98l-2.11 1.65a.5.5 0 0 0-.11.65l2 3.46c.13.23.4.32.61.22l2.49-1c.53.43 1.1.75 1.7.98l.38 2.65c.03.24.25.42.5.42h2c.25 0 .47-.18.5-.42l.38-2.65c.6-.23 1.17-.55 1.7-.98l2.49 1c.21.1.48.01.61-.22l2-3.46a.5.5 0 0 0-.11-.65L19.43 12.98Z" fill="currentColor"/>
          </svg>
        </button> -->

        <button class="icon-btn" @click="goNotifications" title="알림">
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" focusable="false" aria-hidden="true">
          <path d="M12 22c1.1 0 2-.9 2-2h-4c0 1.1.89 2 2 2zm6-6v-5c0-3.07-1.64-5.64-4.5-6.32V4c0-.83-.67-1.5-1.5-1.5s-1.5.67-1.5 1.5v.68C7.63 5.36 6 7.92 6 11v5l-2 2v1h16v-1l-2-2z" fill="currentColor"/>
        </svg>
      </button> 
      </div>
    </div>
    
    <!-- 메인 제목 -->
    <div class="header-content">
      <h1 class="header-title">택배 배송 조회</h1>
    </div>
    
    <!-- 검색창 -->
    <div class="search-container">
      <div class="search-box">
        <div class="search-icon">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <circle cx="11" cy="11" r="8" stroke="#6c757d" stroke-width="2"/>
            <path d="m21 21-4.35-4.35" stroke="#6c757d" stroke-width="2" stroke-linecap="round"/>
          </svg>
        </div>
        <input 
          type="text" 
          placeholder="택배 번호를 입력해 주세요." 
          class="search-input"
        />
      </div>
    </div>
  </header>
</template>

<script setup>
import { useRouter } from 'vue-router'

const router = useRouter()
const goBack = () => router.back()
const openSupport = () => console.log('고객센터 열기')
const openSettings = () => console.log('설정 열기')
const goNotifications = () => router.push({ name: 'Notifications' });

</script>


<style scoped>
.delivery-header {
  position: fixed;
  top: 0;
  left: 50%;
  transform: translateX(-50%);
  width: 100%;
  max-width: 390px;
  z-index: 50;
  background: linear-gradient(90deg, #4682B4, #6EC6CA);
  color: white;
  padding: 12px 16px 14px 16px;
  box-shadow: 0 4px 12px rgba(0,0,0,0.12);
}

.top-bar {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 8px;
}

.header-icons {
  display: flex;
  justify-content: flex-end;
  gap: 8px;
  margin-bottom: 8px;
}

/* 기존 이모지 아이콘 스타일 제거 (SVG 버튼으로 통일) */

.icon-btn {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 36px;
  height: 36px;
  border-radius: 50%;
  background-color: rgba(255,255,255,0.18);
  color: #ffffff;
  transition: background-color 0.2s ease, transform 0.2s ease;
}

.icon-btn:hover {
  background-color: rgba(255,255,255,0.28);
}

.header-content {
  margin-bottom: 8px;
}

.header-title {
  font-size: 18px;
  font-weight: 700;
  color: white !important;
  margin: 0;
  text-align: left;
  letter-spacing: -0.2px;
}

.search-container {
  margin-top: 6px;
}

.search-box {
  background-color: white;
  border-radius: 12px;
  padding: 12px 14px;
  display: flex;
  align-items: center;
  gap: 10px;
  box-shadow: 0 2px 12px rgba(0, 0, 0, 0.08);
  border: 1px solid rgba(0, 0, 0, 0.05);
}

.search-icon {
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
}

.search-input {
  flex: 1;
  border: none;
  outline: none;
  font-size: 14px;
  color: #495057;
  background: transparent;
  font-weight: 400;
}

.search-input::placeholder {
  color: #adb5bd;
  font-size: 14px;
  font-weight: 400;
}

.search-input:focus {
  outline: none;
}

/* blue-space 제거: 고정 헤더로 대체 */
</style>
