<template>
  <header class="delivery-header">
    <!-- 상단 아이콘들 -->
    <div class="header-icons">
      <div class="icon">🎧</div>
      <div class="icon">⚙️</div>
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
    
    <!-- 추가 파란색 공간 -->
    <div class="blue-space"></div>
  </header>
</template>

<script setup>
// 헤더 컴포넌트
</script>

<style scoped>
.delivery-header {
  background: #007bff !important;
  padding: 20px;
  padding-top: 50px;
  color: white;
  position: relative;
  min-height: 320px;
  width: 100%;
}

.header-icons {
  display: flex;
  justify-content: flex-end;
  gap: 20px;
  margin-bottom: 30px;
}

.icon {
  font-size: 22px;
  cursor: pointer;
  transition: opacity 0.3s ease;
  filter: brightness(0) invert(1);
}

.icon:hover {
  opacity: 0.8;
}

.header-content {
  margin-bottom: 30px;
}

.header-title {
  font-size: 22px;
  font-weight: 700;
  color: white !important;
  margin: 0;
  text-align: left;
  letter-spacing: -0.3px;
}

.search-container {
  margin-top: 20px;
}

.search-box {
  background-color: white;
  border-radius: 16px;
  padding: 18px 20px;
  display: flex;
  align-items: center;
  gap: 16px;
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.1);
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
  font-size: 16px;
  color: #495057;
  background: transparent;
  font-weight: 400;
}

.search-input::placeholder {
  color: #adb5bd;
  font-size: 16px;
  font-weight: 400;
}

.search-input:focus {
  outline: none;
}

.blue-space {
  height: 60px;
  background: #007bff !important;
  margin: 0 -20px -20px -20px;
}
</style>
