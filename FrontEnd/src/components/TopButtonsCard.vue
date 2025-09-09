<template>
  <div class="top-buttons-card">
    <button class="top-button" @click="openDeliveryStatus">
      <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M20 8H4C2.9 8 2 8.9 2 10V20C2 21.1 2.9 22 4 22H20C21.1 22 22 21.1 22 20V10C22 8.9 21.1 8 20 8ZM20 20H4V10H20V20ZM12 12C13.1 12 14 12.9 14 14C14 15.1 13.1 16 12 16C10.9 16 10 15.1 10 14C10 12.9 10.9 12 12 12Z" stroke="currentColor" stroke-width="1.5" fill="none"/>
      </svg>
      <span>택배 현황</span>
    </button>
    
    <div class="divider"></div>
    
    <button class="top-button" @click="openWishlist">
      <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M12 21.35L10.55 20.03C5.4 15.36 2 12.27 2 8.5C2 5.41 4.42 3 7.5 3C9.24 3 10.91 3.81 12 5.08C13.09 3.81 14.76 3 16.5 3C19.58 3 22 5.41 22 8.5C22 12.27 18.6 15.36 13.45 20.03L12 21.35Z" stroke="currentColor" stroke-width="1.5" fill="none"/>
      </svg>
      <span>관심 목록</span>
    </button>
    
    <div class="divider"></div>
    
    <button class="top-button" @click="openRecentPosts">
      <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M12 2C6.48 2 2 6.48 2 12S6.48 22 12 22 22 17.52 22 12 17.52 2 12 2ZM12 20C7.59 20 4 16.41 4 12S7.59 4 12 4 20 7.59 20 12 16.41 20 12 20ZM12.5 7H11V13L16.25 16.15L17 14.92L12.5 12.25V7Z" stroke="currentColor" stroke-width="1.5" fill="none"/>
      </svg>
      <span>최근 본 글</span>
    </button>
  </div>
</template>

<script setup>
const openDeliveryStatus = () => {
  console.log('택배 현황 열기');
};

const openWishlist = () => {
  console.log('관심 목록 열기');
};

const openRecentPosts = () => {
  console.log('최근 본 글 열기');
};
</script>

<style scoped>
.top-buttons-card {
  display: flex;
  background-color: white;
  border-radius: 12px;
  margin: 0 20px 20px 20px;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
  overflow: hidden;
}

.top-button {
  flex: 1;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 20px 12px;
  border: none;
  background: none;
  cursor: pointer;
  transition: background-color 0.2s ease;
}

.top-button:hover {
  background-color: #f8f9fa;
}

.top-button svg {
  color: #333;
  margin-bottom: 8px;
}

.top-button span {
  font-size: 14px;
  color: #333;
  font-weight: 500;
}

.divider {
  width: 1px;
  background-color: #f0f0f0;
  margin: 16px 0;
}
</style>