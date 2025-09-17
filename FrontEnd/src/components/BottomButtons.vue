<template>
  <div class="bottom-buttons">
    <button class="bottom-button" @click="openNeighborhoodAuth">
      <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M12 2C8.13 2 5 5.13 5 9C5 14.25 12 22 12 22S19 14.25 19 9C19 5.13 15.87 2 12 2ZM12 11.5C10.62 11.5 9.5 10.38 9.5 9C9.5 7.62 10.62 6.5 12 6.5C13.38 6.5 14.5 7.62 14.5 9C14.5 10.38 13.38 11.5 12 11.5Z" stroke="currentColor" stroke-width="1.5" fill="none"/>
      </svg>
      <span>동네 인증하기</span>
    </button>
    
    <button class="bottom-button" @click="openSalesHistory">
      <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M14 2H6C4.9 2 4 2.9 4 4V20C4 21.1 4.89 22 5.99 22H18C19.1 22 20 21.1 20 20V8L14 2ZM18 20H6V4H13V9H18V20Z" stroke="currentColor" stroke-width="1.5" fill="none"/>
      </svg>
      <span>판매내역</span>
    </button>
    
    <button class="bottom-button" @click="openPurchaseHistory">
      <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M7 4V2C7 1.45 7.45 1 8 1H16C16.55 1 17 1.45 17 2V4H20C20.55 4 21 4.45 21 5S20.55 6 20 6H19V19C19 20.1 18.1 21 17 21H7C5.9 21 5 20.1 5 19V6H4C3.45 6 3 5.55 3 5S3.45 4 4 4H7ZM9 3V4H15V3H9ZM7 6V19H17V6H7ZM9 8H15V10H9V8ZM9 12H15V14H9V12Z" stroke="currentColor" stroke-width="1.5" fill="none"/>
      </svg>
      <span>구매내역</span>
    </button>
    
    <button class="bottom-button" @click="openWishlistBottom">
      <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M12 21.35L10.55 20.03C5.4 15.36 2 12.27 2 8.5C2 5.41 4.42 3 7.5 3C9.24 3 10.91 3.81 12 5.08C13.09 3.81 14.76 3 16.5 3C19.58 3 22 5.41 22 8.5C22 12.27 18.6 15.36 13.45 20.03L12 21.35Z" stroke="currentColor" stroke-width="1.5" fill="none"/>
      </svg>
      <span>관심목록</span>
    </button>
  </div>
</template>

<script setup>
import { useRouter } from 'vue-router'

const router = useRouter()

const openNeighborhoodAuth = () => {
  router.push('/neighborhood-verification')
};

const openSalesHistory = () => {
  console.log('판매내역 열기');
};

const openPurchaseHistory = () => {
  console.log('구매내역 열기');
};

const openWishlistBottom = () => {
  console.log('관심목록 열기');
};
</script>

<style scoped>
.bottom-buttons {
  padding: 0 20px 20px 20px;
}

.bottom-button {
  display: flex;
  align-items: center;
  width: 100%;
  padding: 16px 20px;
  border: none;
  background: white;
  cursor: pointer;
  transition: all 0.2s ease;
  border-radius: 12px;
  margin-bottom: 12px;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
}

.bottom-button:hover {
  background-color: #f8f9fa;
}

.bottom-button svg {
  color: #333;
  margin-right: 16px;
  flex-shrink: 0;
}

.bottom-button span {
  font-size: 16px;
  color: #333;
  font-weight: 500;
}
</style>