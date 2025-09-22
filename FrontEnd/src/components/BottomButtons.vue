<template>
  <div class="bottom-buttons">
    <button class="bottom-button" @click="openNeighborhoodAuth">
      <!-- Location Pin (solid) -->
      <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor" xmlns="http://www.w3.org/2000/svg" aria-hidden="true">
        <path d="M12 2C8.14 2 5 5.14 5 9c0 5.25 7 13 7 13s7-7.75 7-13c0-3.86-3.14-7-7-7Zm0 9.5A2.5 2.5 0 1 1 12 6.5a2.5 2.5 0 0 1 0 5Z"/>
      </svg>
      <span>동네 인증하기</span>
    </button>
    
    <button class="bottom-button" @click="openSalesHistory">
      <!-- Sell Tag (solid) -->
      <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor" xmlns="http://www.w3.org/2000/svg" aria-hidden="true">
        <path d="M21.41 11.58 12.42 2.59A2.004 2.004 0 0 0 11 2H6c-1.1 0-2 .9-2 2v5c0 .53.21 1.04.59 1.41l8.99 8.99c.78.78 2.05.78 2.83 0l4-4c.79-.78.79-2.05 0-2.83ZM7.5 7.5A1.5 1.5 0 1 1 9 6a1.5 1.5 0 0 1-1.5 1.5Z"/>
      </svg>
      <span>판매내역</span>
    </button>
    
    <button class="bottom-button" @click="openPurchaseHistory">
      <!-- Shopping Bag (solid) -->
      <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor" xmlns="http://www.w3.org/2000/svg" aria-hidden="true">
        <path d="M18 6h-2.25A3.75 3.75 0 0 0 12 2.25 3.75 3.75 0 0 0 8.25 6H6c-1.1 0-2 .9-2 2v9c0 1.66 1.34 3 3 3h10c1.66 0 3-1.34 3-3V8c0-1.1-.9-2-2-2Zm-6-1.5c1.24 0 2.25 1.01 2.25 2.25H9.75C9.75 5.51 10.76 4.5 12 4.5Z"/>
      </svg>
      <span>구매내역</span>
    </button>
    
    <button class="bottom-button" @click="openWishlistBottom">
      <!-- Heart (solid) -->
      <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor" xmlns="http://www.w3.org/2000/svg" aria-hidden="true">
        <path d="M12.1 21.35c-.07.05-.13.08-.2.08s-.13-.03-.2-.08C7.14 17.24 4 14.39 4 10.9 4 8.5 5.9 6.6 8.3 6.6c1.25 0 2.45.57 3.2 1.49.75-.92 1.95-1.49 3.2-1.49 2.4 0 4.3 1.9 4.3 4.3 0 3.49-3.14 6.34-6.9 10.45Z"/>
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
  background: var(--surface, #fff);
  cursor: pointer;
  transition: all 0.2s ease;
  border-radius: 12px;
  margin-bottom: 12px;
  box-shadow: 0 2px 10px rgba(0, 0, 0, 0.06);
}

.bottom-button:hover {
  background-color: var(--surface-2, #f3f6fa);
  transform: translateY(-1px);
}

.bottom-button svg {
  color: var(--muted, #6b7280);
  margin-right: 16px;
  flex-shrink: 0;
}

.bottom-button span {
  font-size: 16px;
  color: var(--secondary, #2c3e50);
  font-weight: 600;
}

.bottom-button:hover svg {
  color: var(--primary, #4682b4);
}
</style>