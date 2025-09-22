<template>
  <div class="bottom-buttons">
    <button class="bottom-button verify" @click="openNeighborhoodAuth">
      <div class="icon-circle">
        <!-- Shield Check -->
        <svg width="22" height="22" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" aria-hidden="true">
          <path d="M12 2l7 3v6c0 5-3.5 9-7 11-3.5-2-7-6-7-11V5l7-3Z" fill="currentColor"/>
          <path d="M9.5 12.2l2 2 3.5-4" stroke="#fff" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"/>
        </svg>
      </div>
      <span>동네 인증하기</span>
    </button>
    
    <button class="bottom-button sales" @click="openSalesHistory">
      <div class="icon-circle">
        <!-- Receipt -->
        <svg width="22" height="22" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" aria-hidden="true">
          <path d="M7 3h10a2 2 0 0 1 2 2v14l-2-1-2 1-2-1-2 1-2-1-2 1V5a2 2 0 0 1 2-2Z" fill="currentColor"/>
          <path d="M9 8h6M9 12h6" stroke="#fff" stroke-width="1.6" stroke-linecap="round"/>
        </svg>
      </div>
      <span>판매내역</span>
    </button>
    
    <button class="bottom-button purchase" @click="openPurchaseHistory">
      <div class="icon-circle">
        <!-- Wallet -->
        <svg width="22" height="22" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" aria-hidden="true">
          <path d="M4 7.5C4 6.12 5.12 5 6.5 5H17a2 2 0 1 1 0 4H6.5A2.5 2.5 0 0 1 4 6.5v1Z" fill="currentColor"/>
          <rect x="3" y="8" width="18" height="10" rx="2" fill="currentColor"/>
          <circle cx="17" cy="13" r="1.5" fill="#fff"/>
        </svg>
      </div>
      <span>구매내역</span>
    </button>
    
    <button class="bottom-button wishlist" @click="openWishlistBottom">
      <div class="icon-circle">
        <!-- Bookmark Heart -->
        <svg width="22" height="22" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" aria-hidden="true">
          <path d="M6 3h12a1 1 0 0 1 1 1v17l-7-3-7 3V4a1 1 0 0 1 1-1Z" fill="currentColor"/>
          <path d="M12 11.8s-2.2-1.4-2.2-3A1.8 1.8 0 0 1 12 7.2a1.8 1.8 0 0 1 2.2 1.6c0 1.6-2.2 3-2.2 3Z" fill="#fff"/>
        </svg>
      </div>
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

.icon-circle {
  width: 36px;
  height: 36px;
  border-radius: 50%;
  display: inline-flex;
  align-items: center;
  justify-content: center;
  margin-right: 14px;
}

/* 색상 차별화로 네비게이션 아이콘과 구분감 제공 */
.bottom-button.verify .icon-circle { background: rgba(72, 187, 120, 0.15); }
.bottom-button.verify .icon-circle svg { color: #2F855A; }

.bottom-button.sales .icon-circle { background: rgba(246, 173, 85, 0.18); }
.bottom-button.sales .icon-circle svg { color: #C05621; }

.bottom-button.purchase .icon-circle { background: rgba(99, 179, 237, 0.18); }
.bottom-button.purchase .icon-circle svg { color: #2B6CB0; }

.bottom-button.wishlist .icon-circle { background: rgba(245, 101, 101, 0.16); }
.bottom-button.wishlist .icon-circle svg { color: #C53030; }

.bottom-button span {
  font-size: 16px;
  color: var(--secondary, #2c3e50);
  font-weight: 600;
}

.bottom-button:hover svg {
  color: var(--primary, #4682b4);
}
</style>