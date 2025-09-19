<template>
  <div class="top-buttons-card">
    <button class="top-button delivery" @click="openDeliveryStatus">
      <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M20 8H4c-1.1 0-2 .9-2 2v10c0 1.1.9 2 2 2h16c1.1 0 2-.9 2-2V10c0-1.1-.9-2-2-2Zm0 12H4V10h16v10ZM12 12.5c1.38 0 2.5 1.12 2.5 2.5S13.38 17.5 12 17.5 9.5 16.38 9.5 15 10.62 12.5 12 12.5Z" fill="currentColor"/>
      </svg>
      <span>택배 현황</span>
    </button>
    
    <div class="divider"></div>
    
    <button class="top-button wishlist" @click="openWishlist">
      <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M12 21.35 10.55 20.03C5.4 15.36 2 12.27 2 8.5 2 5.41 4.42 3 7.5 3c1.74 0 3.41.81 4.5 2.08C13.09 3.81 14.76 3 16.5 3 19.58 3 22 5.41 22 8.5c0 3.77-3.4 6.86-8.55 11.53L12 21.35Z" fill="currentColor"/>
      </svg>
      <span>관심 목록</span>
    </button>
    
    <div class="divider"></div>
    
    <button class="top-button recent" @click="openRecentPosts">
      <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2Zm1 11h-3V7h2v4l3.5 2.1-1 1.7L13 13Z" fill="currentColor"/>
      </svg>
      <span>최근 본 글</span>
    </button>
  </div>
</template>

<script setup>
import { useRouter } from 'vue-router';

const router = useRouter();

const openDeliveryStatus = () => {
  console.log('택배 현황 열기');
  router.push('/delivery-status');
};

const openWishlist = () => {
  console.log('관심 목록 열기');
  router.push('/items?filter=favorites');
};

const openRecentPosts = () => {
  console.log('최근 본 글 열기');
  router.push('/items?filter=recent');
};
</script>

<style scoped>
.top-buttons-card {
  display: flex;
  background-color: var(--surface, #fff);
  border-radius: 12px;
  margin: 0 20px 20px 20px;
  box-shadow: 0 2px 10px rgba(0, 0, 0, 0.06);
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
  background-color: var(--surface-2, #f3f6fa);
}

.top-button svg {
  color: var(--muted, #6b7280);
  margin-bottom: 8px;
}

.top-button span {
  font-size: 14px;
  color: var(--secondary, #2c3e50);
  font-weight: 600;
}

/* Per-button accent colors */
.top-button.delivery svg { color: var(--primary, #4682b4); }
.top-button.wishlist svg { color: #ff6b9a; }
.top-button.recent svg { color: #f59e0b; }

/* Hover states darken within the same family */
.top-button.delivery:hover svg { color: var(--primary-600, #3a6b9a); }
.top-button.wishlist:hover svg { color: #e85f8a; }
.top-button.recent:hover svg { color: #d97706; }

.divider {
  width: 1px;
  background-color: rgba(0,0,0,0.06);
  margin: 16px 0;
}

/* Responsive spacing */
@media (max-width: 400px) {
  .top-buttons-card { margin: 0 12px 16px 12px; }
  .top-button { padding: 16px 8px; }
  .top-button svg { width: 22px; height: 22px; }
  .top-button span { font-size: 13px; }
}
</style>