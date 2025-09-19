<template>
  <nav class="bottom-nav">
    <button
      class="nav-item"
      :class="{ active: activeTab === 'home' }"
      @click="$emit('navigate', 'home')"
    >
      <div class="icon-wrapper">
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
          <path d="M10 20v-6h4v6h5v-8h3L12 3 2 12h3v8z" fill="currentColor"/>
        </svg>
        <div class="active-line" v-if="activeTab === 'home'"></div>
      </div>
      <span>홈</span>
    </button>
    <button
      class="nav-item"
      :class="{ active: activeTab === 'chat' }"
      @click="$emit('navigate', 'chat')"
    >
      <div class="icon-wrapper">
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
          <path d="M20 2H4c-1.1 0-2 .9-2 2v12c0 1.1.9 2 2 2h14l4 4V4c0-1.1-.9-2-2-2zm-2 12H6v-2h12v2zm0-3H6V9h12v2zm0-3H6V6h12v2z" fill="currentColor"/>
        </svg>
        <div class="active-line" v-if="activeTab === 'chat'"></div>
      </div>
      <span>채팅</span>
    </button>
    <button
      class="nav-item"
      :class="{ active: activeTab === 'profile' }"
      @click="$emit('navigate', 'profile')"
    >
      <div class="icon-wrapper">
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
          <path d="M12 12c2.21 0 4-1.79 4-4s-1.79-4-4-4-4 1.79-4 4 1.79 4 4 4zm0 2c-2.67 0-8 1.34-8 4v2h16v-2c0-2.66-5.33-4-8-4z" fill="currentColor"/>
        </svg>
        <div class="active-line" v-if="activeTab === 'profile'"></div>
      </div>
      <span>마이 페이지</span>
    </button>
  </nav>
</template>

<script setup>
defineProps({
  activeTab: {
    type: String,
    default: 'home'
  }
})
defineEmits(['navigate'])
</script>

<style scoped>
.bottom-nav {
  position: fixed;
  left: 50%;
  transform: translateX(-50%);
  width: 100%;
  max-width: 390px;
  bottom: 0;
  z-index: 50;
  display: flex;
  justify-content: space-around;
  align-items: center;
  padding: 8px 0 calc(8px + env(safe-area-inset-bottom));
  background-color: #fff;
  border-top: 1px solid #e0e0e0;
  box-shadow: 0 -1px 4px rgba(0,0,0,0.05);
}

.nav-item {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 4px;
  padding: 6px 12px;
  border-radius: 12px;
  color: #999;
  transition: all 0.2s ease;
}

.nav-item.active {
  color: #4682b4; /* 강조 색상 */
  background-color: rgba(70,130,180,0.1); /* 활성 배경 */
}

.nav-item span {
  font-size: 12px;
  font-weight: 500;
}

.icon-wrapper {
  display: flex;
  flex-direction: column;
  align-items: center;
}

.icon-wrapper svg {
  width: 28px;
  height: 28px;
  transition: transform 0.2s ease;
}

.nav-item.active .icon-wrapper svg {
  transform: scale(1.2); /* 활성 아이콘 살짝 확대 */
}

</style>