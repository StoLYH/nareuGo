<template>
  <nav class="bottom-nav">
    <button 
      class="nav-item" 
      :class="{ active: activeTab === 'home' }"
      @click="$emit('navigate', 'home')"
    >
      <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
        <path d="M10 20v-6h4v6h5v-8h3L12 3 2 12h3v8z" fill="currentColor"/>
      </svg>
      <span>홈</span>
    </button>
    <button 
      class="nav-item" 
      :class="{ active: activeTab === 'chat' }"
      @click="$emit('navigate', 'chat')"
    >
      <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
        <path d="M20 2H4c-1.1 0-2 .9-2 2v12c0 1.1.9 2 2 2h14l4 4V4c0-1.1-.9-2-2-2zm-2 12H6v-2h12v2zm0-3H6V9h12v2zm0-3H6V6h12v2z" fill="currentColor"/>
      </svg>
      <span>채팅</span>
    </button>
    <button 
      class="nav-item" 
      :class="{ active: activeTab === 'profile' }"
      @click="$emit('navigate', 'profile')"
    >
      <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
        <path d="M12 12c2.21 0 4-1.79 4-4s-1.79-4-4-4-4 1.79-4 4 1.79 4 4 4zm0 2c-2.67 0-8 1.34-8 4v2h16v-2c0-2.66-5.33-4-8-4z" fill="currentColor"/>
      </svg>
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
  display: flex;
  justify-content: space-around;
  align-items: center;
  padding: 8px 0 10px 0;
  background-color: white;
}

.nav-item {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 2px;
  padding: 4px 12px;
  color: #999;
  transition: color 0.2s;
}

.nav-item.active {
  color: #000;
}

.nav-item span {
  font-size: 12px;
  font-weight: 400;
}
</style>
