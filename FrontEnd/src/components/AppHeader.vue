<template>
  <header class="header">
    <div class="location">{{ location }}</div>
    <div class="header-icons">
      <button class="icon-btn" @click="$emit('edit')">
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
          <path d="M3 17.25V21h3.75L17.81 9.94l-3.75-3.75L3 17.25zM20.71 7.04c.39-.39.39-1.02 0-1.41l-2.34-2.34c-.39-.39-1.02-.39-1.41 0l-1.83 1.83 3.75 3.75 1.83-1.83z" fill="currentColor"/>
        </svg>
      </button>
      <button class="icon-btn" @click="$emit('search')">
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
          <path d="M15.5 14h-.79l-.28-.27C15.41 12.59 16 11.11 16 9.5 16 5.91 13.09 3 9.5 3S3 5.91 3 9.5 5.91 16 9.5 16c1.61 0 3.09-.59 4.23-1.57l.27.28v.79l5 4.99L20.49 19l-4.99-5zm-6 0C7.01 14 5 11.99 5 9.5S7.01 5 9.5 5 14 7.01 14 9.5 11.99 14 9.5 14z" fill="currentColor"/>
        </svg>
      </button>
      <button class="icon-btn" @click="$emit('notification')">
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
          <path d="M12 22c1.1 0 2-.9 2-2h-4c0 1.1.89 2 2 2zm6-6v-5c0-3.07-1.64-5.64-4.5-6.32V4c0-.83-.67-1.5-1.5-1.5s-1.5.67-1.5 1.5v.68C7.63 5.36 6 7.92 6 11v5l-2 2v1h16v-1l-2-2z" fill="currentColor"/>
        </svg>
      </button>
    </div>
  </header>
</template>

<script setup>
defineProps({
  location: {
    type: String,
    default: 'OO마을 12단지'
  }
})

defineEmits(['edit', 'search', 'notification'])
</script>

<style scoped>
.header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 16px 20px;
  background-color: white;
}

.location {
  font-size: 18px;
  font-weight: 600;
  color: #333;
}

.header-icons {
  display: flex;
  gap: 16px;
}

.icon-btn {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 40px;
  height: 40px;
  border-radius: 50%;
  background-color: #f8f8f8;
  color: #666;
  transition: background-color 0.2s;
}

.icon-btn:hover {
  background-color: #f0f0f0;
}
</style>
