<template>
  <div class="start-button-container">
    <button class="start-button" @click="startMyNarGo">
      나르고 시작하기
    </button>
  </div>
</template>

<script setup>
import { useRouter } from 'vue-router';

const router = useRouter();

const startMyNarGo = () => {
  console.log('나르고 시작하기 - ItemList로 이동');
  router.push('/items');
};
</script>

<style scoped>
.start-button-container {
  padding: 20px;
  margin-bottom: 32px;
}

.start-button {
  width: 100%;
  padding: 16px 24px;
  background: linear-gradient(135deg, #007AFF 0%, #0051D5 100%);
  color: white;
  border: none;
  border-radius: 12px;
  font-size: 16px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s ease;
  box-shadow: 0 2px 8px rgba(0, 122, 255, 0.3);
}

.start-button:hover {
  background: linear-gradient(135deg, #0056CC 0%, #003D99 100%);
  box-shadow: 0 4px 12px rgba(0, 122, 255, 0.4);
  transform: translateY(-1px);
}

.start-button:active {
  transform: translateY(0);
  box-shadow: 0 2px 8px rgba(0, 122, 255, 0.3);
}
</style>