<template>
  <div class="item-card">
    <div class="item-image">
      <img :src="item.image" :alt="item.title" />
    </div>
    <div class="item-info">
      <h3 class="item-title">{{ item.title }}</h3>
      <p class="item-location">{{ item.location }} · {{ item.time }}</p>
      <p class="item-price">{{ item.price.toLocaleString() }}원</p>
    </div>
  </div>
</template>

<script setup>
defineProps({
  item: {
    type: Object,
    required: true
  }
})
</script>

<style scoped>
.item-card {
  display: flex;
  padding: 16px 0;
  border-bottom: 1px solid #f0f0f0;
  gap: 12px;
}

.item-image {
  width: 120px;
  height: 120px;
  border-radius: 8px;
  overflow: hidden;
  background-color: #f8f8f8;
  flex-shrink: 0;
}

.item-image img {
  width: 100%;
  height: 100%;
  object-fit: cover;
}

.item-info {
  flex: 1;
  display: flex;
  flex-direction: column;
  justify-content: flex-start;
  min-width: 0;
  padding-top: 2px;
}

.item-title {
  font-size: 16px;
  font-weight: 400;
  color: #000;
  margin: 0 0 4px 0;
  line-height: 1.3;
  letter-spacing: -0.01em;
  display: -webkit-box;
  -webkit-line-clamp: 2;
  -webkit-box-orient: vertical;
  overflow: hidden;
}

.item-location {
  font-size: 13px;
  color: #666;
  margin: 0 0 8px 0;
  font-weight: 400;
  line-height: 1.3;
  letter-spacing: -0.005em;
}

.item-price {
  font-size: 16px;
  font-weight: 600;
  color: #000;
  margin: 0;
  letter-spacing: -0.01em;
}
</style>
