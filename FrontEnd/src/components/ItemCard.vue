<template>
  <div class="item-card" @click="handleItemClick">
    <div class="item-image">
      <img 
        v-if="item.image" 
        :src="item.image" 
        :alt="item.title" 
        @error="handleImageError"
        @load="handleImageLoad"
      />
      <div v-else class="no-image">
        <span>이미지 없음</span>
      </div>
    </div>
    <div class="item-info">
      <h3 class="item-title">{{ item.title }}</h3>
      <p class="item-location">{{ item.location }} · {{ item.time }}</p>
      <p class="item-price">{{ item.price.toLocaleString() }}원</p>
    </div>
  </div>
</template>

<script setup>
import { useRouter } from 'vue-router'

const router = useRouter()

const props = defineProps({
  item: {
    type: Object,
    required: true
  }
})

const handleItemClick = () => {
  // 상품 상세 페이지로 이동하면서 상품 정보를 query로 전달
  console.log('ItemCard에서 전달할 상품 정보:', props.item)
  
  // sessionStorage를 사용해서 데이터 전달 (더 안정적)
  sessionStorage.setItem(`item_${props.item.id}`, JSON.stringify(props.item))
  
  router.push(`/item/${props.item.id}`)
}

const handleImageError = (event) => {
  console.error('이미지 로드 실패:', props.item.image, event)
  console.error('에러 상세:', event.target.src, event.target.naturalWidth, event.target.naturalHeight)
  
  // 이미지 URL을 새 탭에서 직접 열어보기 (디버깅용)
  console.log('이미지 URL 직접 테스트:', props.item.image)
}

const handleImageLoad = () => {
  console.log('이미지 로드 성공:', props.item.image)
}
</script>

<style scoped>
.item-card {
  display: flex;
  padding: 16px 0;
  border-bottom: 1px solid #f0f0f0;
  gap: 12px;
  cursor: pointer;
  transition: background-color 0.2s;
}

.item-card:hover {
  background-color: #f8f8f8;
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
  line-clamp: 2;
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

.no-image {
  width: 100%;
  height: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
  background-color: #f0f0f0;
  color: #999;
  font-size: 12px;
}
</style>
