<template>
  <div class="item-card" @click="handleItemClick">
    <div class="item-image-wrapper">
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
    </div>
    <div class="item-info">
      <div class="title-row">
        <h3 class="item-title">{{ item.title }}</h3>
        <span
          v-if="isForSale(item.status)"
          class="status-inline"
          aria-label="판매중"
        >
          판매중
        </span>
      </div>
      <p class="item-location">{{ item.location }} · {{ item.time }}</p>
      <div class="card-footer">
        <span class="item-price-badge">{{ item.price.toLocaleString() }}원</span>
      </div>
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

// FOR_SALE 상태 판단 (백엔드 표기 변형 대응)
const isForSale = (status) => {
  const s = (status || '').toString().toLowerCase()
  return s === 'for_sale' || s === 'for-sale' || s === 'forsale' || s === 'available'
}
</script>

<style scoped>
/* 카드 전체 */
.item-card {
  display: flex;
  align-items: flex-start;
  padding: 18px;
  margin-bottom: 14px;
  border-radius: 12px;
  background: #fff;
  box-shadow: 0 2px 8px rgba(0,0,0,0.06);
  border: 1px solid #e6edf5;
  gap: 14px;
  cursor: pointer;
  transition: transform 0.2s ease, box-shadow 0.3s ease, background-color 0.3s;
}

.item-card:hover {
  transform: translateY(-3px);
  box-shadow: 0 6px 18px rgba(70,130,180,0.22);
  border-color: #b9d1e6;
  background-color: #f9fbfd;
}

/* 이미지 래퍼 */
.item-image-wrapper {
  flex-shrink: 0;
  position: relative;
}

.item-image {
  width: 120px;
  height: 120px;
  border-radius: 12px;
  overflow: hidden;
  background: #f0f4f8;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: transform 0.3s ease;
  border: 1px solid #e6edf5;
}

.item-card:hover .item-image {
  transform: scale(1.05);
}

.item-image img {
  width: 100%;
  height: 100%;
  object-fit: cover;
  display: block;
}

/* 이미지 없음 */
.no-image {
  width: 100%;
  height: 100%;
  font-size: 13px;
  color: #999;
  display: flex;
  align-items: center;
  justify-content: center;
  background: repeating-linear-gradient(
    45deg,
    #f0f0f0,
    #f0f0f0 10px,
    #e6e6e6 10px,
    #e6e6e6 20px
  );
}

/* 정보 영역 */
.item-info {
  flex: 1;
  display: flex;
  flex-direction: column;
  justify-content: center;
  gap: 8px;
  min-width: 0;
}

/* 제목 */
.item-title {
  font-size: 17px;
  font-weight: 600;
  color: #2c3e50;
  margin: 0 0 6px 0;
  line-height: 1.4;
  display: -webkit-box;
  -webkit-line-clamp: 2;
  line-clamp: 2;
  -webkit-box-orient: vertical;
  overflow: hidden;
  transition: color 0.2s ease;
}

.title-row {
  display: flex;
  align-items: flex-start;
  gap: 8px;
}

.status-inline {
  margin-left: auto;
  flex-shrink: 0;
  display: inline-flex;
  align-items: center;
  padding: 2px 8px;
  height: 22px;
  border-radius: 9999px;
  font-size: 12px;
  font-weight: 700;
  color: rgb(242, 139, 108); /* 코랄 포인트 */
  background: rgba(249, 248, 246, 0.12);
  border: 1px solid rgba(209, 77, 114, 0.24);
}



/* 위치 */
.item-location {
  font-size: 14px;
  color: #6f7d8a;
  margin: 2px 0 6px 0;
  letter-spacing: -0.1px;
}

/* 카드 푸터 */
.card-footer {
  display: flex;
  align-items: center;
  justify-content: flex-end;
  border-top: 1px solid #f0f4f8;
  margin-top: 4px;
  padding-top: 8px;
}

.item-price-badge {
  display: inline-flex;
  align-items: center;
  padding: 10px 16px;
  border-radius: 12px;
  font-size: 16px;
  font-weight: 800;
  color: #4682b4; /* Option C: soft indigo */
  /* background: rgba(220, 226, 246, 0.12); */
  /* border: 1px solid rgba(103, 136, 235, 0.24); */
  /* box-shadow: 0 2px 8px rgba(0,0,0,0.06); */
}
</style>