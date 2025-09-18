<template>
  <div class="first-page">
    <!-- 헤더 -->
    <AppHeader 
      location="삼성동"
      @edit="handleEdit"
      @search="handleSearch"
      @notification="handleNotification"
    />

    <!-- 상품 목록 -->
    <main class="item-list">
      <ItemCard v-for="item in items" :key="item.id" :item="item" />
      
      <!-- 로딩 인디케이터 -->
      <div v-if="isLoading" class="loading-indicator">
        <div class="spinner"></div>
        <p>상품을 불러오는 중...</p>
      </div>
      
      <!-- 더 이상 로드할 데이터가 없을 때 -->
      <div v-if="!hasMore && items.length > 0" class="no-more-data">
        <p>모든 상품을 불러왔습니다</p>
      </div>
    </main>

    <!-- 하단 네비게이션 -->
    <BottomNavigation 
      active-tab="home"
      @navigate="handleNavigation"
    />

  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, nextTick, watch } from 'vue'
import { useRouter } from 'vue-router'
import AppHeader from '../components/AppHeader.vue'
import BottomNavigation from '../components/BottomNavigation.vue'
import ItemCard from '../components/ItemCard.vue'
import { getProducts } from '@/api/product/product.js'

const router = useRouter()

// 상품 목록 관련 상태
const items = ref([])
const isLoading = ref(false)

// 시간 포맷팅 함수 (UTC/KST 시간대 고려)
const formatTimeAgo = (dateString) => {
  const now = new Date()
  
  // 백엔드에서 받은 시간을 UTC로 파싱 후 로컬 시간대로 변환
  let past
  if (dateString.includes('T') && dateString.includes('Z')) {
    // 이미 UTC 형식인 경우 (예: 2025-09-17T04:49:39Z)
    past = new Date(dateString)
  } else if (dateString.includes('T')) {
    // ISO 형식이지만 Z가 없는 경우 UTC로 가정
    past = new Date(dateString + 'Z')
  } else {
    // 다른 형식인 경우 그대로 파싱
    past = new Date(dateString)
  }
  
  const diffInMinutes = Math.floor((now - past) / (1000 * 60))

  // 디버깅용 로그
  console.log(`시간 계산: now=${now.toISOString()}, past=${past.toISOString()}, diff=${diffInMinutes}분`)

  if (diffInMinutes < 1) return '방금 전'
  if (diffInMinutes < 60) return `${diffInMinutes}분 전`

  const diffInHours = Math.floor(diffInMinutes / 60)
  if (diffInHours < 24) return `${diffInHours}시간 전`

  const diffInDays = Math.floor(diffInHours / 24)
  return `${diffInDays}일 전`
}

// 실제 데이터 로드
const loadProductData = async () => {
  if (isLoading.value) return

  isLoading.value = true

  try {
    // 로컬 스토리지에서 사용자 정보 가져오기
    const userInfo = JSON.parse(localStorage.getItem('user'))
    if (!userInfo || !userInfo.userId) {
      console.error('사용자 정보가 없습니다.')
      return
    }

    // API 호출하여 상품 목록 조회
    const productList = await getProducts(userInfo.userId)
    console.log('상품 목록 조회 성공:', productList)

    // 백엔드 데이터를 프론트엔드 형식으로 변환
    items.value = productList.map(product => {
      const firstImage = product.imageUrls && product.imageUrls.length > 0 ? product.imageUrls[0] : null
      console.log(`상품 ${product.productId} 이미지 URL:`, firstImage)

      return {
        id: product.productId,
        title: product.title,
        location: `${product.siGunGu} ${product.eupMyeonDong}`, // 시군구 + 읍면동
        time: formatTimeAgo(product.createdAt),
        price: product.price,
        image: firstImage, // 첫 번째 이미지만 표시용
        // 상세 페이지를 위한 추가 정보
        allImages: product.imageUrls || [], // 모든 이미지
        description: product.description || "", // 상품 설명
        sellerId: product.sellerId,
        siDo: product.siDo,
        siGunGu: product.siGunGu,
        eupMyeonDong: product.eupMyeonDong,
        apartmentName: product.apartmentName,
        status: product.status,
        createdAt: product.createdAt,
        updatedAt: product.updatedAt
      }
    })

  } catch (error) {
    console.error('상품 목록 조회 실패:', error)
    // 에러 발생 시 빈 배열로 설정
    items.value = []
  } finally {
    isLoading.value = false
  }
}

// 이벤트 핸들러들
const handleEdit = () => {
  router.push('/item/register')
}

const handleSearch = () => {
  router.push('/search')
}

const handleNotification = () => {
  router.push('/notifications')
}


const handleNavigation = (tab) => {
  switch(tab) {
    case 'home':
      router.push('/items')
      break
    case 'chat':
      router.push('/chat')
      break
    case 'profile':
      router.push('/profile')
      break
    default:
      console.log('Unknown navigation tab:', tab)
  }
}

// 컴포넌트 마운트
onMounted(() => {
  loadProductData()
})
</script>

<style scoped>
.first-page {
  width: 100%;
  height: 100vh;
  display: flex;
  flex-direction: column;
  background-color: white;
}


/* 상품 목록 */
.item-list {
  flex: 1;
  overflow-y: auto;
  padding: 0 20px;
  /* 스크롤바 숨기기 */
  scrollbar-width: none; /* Firefox */
  -ms-overflow-style: none; /* IE and Edge */
}

.item-list::-webkit-scrollbar {
  display: none; /* Chrome, Safari, Opera */
}


/* 로딩 인디케이터 */
.loading-indicator {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 20px 0;
  gap: 12px;
}

.spinner {
  width: 24px;
  height: 24px;
  border: 2px solid #f3f3f3;
  border-top: 2px solid var(--main);
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}

.loading-indicator p {
  font-size: 14px;
  color: #999;
  margin: 0;
}

/* 더 이상 데이터가 없을 때 */
.no-more-data {
  display: flex;
  justify-content: center;
  align-items: center;
  padding: 20px 0;
}

.no-more-data p {
  font-size: 14px;
  color: #ccc;
  margin: 0;
}


</style>
