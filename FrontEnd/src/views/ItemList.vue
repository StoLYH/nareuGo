<template>
  <div class="page-container">
    <AppHeader
      :location="headerLocation"
      @edit="handleEdit"
      @search="handleSearch"
      @notification="handleNotification"
    />

    <main class="item-list-container">
      <!-- <h2 class="section-title">우리 동네 최신 상품</h2> -->
      <ItemCard v-for="item in items" :key="item.id" :item="item" />

      <div v-if="isLoading" class="status-indicator">
        <div class="spinner"></div>
        <p>상품을 불러오는 중...</p>
      </div>

      <div v-if="!hasMore && items.length > 0" class="status-indicator">
        <p>모든 상품을 불러왔습니다 ✨</p>
      </div>
    </main>

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

// State
const items = ref([])
const isLoading = ref(false)
const headerLocation = ref('')

// Time formatting function (remains the same)
const formatTimeAgo = (dateString) => {
  const now = new Date()
  let past = new Date(dateString)
  if (dateString.includes('T') && !dateString.includes('Z')) {
    past = new Date(dateString + 'Z')
  }

  const diffInMinutes = Math.floor((now - past) / (1000 * 60))

  if (diffInMinutes < 1) return '방금 전'
  if (diffInMinutes < 60) return `${diffInMinutes}분 전`
  if (diffInMinutes < 1440) return `${Math.floor(diffInMinutes / 60)}시간 전`
  return `${Math.floor(diffInMinutes / 1440)}일 전`
}

// Data loading function (remains the same)
const loadProductData = async () => {
  if (isLoading.value) return
  isLoading.value = true

  try {
    const userInfo = JSON.parse(localStorage.getItem('user'))
    if (!userInfo || !userInfo.userId) {
      console.error('사용자 정보가 없습니다.')
      return
    }

    const productList = await getProducts(userInfo.userId)
    items.value = productList.map(product => {
      const firstImage = product.imageUrls && product.imageUrls.length > 0 ? product.imageUrls[0] : null
      return {
        id: product.productId,
        title: product.title,
        location: `${product.siGunGu} ${product.eupMyeonDong}`,
        time: formatTimeAgo(product.createdAt),
        price: product.price,
        image: firstImage,
        allImages: product.imageUrls || [],
        description: product.description || "",
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
    // 헤더 위치를 첫 상품의 읍면동으로 표시 (없으면 시군구 또는 기본값)
    if (items.value.length > 0) {
      const first = items.value[0]
      const dong = first.eupMyeonDong || ''
      const apt = first.apartmentName || ''
      const composed = [dong, apt].filter(Boolean).join(' ')
      headerLocation.value = composed || first.siGunGu || '내 동네'
    } else {
      headerLocation.value = '내 동네'
    }
  } catch (error) {
    console.error('상품 목록 조회 실패:', error)
    items.value = []
  } finally {
    isLoading.value = false
  }
}

// Event handlers (remains the same)
const handleEdit = () => router.push('/item/register')
const handleSearch = () => router.push('/search')
const handleNotification = () => router.push('/notifications')
const handleNavigation = (tab) => {
  switch(tab) {
    case 'home': router.push('/items'); break
    case 'chat': router.push('/chat'); break
    case 'profile': router.push('/profile'); break
    default: console.log('Unknown navigation tab:', tab)
  }
}

// Lifecycle hook
onMounted(() => {
  loadProductData()
})
</script>

<style scoped>
.page-container {
  width: 100%;
  height: 100vh;
  display: flex;
  flex-direction: column;
  background: linear-gradient(180deg, #F5F9FC 0%, #FFFFFF 60%);
  font-family: 'Pretendard', sans-serif;
}

.item-list-container {
  flex: 1;
  overflow-y: auto;
  padding: 20px;
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(260px, 1fr));
  gap: 16px;
  scrollbar-width: none;
  -ms-overflow-style: none;
  background: linear-gradient(180deg, #F7FAFE 0%, #FFFFFF 65%);
  border-top: 1px solid #eef2f6;
}

.item-list-container::-webkit-scrollbar {
  display: none;
}

.section-title {
  grid-column: 1 / -1;
  margin: 4px 0 8px 0;
  font-size: 18px;
  font-weight: 700;
  color: #3a6b9a;
  letter-spacing: -0.2px;
}

.status-indicator {
  grid-column: 1 / -1;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 20px 0;
  gap: 8px;
  color: #a0a0a0;
  font-size: 14px;
}

.spinner {
  width: 24px;
  height: 24px;
  border: 2px solid #e0e0e0;
  border-top: 2px solid #2c3e50;
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}

</style>