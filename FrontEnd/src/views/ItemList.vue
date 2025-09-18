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
import { useRouter, useRoute } from 'vue-router'
import axios from 'axios'
import AppHeader from '../components/AppHeader.vue'
import BottomNavigation from '../components/BottomNavigation.vue'
import ItemCard from '../components/ItemCard.vue'

const router = useRouter()
const route = useRoute()

// 무한 스크롤 관련 상태
const items = ref([])
const isLoading = ref(false)
const hasMore = ref(true)
const currentPage = ref(1)

// 필터링 관련 상태
const currentFilter = ref('all') // 'all', 'favorites', 'recent'
const favoriteProductIds = ref([])
const recentProductIds = ref([])
const userId = ref(localStorage.getItem('userId') || 1)

// 더미 데이터 생성 함수
const generateItems = (page, count = 5) => {
  const newItems = []
  const baseId = (page - 1) * count
  
  const sampleData = [
    { 
      title: '에어팟 프로', 
      location: '군자동', 
      time: '3일 전', 
      price: 220000,
      image: 'https://images.unsplash.com/photo-1572569511254-d8f925fe2cbb?w=400&h=400&fit=crop&crop=center&auto=format&q=90'
    },
    { 
      title: '바이레도 블랑쉬 50ml', 
      location: '광진구 구의제3동', 
      time: '26초 전', 
      price: 4000,
      image: 'https://images.unsplash.com/photo-1541643600914-78b084683601?w=400&h=400&fit=crop&crop=center&auto=format&q=90'
    },
    { 
      title: '샌드위치', 
      location: '동대문구 휘경동', 
      time: '끌올 59초 전', 
      price: 8000,
      image: 'https://images.unsplash.com/photo-1539252554453-80ab65ce3586?w=400&h=400&fit=crop&crop=center&auto=format&q=90'
    },
    { 
      title: '아이폰 13프로맥스', 
      location: '군자동', 
      time: '1일 전', 
      price: 1000000,
      image: 'https://images.unsplash.com/photo-1592750475338-74b7b21085ab?w=400&h=400&fit=crop&crop=center&auto=format&q=90'
    },
    { 
      title: '커피머신', 
      location: '구리시 교문1동', 
      time: '1초 전', 
      price: 100000,
      image: 'https://images.unsplash.com/photo-1495474472287-4d71bcdd2085?w=400&h=400&fit=crop&crop=center&auto=format&q=90'
    },
    { 
      title: '맥북 프로', 
      location: '강남구 역삼동', 
      time: '2시간 전', 
      price: 1500000,
      image: 'https://images.unsplash.com/photo-1517336714731-489689fd1ca8?w=400&h=400&fit=crop&crop=center&auto=format&q=90'
    },
    { 
      title: '나이키 운동화', 
      location: '송파구 잠실동', 
      time: '5시간 전', 
      price: 120000,
      image: 'https://images.unsplash.com/photo-1549298916-b41d501d3772?w=400&h=400&fit=crop&crop=center&auto=format&q=90'
    },
    { 
      title: '갤럭시 버즈', 
      location: '마포구 홍대동', 
      time: '1일 전', 
      price: 180000,
      image: 'https://images.unsplash.com/photo-1606220945770-b5b6c2c55bf1?w=400&h=400&fit=crop&crop=center&auto=format&q=90'
    },
    { 
      title: '무선 마우스', 
      location: '서초구 서초동', 
      time: '2일 전', 
      price: 25000,
      image: 'https://images.unsplash.com/photo-1527864550417-7fd91fc51a46?w=400&h=400&fit=crop&crop=center&auto=format&q=90'
    },
    { 
      title: '키보드', 
      location: '용산구 이태원동', 
      time: '3일 전', 
      price: 80000,
      image: 'https://images.unsplash.com/photo-1541140532154-b024d705b90a?w=400&h=400&fit=crop&crop=center&auto=format&q=90'
    }
  ]
  
  for (let i = 0; i < count; i++) {
    const dataIndex = (baseId + i) % sampleData.length
    const data = sampleData[dataIndex]
    
    newItems.push({
      id: baseId + i + 1,
      title: data.title,
      location: data.location,
      time: data.time,
      price: data.price,
      image: data.image
    })
  }
  
  return newItems
}

// 필터링된 아이템 가져오기
const getFilteredItems = (allItems) => {
  if (currentFilter.value === 'favorites') {
    return allItems.filter(item => favoriteProductIds.value.includes(item.id))
  } else if (currentFilter.value === 'recent') {
    return allItems.filter(item => recentProductIds.value.includes(item.id))
  }
  return allItems
}

// 초기 데이터 로드
const loadInitialData = async () => {
  // URL 파라미터에서 필터 확인
  const filter = route.query.filter
  if (filter && ['favorites', 'recent'].includes(filter)) {
    currentFilter.value = filter
    await loadFilterData()
  }

  const allItems = generateItems(1)
  items.value = getFilteredItems(allItems)
}

// 필터 데이터 로드
const loadFilterData = async () => {
  try {
    if (currentFilter.value === 'favorites') {
      const response = await axios.get(`/api/favorites/user/${userId.value}`)
      favoriteProductIds.value = response.data
    } else if (currentFilter.value === 'recent') {
      const response = await axios.get(`/api/history/user/${userId.value}`)
      recentProductIds.value = response.data
    }
  } catch (error) {
    console.error('필터 데이터 로드 실패:', error)
  }
}

// 추가 데이터 로드
const loadMoreData = async () => {
  if (isLoading.value || !hasMore.value) return

  isLoading.value = true

  // 실제 API 호출을 시뮬레이션
  await new Promise(resolve => setTimeout(resolve, 1000))

  currentPage.value++
  const newItems = generateItems(currentPage.value)
  const filteredNewItems = getFilteredItems(newItems)

  // 최대 50개까지만 로드 (무한 스크롤 시뮬레이션)
  if (items.value.length >= 50) {
    hasMore.value = false
  } else {
    items.value.push(...filteredNewItems)
  }

  isLoading.value = false
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

// 스크롤 이벤트 핸들러
const handleScroll = () => {
  const scrollTop = document.documentElement.scrollTop || document.body.scrollTop
  const scrollHeight = document.documentElement.scrollHeight
  const clientHeight = document.documentElement.clientHeight
  
  // 스크롤이 하단에서 100px 이내에 도달하면 더 로드
  if (scrollTop + clientHeight >= scrollHeight - 100) {
    loadMoreData()
  }
}

// URL 파라미터 변경 감지
watch(
  () => route.query.filter,
  async (newFilter) => {
    currentFilter.value = newFilter || 'all'
    if (newFilter && ['favorites', 'recent'].includes(newFilter)) {
      await loadFilterData()
    }
    // 필터가 변경되면 아이템 목록 다시 로드
    const allItems = generateItems(1, Math.min(items.value.length, 10))
    items.value = getFilteredItems(allItems)
  }
)

// 컴포넌트 마운트/언마운트
onMounted(() => {
  loadInitialData()
  window.addEventListener('scroll', handleScroll)
})

onUnmounted(() => {
  window.removeEventListener('scroll', handleScroll)
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
