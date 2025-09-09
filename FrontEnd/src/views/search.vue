  <template>
    <div class="search-page" :class="{ 'page-enter': true }">
      <!-- 닫기 버튼 -->
      <button class="close-btn" @click="goBack">
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
          <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z" fill="currentColor"/>
        </svg>
      </button>

      <!-- 검색창 -->
      <div class="search-input-container">
        <div class="search-input">
          <svg class="search-icon" width="20" height="20" viewBox="0 0 24 24" fill="none">
            <path d="M15.5 14h-.79l-.28-.27C15.41 12.59 16 11.11 16 9.5 16 5.91 13.09 3 9.5 3S3 5.91 3 9.5 5.91 16 9.5 16c1.61 0 3.09-.59 4.23-1.57l.27.28v.79l5 4.99L20.49 19l-4.99-5zm-6 0C7.01 14 5 11.99 5 9.5S7.01 5 9.5 5 14 7.01 14 9.5 11.99 14 9.5 14z" fill="currentColor"/>
          </svg>
          <div class="search-divider"></div>
          <input 
            v-model="searchQuery"
            @keyup.enter="performSearch"
            type="text" 
            placeholder="Search" 
            class="search-field"
            ref="searchInput"
          />
        </div>
        <button class="filter-btn">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
            <path d="M3 17h18v-2H3v2zm0-5h18V7H3v5zm0-7v2h18V5H3z" fill="currentColor"/>
          </svg>
        </button>
      </div>

      <!-- 최근 검색어 -->
      <div class="recent-searches">
        <h3 class="recent-title">Recent searches</h3>
        <div class="search-list">
          <div 
            v-for="(search, index) in recentSearches" 
            :key="index" 
            class="search-item"
            @click="selectRecentSearch(search)"
          >
            <svg class="search-item-icon" width="16" height="16" viewBox="0 0 24 24" fill="none">
              <path d="M15.5 14h-.79l-.28-.27C15.41 12.59 16 11.11 16 9.5 16 5.91 13.09 3 9.5 3S3 5.91 3 9.5 5.91 16 9.5 16c1.61 0 3.09-.59 4.23-1.57l.27.28v.79l5 4.99L20.49 19l-4.99-5zm-6 0C7.01 14 5 11.99 5 9.5S7.01 5 9.5 5 14 7.01 14 9.5 11.99 14 9.5 14z" fill="currentColor"/>
            </svg>
            <span class="search-term">{{ search }}</span>
            <button 
              class="remove-btn"
              @click.stop="removeRecentSearch(index)"
            >
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none">
                <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z" fill="currentColor"/>
              </svg>
            </button>
          </div>
        </div>
      </div>
    </div>
  </template>

  <script setup>
  import { ref, onMounted, nextTick } from 'vue'

  const searchQuery = ref('')
  const recentSearches = ref(['에어팟', '아이폰', '맥북', '운동화', '키보드'])
  const searchInput = ref(null)

  const goBack = () => {
    // 애니메이션 없이 바로 페이지 이동
    window.history.back()
  }

  const performSearch = () => {
    if (searchQuery.value.trim()) {
      // 최근 검색어에 추가 (중복 제거)
      if (!recentSearches.value.includes(searchQuery.value.trim())) {
        recentSearches.value.unshift(searchQuery.value.trim())
        if (recentSearches.value.length > 5) {
          recentSearches.value.pop()
        }
      }
      console.log('검색어:', searchQuery.value)
      // 검색 결과 페이지로 이동하거나 검색 로직 실행
    }
  }

  const removeRecentSearch = (index) => {
    recentSearches.value.splice(index, 1)
  }

  const selectRecentSearch = (searchTerm) => {
    searchQuery.value = searchTerm
  }

  onMounted(() => {
    nextTick(() => {
      if (searchInput.value) {
        searchInput.value.focus()
      }
    })
  })
  </script>

  <style scoped>
  .search-page {
    width: 100%;
    height: 100vh;
    background-color: white;
    padding: 60px 20px 20px;
    position: relative;
    transform: translateX(100%);
    animation: slideInFromRight 0.3s ease-out forwards;
  }

  @keyframes slideInFromRight {
    to {
      transform: translateX(0);
    }
  }

  .page-enter {
    animation: slideInFromRight 0.3s ease-out forwards;
  }


  .close-btn {
    position: absolute;
    top: 20px;
    right: 20px;
    display: flex;
    align-items: center;
    justify-content: center;
    width: 40px;
    height: 40px;
    background-color: #f8f8f8;
    border: none;
    border-radius: 50%;
    color: #666;
    cursor: pointer;
    transition: background-color 0.2s;
  }

  .close-btn:hover {
    background-color: #f0f0f0;
  }

  .search-input-container {
    display: flex;
    align-items: center;
    gap: 12px;
    margin-bottom: 20px;
  }

  .search-input {
    flex: 1;
    display: flex;
    align-items: center;
    background-color: #f8f8f8;
    border-radius: 12px;
    padding: 12px 16px;
    gap: 12px;
  }

  .search-icon {
    color: #999;
    flex-shrink: 0;
  }

  .search-divider {
    width: 1px;
    height: 20px;
    background-color: #ddd;
  }

  .search-field {
    flex: 1;
    border: none;
    background: none;
    outline: none;
    font-size: 16px;
    color: #333;
  }

  .search-field::placeholder {
    color: #999;
  }

  .filter-btn {
    display: flex;
    align-items: center;
    justify-content: center;
    width: 44px;
    height: 44px;
    background-color: #f8f8f8;
    border-radius: 12px;
    color: #666;
    transition: background-color 0.2s;
  }

  .filter-btn:hover {
    background-color: #f0f0f0;
  }

  .recent-searches {
    border-top: 1px solid #f0f0f0;
    padding-top: 20px;
  }

  .recent-title {
    font-size: 14px;
    color: #999;
    margin: 0 0 16px 0;
    font-weight: 500;
  }

  .search-list {
    display: flex;
    flex-direction: column;
    gap: 8px;
  }

  .search-item {
    display: flex;
    align-items: center;
    gap: 12px;
    padding: 8px 0;
    cursor: pointer;
    transition: background-color 0.2s;
    border-radius: 8px;
  }

  .search-item:hover {
    background-color: #f8f8f8;
  }

  .search-item-icon {
    color: #999;
    flex-shrink: 0;
  }

  .search-term {
    flex: 1;
    font-size: 16px;
    color: #333;
  }

  .remove-btn {
    display: flex;
    align-items: center;
    justify-content: center;
    width: 24px;
    height: 24px;
    background: none;
    border: none;
    color: #999;
    cursor: pointer;
    border-radius: 4px;
    transition: background-color 0.2s;
  }

  .remove-btn:hover {
    background-color: #f0f0f0;
  }
  </style>
