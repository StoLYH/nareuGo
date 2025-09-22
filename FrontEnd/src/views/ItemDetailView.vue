<template>
  <div class="item-detail-view">
    <!-- 상단 뒤로가기 / 더보기 헤더 -->
    <header class="detail-header">
      <!-- Back: solid chevron left -->
      <svg
        @click="goBack"
        xmlns="http://www.w3.org/2000/svg"
        width="24"
        height="24"
        viewBox="0 0 24 24"
        fill="currentColor"
        aria-label="뒤로가기"
      >
        <path d="M15.41 7.41 14 6l-6 6 6 6 1.41-1.41L10.83 12z"/>
      </svg>
      <!-- More: solid kebab horizontal -->
      <svg
        xmlns="http://www.w3.org/2000/svg"
        width="24"
        height="24"
        viewBox="0 0 24 24"
        fill="currentColor"
        aria-label="더보기"
      >
        <circle cx="12" cy="12" r="2"></circle>
        <circle cx="19" cy="12" r="2"></circle>
        <circle cx="5" cy="12" r="2"></circle>
      </svg>
    </header>

    <!-- 메인 스크롤 영역 -->
    <main class="content-area">
      <!-- 상품 이미지 슬라이더 -->
      <div class="image-slider">
        <!-- 이미지들이 담길 슬라이더 트랙 -->
        <div
          class="slider-track"
          :style="{ transform: `translateX(-${currentImageIndex * 100}%)` }"
        >
          <div
            class="slide"
            v-for="(imageUrl, index) in item.imageUrls"
            :key="index"
          >
            <img :src="imageUrl" :alt="`${item.title} 이미지 ${index + 1}`" />
          </div>
        </div>

        <!-- 이전/다음 버튼 -->
        <button
          v-if="currentImageIndex > 0"
          @click="prevImage"
          class="slider-nav prev"
        >
          <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24">
            <path d="M15.41 7.41L14 6l-6 6 6 6 1.41-1.41L10.83 12z" />
          </svg>
        </button>
        <button
          v-if="currentImageIndex < item.imageUrls.length - 1"
          @click="nextImage"
          class="slider-nav next"
        >
          <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24">
            <path d="M10 6L8.59 7.41 13.17 12l-4.58 4.59L10 18l6-6z" />
          </svg>
        </button>

        <!-- 하단 점 인디케이터 -->
        <div class="slider-dots">
          <span
            v-for="(imageUrl, index) in item.imageUrls"
            :key="`dot-${index}`"
            class="dot"
            :class="{ active: index === currentImageIndex }"
            @click="goToImage(index)"
          ></span>
        </div>
      </div>

      <div class="info-wrapper">
        <!-- 판매자 프로필 -->
        <section class="seller-profile">
          <div class="avatar">
            <img :src="item.seller.avatarUrl" :alt="item.seller.name" />
          </div>
          <div class="seller-info">
            <span class="seller-name">{{ item.seller.name }}</span>
            <span class="seller-location">{{ item.seller.location }}</span>
          </div>
        </section>

        <!-- 상품 상세 정보 (이하 생략) -->
        <section class="item-info">
          <h1 class="item-title">{{ item.title }}</h1>
          <p class="item-meta">{{ item.category }} · {{ item.postedAt }}</p>
          <p class="item-description" v-html="item.description"></p>
          <p class="item-stats">
            <!-- 관심 {{ item.likes }} · 조회 {{ item.views }} -->
            관심 7 · 조회 13
          </p>
        </section>

        <!-- 판매자의 다른 상품 -->
         <section class="other-items">
          <div class="section-header">
            <h2>{{ item.seller.name }}님의 판매 상품</h2>
            <svg
              xmlns="http://www.w3.org/2000/svg"
              width="20"
              height="20"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              stroke-width="2"
              stroke-linecap="round"
              stroke-linejoin="round"
            >
              <polyline points="9 18 15 12 9 6"></polyline>
            </svg>
          </div>
          <div class="item-list">
            <div
              class="other-item-card"
              v-for="otherItem in otherItems"
              :key="otherItem.id"
            >
              <img :src="otherItem.imageUrl" :alt="otherItem.title" />
            </div>
          </div>
        </section>
       </div>
    </main>

    <!-- 하단 고정 액션 바 -->
    <footer class="action-bar">
      <div class="like-button">
        <svg
          xmlns="http://www.w3.org/2000/svg"
          width="24"
          height="24"
          viewBox="0 0 24 24"
          fill="currentColor"
          aria-label="좋아요"
        >
          <path d="M12.1 21.35c-.07.05-.13.08-.2.08s-.13-.03-.2-.08C7.14 17.24 4 14.39 4 10.9 4 8.5 5.9 6.6 8.3 6.6c1.25 0 2.45.57 3.2 1.49.75-.92 1.95-1.49 3.2-1.49 2.4 0 4.3 1.9 4.3 4.3 0 3.49-3.14 6.34-6.9 10.45Z"/>
        </svg>
      </div>
      <div class="price-info">
        <span class="price">{{ formattedPrice }}원</span>
      </div>
      <button class="chat-button" @click="startChat">채팅하기</button>
    </footer>
  </div>
</template>

<script>
import { getProducts } from '@/api/product/product.js'
import { findOrCreateChatRoom } from '@/api/chat/chat.js'

export default {
  name: "ItemDetailView",
  data() {
    return {
      currentImageIndex: 0,
      item: {
        id: null,
        title: "",
        imageUrls: [],
        category: "디지털기기",
        postedAt: "",
        description: "",
        likes: 0,
        views: 0,
        price: 0,
        location: "",
        seller: {
          name: "201동 행복주민",
          location: "",
          avatarUrl: "/images/social/사용자더미.png",
        },
      },
      otherItems: [
        {
          id: 1,
          imageUrl: "/images/social/상품더미1.png",
          title: "다른 상품 1",
        },
        {
          id: 2,
          imageUrl: "/images/social/상품더미2.png", 
          title: "다른 상품 2",
        },
      ],
    };
  },
  async mounted() {
    await this.loadItemData();
  },
  computed: {
    formattedPrice() {
      return this.item.price.toLocaleString("ko-KR");
    },
  },
  methods: {
    async loadItemData() {
      try {
        const productId = this.$route.params.id;
        console.log('상품 ID:', productId);
        
        // 1. sessionStorage에서 상품 정보 확인
        const storedItemData = sessionStorage.getItem(`item_${productId}`);
        if (storedItemData) {
          console.log('sessionStorage에서 상품 정보 가져옴');
          const itemData = JSON.parse(storedItemData);
          this.setItemData(itemData);
          return;
        }
        
        // 2. 라우터 state에서 전달받은 상품 정보 확인
        const routerState = history.state;
        if (routerState && routerState.itemData) {
          console.log('라우터에서 전달받은 상품 정보:', routerState.itemData);
          this.setItemData(routerState.itemData);
          return;
        }
        
        // 3. 위의 방법들이 모두 실패하면 API 호출
        if (productId) {
          console.log('API에서 상품 정보 가져오기');
          await this.fetchItemFromAPI(productId);
        }
      } catch (error) {
        console.error('상품 정보 로드 실패:', error);
      }
    },

    setItemData(itemData) {
      // 받은 데이터 전체를 콘솔에 출력
      console.log('=== ItemDetail에서 받은 상품 데이터 전체 ===');
      console.log('itemData:', itemData);
      console.log('itemData.description:', itemData.description);
      console.log('itemData.allImages:', itemData.allImages);
      console.log('itemData.image:', itemData.image);
      console.log('itemData 키들:', Object.keys(itemData));
      console.log('=======================================');

      // ItemList에서 받은 데이터를 ItemDetail 형식으로 변환
      this.item = {
        id: itemData.id,
        title: itemData.title,
        imageUrls: itemData.allImages || [itemData.image], // 모든 이미지 또는 첫 번째 이미지
        category: "디지털기기", // 기본값
        postedAt: itemData.time,
        description: itemData.description || "상품 설명을 확인해보세요.", // 설명이 있으면 사용
        likes: 0, // 기본값
        views: 0, // 기본값
        price: itemData.price,
        location: itemData.location,
        sellerId: itemData.sellerId, // 채팅을 위해 필요한 판매자 ID 추가
        seller: {
          name: "201동 행복주민", // 고정값
          location: `${itemData.location} ${itemData.apartmentName || ''}`.trim(), // 구+동 + 아파트명
          avatarUrl: "/images/social/사용자더미.png",
        },
      };

      // 변환된 데이터도 출력
      console.log('=== 변환된 this.item 데이터 ===');
      console.log('this.item:', this.item);
      console.log('this.item.description:', this.item.description);
      console.log('this.item.imageUrls:', this.item.imageUrls);
      console.log('============================');
    },

    async fetchItemFromAPI(productId) {
      // 새로고침 등으로 state가 없을 때 API에서 데이터 가져오기
      // 임시로 전체 목록에서 찾기 (실제로는 단일 상품 API 호출)
      try {
        const userInfo = JSON.parse(localStorage.getItem('user'));
        if (userInfo && userInfo.userId) {
          const productList = await getProducts(userInfo.userId);
          const product = productList.find(p => p.productId == productId);
          if (product) {
            const itemData = {
              id: product.productId,
              title: product.title,
              location: `${product.siGunGu} ${product.eupMyeonDong}`,
              time: this.formatTimeAgo(product.createdAt),
              price: product.price,
              image: product.imageUrls && product.imageUrls.length > 0 ? product.imageUrls[0] : null,
              // 추가 정보들
              allImages: product.imageUrls || [],
              description: product.description || "",
              sellerId: product.sellerId, // 채팅을 위해 필요한 판매자 ID
              siDo: product.siDo,
              siGunGu: product.siGunGu,
              eupMyeonDong: product.eupMyeonDong,
              apartmentName: product.apartmentName, // 아파트명 포함
              status: product.status,
              createdAt: product.createdAt,
              updatedAt: product.updatedAt
            };
            this.setItemData(itemData);
          }
        }
      } catch (error) {
        console.error('API에서 상품 정보 가져오기 실패:', error);
      }
    },

    formatTimeAgo(dateString) {
      const now = new Date();
      
      // 백엔드에서 받은 시간을 UTC로 파싱 후 로컬 시간대로 변환
      let past;
      if (dateString.includes('T') && dateString.includes('Z')) {
        // 이미 UTC 형식인 경우 (예: 2025-09-17T04:49:39Z)
        past = new Date(dateString);
      } else if (dateString.includes('T')) {
        // ISO 형식이지만 Z가 없는 경우 UTC로 가정
        past = new Date(dateString + 'Z');
      } else {
        // 다른 형식인 경우 그대로 파싱
        past = new Date(dateString);
      }
      
      const diffInMinutes = Math.floor((now - past) / (1000 * 60));
      
      // 디버깅용 로그
      console.log(`시간 계산: now=${now.toISOString()}, past=${past.toISOString()}, diff=${diffInMinutes}분`);
      
      if (diffInMinutes < 1) return '방금 전';
      if (diffInMinutes < 60) return `${diffInMinutes}분 전`;
      
      const diffInHours = Math.floor(diffInMinutes / 60);
      if (diffInHours < 24) return `${diffInHours}시간 전`;
      
      const diffInDays = Math.floor(diffInHours / 24);
      return `${diffInDays}일 전`;
    },

    goBack() {
      this.$router.go(-1);
    },

    nextImage() {
      if (this.currentImageIndex < this.item.imageUrls.length - 1) {
        this.currentImageIndex++;
      }
    },
    prevImage() {
      if (this.currentImageIndex > 0) {
        this.currentImageIndex--;
      }
    },
    goToImage(index) {
      this.currentImageIndex = index;
    },
    async startChat() {
      try {
        // 현재 로그인한 사용자 정보 가져오기
        const currentUser = JSON.parse(localStorage.getItem('user'));
        if (!currentUser || !currentUser.userId) {
          alert('로그인이 필요합니다.');
          this.$router.push('/login');
          return;
        }

        // 자기 자신과 채팅하려고 하는 경우 방지
        if (currentUser.userId.toString() === this.item.sellerId?.toString()) {
          alert('자신의 상품에는 채팅할 수 없습니다.');
          return;
        }

        console.log('채팅방 생성 요청:', {
          currentUserId: currentUser.userId,
          sellerId: this.item.sellerId || 'unknown',
          productId: this.item.id // 상품 ID 로깅 추가
        });

        // 채팅방 생성 또는 기존 채팅방 찾기
        const roomId = await findOrCreateChatRoom(
          currentUser.userId.toString(), 
          this.item.sellerId?.toString() || '1', // sellerId가 없으면 임시로 1 사용
          this.item.id // this.item.productId 대신 this.item.id 사용
        );

        console.log('생성된 채팅방 ID:', roomId);

        // 채팅 페이지로 이동 (roomId와 상대방 정보 전달)
        this.$router.push({
          path: `/chat/${roomId}`,
          query: {
            otherUserId: this.item.sellerId,
            otherUserName: this.item.seller.name,
            productTitle: this.item.title,
            productId: this.item.id // productId 추가
          }
        });

      } catch (error) {
        console.error('채팅방 생성 실패:', error);
        alert('채팅방 생성에 실패했습니다. 다시 시도해주세요.');
      }
    },

    goToPayment() {
      // 결제 페이지로 이동 (기존 함수 유지)
      this.$router.push("/payment");
    },
  },
};
</script>

<style scoped>
/* 전체 레이아웃 */
.item-detail-view {
  display: flex;
  flex-direction: column;
  height: 100%;
  background-color: #fff;
  position: relative;
}

/* 상단 헤더 */
.detail-header {
  position: fixed;
  top: 0;
  left: 50%;
  transform: translateX(-50%);
  width: 100%;
  max-width: 390px;
  display: flex;
  justify-content: space-between;
  align-items: center;
  height: 64px; /* 헤더 고정 높이 */
  padding: 12px 16px;
  z-index: 50;
  color: #ffffff;
  background-color: #4682B4;
  box-shadow: 0 4px 12px rgba(0,0,0,0.12);
}
.detail-header svg {
  width: 24px;
  height: 24px;
  color: #ffffff; /* 아이콘 색상 명시 */
}
/* 아이콘 버튼 감성으로 보이도록 배경/라운드 부여 */
.detail-header svg {
  cursor: pointer;
  border-radius: 50%;
  background-color: rgba(255,255,255,0.18);
  transition: background-color 0.2s ease, transform 0.2s ease;
}
.detail-header svg:hover {
  background-color: rgba(255,255,255,0.28);
}

/* 메인 스크롤 영역 */
.content-area {
  flex-grow: 1;
  overflow-y: auto;
  /* 고정 헤더 높이만큼 상단 여백 확보 */
  padding-top: 64px; /* 헤더 높이에 맞춤 */
  padding-bottom: 80px;
}

/* 이미지 슬라이더 스타일 */
.image-slider {
  position: relative;
  width: 100%;
  aspect-ratio: 1 / 1;
  background-color: #eee;
  overflow: hidden;
}
.slider-track {
  display: flex;
  height: 100%;
  transition: transform 0.4s ease-in-out;
}
.slide {
  flex-shrink: 0;
  width: 100%;
  height: 100%;
}
.slide img {
  width: 100%;
  height: 100%;
  object-fit: cover;
}

/* ====================================================== */
/* 이전/다음 버튼 스타일 (수정된 부분) */
/* ====================================================== */
.slider-nav {
  position: absolute;
  top: 50%;
  transform: translateY(-50%);
  background-color: rgba(0, 0, 0, 0.3);
  border: none;
  border-radius: 50%;
  width: 32px;
  height: 32px;
  display: flex;
  justify-content: center;
  align-items: center;
  cursor: pointer;
  z-index: 5;

  /* [추가] 평소에는 투명하게 만듦 */
  opacity: 0;
  /* [추가] 부드러운 전환 효과 */
  transition: opacity 0.3s ease-in-out;
}

/* [추가] 이미지 슬라이더에 마우스를 올렸을 때만 버튼을 보이게 함 */
.image-slider:hover .slider-nav {
  opacity: 1;
}

.slider-nav.prev {
  left: 10px;
}
.slider-nav.next {
  right: 10px;
}
.slider-nav svg {
  fill: white;
  width: 24px;
  height: 24px;
}

/* 하단 점 인디케이터 */
.slider-dots {
  position: absolute;
  bottom: 16px;
  left: 50%;
  transform: translateX(-50%);
  display: flex;
  gap: 8px;
  z-index: 5;
}
.dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background-color: rgba(255, 255, 255, 0.5);
  cursor: pointer;
  transition: background-color 0.3s;
}
.dot.active {
  background-color: white;
}

/* 콘텐츠 정보 전체 래퍼 */
.info-wrapper {
  padding: 0 16px;
}
/* 이하 스타일은 변경 없음 */
.seller-profile {
  display: flex;
  align-items: center;
  padding: 16px 0;
  border-bottom: 1px solid #f0f0f0;
}
.avatar {
  width: 40px;
  height: 40px;
  border-radius: 50%;
  overflow: hidden;
  margin-right: 12px;
}
.avatar img {
  width: 100%;
  height: 100%;
  object-fit: cover;
}
.seller-info {
  display: flex;
  flex-direction: column;
}
.seller-name {
  font-weight: 600;
  font-size: 16px;
}
.seller-location {
  font-size: 14px;
  color: #666;
}
.item-info {
  padding: 20px 0;
  border-bottom: 1px solid #f0f0f0;
}
.item-title {
  font-size: 20px;
  font-weight: bold;
  margin-bottom: 8px;
}
.item-meta {
  font-size: 13px;
  color: #888;
  margin-bottom: 16px;
}
.item-description {
  font-size: 16px;
  line-height: 1.6;
  margin-bottom: 16px;
}
.item-stats {
  font-size: 13px;
  color: #888;
}
.other-items {
  padding: 24px 0;
}
.section-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 16px;
  cursor: pointer;
}
.section-header h2 {
  font-size: 16px;
  font-weight: bold;
}
.section-header svg {
  color: #888;
}
.item-list {
  display: flex;
  gap: 12px;
  width: 100%;
}
.other-item-card {
  flex: 1; /* 동일한 크기로 분할 */
  aspect-ratio: 1 / 1;
  background-color: #eee;
  border-radius: 8px;
  overflow: hidden;
  cursor: pointer;
  max-width: calc(50% - 6px); /* 50%에서 gap의 절반을 뺀 크기 */
}
.other-item-card img {
  width: 100%;
  height: 100%;
  object-fit: cover;
}
.action-bar {
  position: absolute;
  bottom: 0;
  left: 0;
  right: 0;
  height: 80px;
  display: flex;
  align-items: center;
  padding: 0 16px;
  background-color: #fff;
  border-top: 1px solid #f0f0f0;
  box-shadow: 0 -2px 5px rgba(0, 0, 0, 0.05);
}
.like-button {
  padding-right: 16px;
  border-right: 1px solid #f0f0f0;
  color: #ff6b9a;
  cursor: pointer;
}
.price-info {
  flex-grow: 1;
  padding-left: 16px;
}
.price {
  font-size: 18px;
  font-weight: bold;
}
.chat-button {
  background-color: #ff6f0f;
  color: #fff;
  border: none;
  border-radius: 6px;
  padding: 12px 20px;
  font-size: 16px;
  font-weight: 700;
  cursor: pointer;
}
.content-area::-webkit-scrollbar {
  display: none;
}
.content-area {
  -ms-overflow-style: none;
  scrollbar-width: none;
}
.avatar {
  width: 40px;
  height: 40px;
  border-radius: 50%;
  overflow: hidden;
  margin-right: 12px;
}
.avatar img {
  width: 100%;
  height: 100%;
  object-fit: cover;
}
.seller-info {
  display: flex;
  flex-direction: column;
}
.seller-name {
  font-weight: 600;
  font-size: 16px;
}
.seller-location {
  font-size: 14px;
  color: #666;
  margin-top: 6px;
}

/* 상품 정보 */
.item-info {
  padding: 20px 0;
  border-bottom: 1px solid #f0f0f0;
}
.item-title {
  font-size: 20px;
  font-weight: bold;
  margin-bottom: 14px;
}
.item-meta {
  font-size: 13px;
  color: #888;
  margin-bottom: 16px;
}
.item-description {
  font-size: 16px;
  line-height: 1.6;
  margin-bottom: 16px;
}
.item-stats {
  font-size: 13px;
  color: #888;
}

/* 판매자의 다른 상품 */
.other-items {
  padding: 24px 0;
}
.section-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 16px;
}
.section-header h2 {
  font-size: 16px;
  font-weight: bold;
}
.section-header svg {
  color: #888;
  cursor: pointer;
}
.item-list {
  display: flex;
  gap: 12px;
}
.other-item-card {
  width: 40%;
  aspect-ratio: 1 / 1;
  background-color: #eee;
  border-radius: 8px;
  overflow: hidden;
}
.other-item-card img {
  width: 100%;
  height: 100%;
  object-fit: cover;
}

/* 하단 고정 액션 바 */
.action-bar {
  position: fixed;
  bottom: 0;
  left: 50%;
  transform: translateX(-50%);
  width: 100%;
  max-width: 390px;
  z-index: 50;
  height: 80px;
  display: flex;
  align-items: center;
  padding: 0 16px;
  background-color: #fff;
  border-top: 1px solid #f0f0f0;
  box-shadow: 0 -2px 5px rgba(0, 0, 0, 0.05);
}
.like-button {
  padding-right: 16px;
  border-right: 1px solid #f0f0f0;
  color: #ff6b9a;
  cursor: pointer;
}
.price-info {
  flex-grow: 1;
  padding-left: 16px;
}
.price {
  font-size: 18px;
  font-weight: bold;
}
.chat-button {
  background-color: #4682b4;
  color: white;
  border: none;
  border-radius: 6px;
  padding: 12px 20px;
  font-size: 16px;
  font-weight: bold;
  cursor: pointer;
}
/* 스크롤이 발생하는 실제 콘텐츠 영역을 타겟합니다. */
.content-area {
  /* ====================================================== */
  /* ↓↓↓ 아래 스크롤바 커스텀 코드를 추가해주세요 ↓↓↓ */
  /* ====================================================== */

  /* --- Firefox를 위한 설정 --- */
  /* 스크롤바를 얇게 만들고, 색상을 지정합니다 (썸 / 트랙) */
  scrollbar-width: thin;
  scrollbar-color: var(--gray) transparent;
}

/* --- Webkit 기반 브라우저(Chrome, Safari, Edge 등)를 위한 설정 --- */

/* --- Firefox --- */
.content-area {
  scrollbar-width: thin;
  scrollbar-color: #dbdbdb transparent; /* 핸들 색상, 트랙 색상 */
}
/* --- Firefox 브라우저를 위한 설정 --- */
.content-area {
  scrollbar-width: thin;
  scrollbar-color: #c1c1c1 transparent; /* 핸들 색상, 트랙(배경) 색상 */
}

/* --- Webkit 기반 브라우저 (Chrome, Safari, Edge 등) --- */
/* 스크롤바 자체를 보이지 않게 처리합니다. */
.content-area::-webkit-scrollbar {
  display: none;
}

/* --- Firefox 및 기타 브라우저 --- */
/* Firefox에서는 scrollbar-width 속성을 사용하고, */
/* IE/Edge 구버전을 위해 -ms-overflow-style도 추가합니다. */
.content-area {
  -ms-overflow-style: none; /* IE and Edge */
  scrollbar-width: none; /* Firefox */
}
</style>
