<template>
  <div class="item-detail-view">
    <!-- 상단 뒤로가기 / 더보기 헤더 -->
    <header class="detail-header">
      <svg
        @click="goBack"
        xmlns="http://www.w3.org/2000/svg"
        width="24"
        height="24"
        viewBox="0 0 24 24"
        fill="none"
        stroke="currentColor"
        stroke-width="2"
        stroke-linecap="round"
        stroke-linejoin="round"
      >
        <polyline points="15 18 9 12 15 6"></polyline>
      </svg>
      <svg
        xmlns="http://www.w3.org/2000/svg"
        width="24"
        height="24"
        viewBox="0 0 24 24"
        fill="none"
        stroke="currentColor"
        stroke-width="2"
        stroke-linecap="round"
        stroke-linejoin="round"
      >
        <circle cx="12" cy="12" r="1"></circle>
        <circle cx="19" cy="12" r="1"></circle>
        <circle cx="5" cy="12" r="1"></circle>
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
            관심 {{ item.likes }} · 조회 {{ item.views }}
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
          fill="none"
          stroke="currentColor"
          stroke-width="2"
          stroke-linecap="round"
          stroke-linejoin="round"
        >
          <path
            d="M20.84 4.61a5.5 5.5 0 0 0-7.78 0L12 5.67l-1.06-1.06a5.5 5.5 0 0 0-7.78 7.78l1.06 1.06L12 21.23l7.78-7.78 1.06-1.06a5.5 5.5 0 0 0 0-7.78z"
          ></path>
        </svg>
      </div>
      <div class="price-info">
        <span class="price">{{ formattedPrice }}원</span>
      </div>
      <button class="chat-button">채팅하기</button>
    </footer>
  </div>
</template>

<script>
export default {
  name: "ItemDetailView",
  data() {
    return {
      currentImageIndex: 0,
      item: {
        title: "소니 Wh-1000xm5 실버 팝니다.",
        imageUrls: [
          "https://placehold.co/600x600/333333/FFF?text=Image+1",
          "https://placehold.co/600x600/555555/FFF?text=Image+2",
          "https://placehold.co/600x600/777777/FFF?text=Image+3",
          "https://placehold.co/600x600/999999/FFF?text=Image+4",
        ],
        category: "디지털기기",
        postedAt: "10시간 전",
        description:
          "8/31일 해외직구한<br>한달도 안된제품 입니다.<br>박풀 S급입니다.",
        likes: 15,
        views: 311,
        price: 360000,
        seller: {
          name: "감성탐방러",
          location: "102동 501호",
          avatarUrl: "https://placehold.co/100x100/EFEFEF/333?text=User",
        },
      },
      otherItems: [
        {
          id: 1,
          imageUrl: "https://placehold.co/300x300/CCCCCC/FFF?text=Item+1",
          title: "다른 상품 1",
        },
        {
          id: 2,
          imageUrl: "https://placehold.co/300x300/AAAAAA/FFF?text=Item+2",
          title: "다른 상품 2",
        },
      ],
    };
  },
  computed: {
    formattedPrice() {
      return this.item.price.toLocaleString("ko-KR");
    },
  },
  methods: {
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
    goBack() {
      alert("뒤로가기 버튼 클릭!");
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
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  display: flex;
  justify-content: space-between;
  padding: 16px;
  z-index: 10;
  color: white;
}
.detail-header svg {
  width: 28px;
  height: 28px;
  cursor: pointer;
  filter: drop-shadow(0px 1px 2px rgba(0, 0, 0, 0.5));
}

/* 메인 스크롤 영역 */
.content-area {
  flex-grow: 1;
  overflow-y: auto;
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
}
.other-item-card {
  width: 40%;
  aspect-ratio: 1 / 1;
  background-color: #eee;
  border-radius: 8px;
  overflow: hidden;
  cursor: pointer;
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
  color: #555;
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
  position: absolute; /* [수정] fixed -> absolute */
  bottom: 0;
  left: 0;
  right: 0;
  /* max-width와 margin은 부모를 따라가므로 더 이상 필요 없습니다. */
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
  color: #555;
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
