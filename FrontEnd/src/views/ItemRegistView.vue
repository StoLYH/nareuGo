<template>
  <div class="regist-view">
    <!-- 상단 헤더: 닫기 버튼, 페이지 제목 -->
    <header class="regist-header">
      <svg
        @click="closePage"
        class="close-btn"
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
        <line x1="18" y1="6" x2="6" y2="18"></line>
        <line x1="6" y1="6" x2="18" y2="18"></line>
      </svg>
      <h1>내 물건 팔기</h1>
      <div class="placeholder"></div>
    </header>

    <!-- 메인 폼 영역 (스크롤 가능) -->
    <main class="form-content">
      <!-- 이미지 등록 섹션 -->
      <section class="image-upload-section">
        <div class="image-uploader">
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
            <line x1="12" y1="5" x2="12" y2="19"></line>
            <line x1="5" y1="12" x2="19" y2="12"></line>
          </svg>
          <span>{{ uploadedImages.length }}/10</span>
        </div>
        <div
          v-for="(image, index) in uploadedImages"
          :key="index"
          class="thumbnail"
        >
          <img :src="image.url" :alt="`uploaded image ${index + 1}`" />
          <button @click="removeImage(index)" class="remove-btn">×</button>
        </div>
      </section>

      <!-- 제목 입력 -->
      <section class="form-group">
        <label for="item-title">제목</label>
        <input
          type="text"
          id="item-title"
          v-model="item.title"
          placeholder="제목을 입력해주세요."
        />
      </section>

      <!-- 내용 입력 -->
      <section class="form-group">
        <label for="item-content">내용</label>
        <textarea
          id="item-content"
          v-model="item.content"
          placeholder="상품 설명을 입력해주세요."
        ></textarea>
      </section>

      <!-- 거래 방식 선택 -->
      <section class="form-group">
        <label>거래방식</label>
        <div class="button-group">
          <button
            :class="{ active: item.tradeType === 'sell' }"
            @click="selectTradeType('sell')"
          >
            판매하기
          </button>
          <button
            :class="{ active: item.tradeType === 'share' }"
            @click="selectTradeType('share')"
          >
            나눔하기
          </button>
        </div>
      </section>

      <!-- 가격 입력 -->
      <section
        class="form-group price-group"
        :class="{ disabled: item.tradeType === 'share' }"
      >
        <input
          type="number"
          v-model="item.price"
          placeholder="가격을 입력해주세요."
          :disabled="item.tradeType === 'share'"
        />
        <span>₩</span>
      </section>
    </main>

    <!-- 하단 작성 완료 버튼 -->
    <footer class="regist-footer">
      <button :disabled="!isFormValid" @click="submitForm">작성 완료</button>
    </footer>
  </div>
</template>

<script>
export default {
  name: "ItemRegistView",
  data() {
    return {
      uploadedImages: [
        { id: 1, url: "https://placehold.co/200x200/EFEFEF/333?text=Img+1" },
        { id: 2, url: "https://placehold.co/200x200/E0E0E0/555?text=Img+2" },
        { id: 3, url: "https://placehold.co/200x200/D0D0D0/555?text=Img+3" },
        { id: 4, url: "https://placehold.co/200x200/C0C0C0/555?text=Img+4" },
        { id: 5, url: "https://placehold.co/200x200/B0B0B0/555?text=Img+5" },
        { id: 6, url: "https://placehold.co/200x200/A0A0A0/555?text=Img+6" },
        { id: 7, url: "https://placehold.co/200x200/909090/555?text=Img+7" },
        { id: 8, url: "https://placehold.co/200x200/808080/FFF?text=Img+8" },
        { id: 9, url: "https://placehold.co/200x200/707070/FFF?text=Img+9" },
        { id: 10, url: "https://placehold.co/200x200/606060/FFF?text=Img+10" },
      ],
      item: {
        title: "",
        content: "",
        tradeType: "sell",
        price: null,
      },
    };
  },
  computed: {
    isFormValid() {
      const isTitleValid = this.item.title.trim() !== "";
      const isContentValid = this.item.content.trim() !== "";
      if (this.item.tradeType === "sell") {
        return (
          isTitleValid &&
          isContentValid &&
          this.item.price !== null &&
          this.item.price > 0
        );
      }
      return isTitleValid && isContentValid;
    },
  },
  methods: {
    closePage() {
      alert("페이지 닫기");
    },
    removeImage(index) {
      this.uploadedImages.splice(index, 1);
    },
    selectTradeType(type) {
      this.item.tradeType = type;
      if (type === "share") {
        this.item.price = null;
      }
    },
    submitForm() {
      if (this.isFormValid) {
        alert("폼 데이터 전송:\n" + JSON.stringify(this.item, null, 2));
      }
    },
  },
};
</script>

<style scoped>
/* ====================================================== */
/* CSS 전체 정리 (중복 제거) */
/* ====================================================== */

/* --- 기본 레이아웃 --- */
.regist-view {
  display: flex;
  flex-direction: column;
  height: 100%;
  background-color: #fff;
}
.regist-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12px 16px;
  border-bottom: 1px solid #f0f0f0;
  flex-shrink: 0;
}
.regist-header h1 {
  font-size: 18px;
  font-weight: 600;
}
.close-btn,
.placeholder {
  width: 24px;
  height: 24px;
}
.close-btn {
  cursor: pointer;
}
.form-content {
  flex-grow: 1;
  overflow-y: auto;
  padding: 20px 16px;
}
.regist-footer {
  flex-shrink: 0;
  padding: 16px;
  background-color: #fff;
  border-top: 1px solid #f0f0f0;
  box-shadow: 0 -2px 5px rgba(0, 0, 0, 0.05);
}

/* --- 이미지 업로드 --- */
.image-upload-section {
  display: flex;
  align-items: center;
  gap: 12px;
  margin-bottom: 24px;
  overflow-x: auto;
  padding-bottom: 16px;
}
.image-uploader {
  flex-shrink: 0;
  width: 80px;
  height: 80px;
  border: 1px solid #ddd;
  border-radius: 8px;
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  cursor: pointer;
  color: #888;
}
.image-uploader svg {
  width: 28px;
  height: 28px;
  margin-bottom: 4px;
}
.thumbnail {
  position: relative;
  flex-shrink: 0;
  width: 80px;
  height: 80px;
  border-radius: 8px;
  overflow: hidden;
}
.thumbnail img {
  width: 100%;
  height: 100%;
  object-fit: cover;
}
.remove-btn {
  position: absolute;
  top: 4px;
  right: 4px;
  width: 20px;
  height: 20px;
  background-color: rgba(0, 0, 0, 0.5);
  color: #fff;
  border: none;
  border-radius: 50%;
  display: flex;
  justify-content: center;
  align-items: center;
  font-size: 14px;
  line-height: 1;
  cursor: pointer;
}

/* --- 폼 요소 --- */
.form-group {
  margin-bottom: 24px;
}
.form-group label {
  display: block;
  font-size: 16px;
  font-weight: 600;
  margin-bottom: 12px;
}
input[type="text"],
textarea,
input[type="number"] {
  width: 100%;
  padding: 12px;
  border: 1px solid #ddd;
  border-radius: 8px;
  font-size: 16px;
}
input[type="text"]:focus,
textarea:focus,
input[type="number"]:focus {
  outline: none;
  border-color: #333;
}
textarea {
  height: 150px;
  resize: none;
}
.button-group {
  display: flex;
  gap: 10px;
}
.button-group button {
  flex-grow: 1;
  padding: 12px;
  font-size: 16px;
  border-radius: 8px;
  border: 1px solid #ddd;
  background-color: #fff;
  cursor: pointer;
  transition: all 0.2s;
}
.button-group button.active {
  background-color: #ff6f0f;
  color: #fff;
  border-color: #ff6f0f;
  font-weight: 600;
}

/* --- 가격 입력 --- */
.price-group {
  position: relative;
  transition: opacity 0.3s ease;
}
.price-group.disabled {
  opacity: 0.5;
}
.price-group input:disabled {
  background-color: #f5f5f5;
  cursor: not-allowed;
}
.price-group input {
  padding-right: 30px;
}
.price-group span {
  position: absolute;
  right: 12px;
  top: 50%;
  transform: translateY(-50%);
  font-size: 16px;
  color: #888;
}

/* --- 하단 버튼 --- */
.regist-footer button {
  width: 100%;
  padding: 14px;
  font-size: 18px;
  font-weight: 600;
  color: #fff;
  background-color: #ff6f0f;
  border: none;
  border-radius: 8px;
  cursor: pointer;
}
.regist-footer button:disabled {
  background-color: #ddd;
  cursor: not-allowed;
}
/* ====================================================== */
/* 스크롤바 스타일링 (최종 수정) */
/* ====================================================== */

/* 1. 메인 콘텐츠의 세로 스크롤바는 숨김 처리 */
.form-content::-webkit-scrollbar {
  display: none;
}
.form-content {
  -ms-overflow-style: none;
  scrollbar-width: none;
}

/* 2. 이미지 업로드 섹션의 가로 스크롤바를 심플하게 디자인 */
.image-upload-section::-webkit-scrollbar {
  height: 8px; /* 스크롤바 영역 높이 */
}
.image-upload-section::-webkit-scrollbar-track {
  background: transparent; /* 트랙(배경)은 보이지 않게 처리 */
}
.image-upload-section::-webkit-scrollbar-thumb {
  background-color: #dbdbdb; /* 핸들 색상 */
  border-radius: 4px; /* 핸들을 둥글게 */
  /* 핸들 양 옆에 투명한 테두리를 주어 더 얇아 보이게 만드는 트릭 */
  border: 2px solid transparent;
  background-clip: padding-box;
}
.image-upload-section {
  scrollbar-width: thin; /* Firefox */
  scrollbar-color: #dbdbdb transparent; /* Firefox: 핸들 색상, 트랙 색상 */
}
</style>
