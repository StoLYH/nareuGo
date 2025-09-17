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
        <div class="image-uploader" @click="selectFiles">
          <input 
            ref="fileInput" 
            type="file" 
            multiple 
            accept="image/*" 
            @change="handleFileSelect"
            style="display: none"
          />
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
          <img :src="image.preview" :alt="`uploaded image ${index + 1}`" />
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
      <button :disabled="!isFormValid || isLoading" @click="submitForm">
        {{ isLoading ? '등록 중...' : '작성 완료' }}
      </button>
    </footer>
  </div>
</template>

<script>
import { createProduct, uploadToS3 } from '@/api/product.js'

export default {
  name: "ItemRegistView",
  data() {
    return {
      uploadedImages: [], // 실제 파일 객체들
      selectedFiles: [], // 선택된 파일들
      item: {
        title: "",
        content: "",
        tradeType: "sell",
        price: null,
      },
      isLoading: false,
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
    
    // 파일 선택
    selectFiles() {
      this.$refs.fileInput.click();
    },
    
    // 파일 선택 처리
    handleFileSelect(event) {
      const files = Array.from(event.target.files);
      const remainingSlots = 10 - this.uploadedImages.length;
      const filesToAdd = files.slice(0, remainingSlots);
      
      filesToAdd.forEach(file => {
        if (file.type.startsWith('image/')) {
          const reader = new FileReader();
          reader.onload = (e) => {
            this.uploadedImages.push({
              file: file,
              preview: e.target.result,
              name: file.name
            });
          };
          reader.readAsDataURL(file);
        }
      });
      
      // 입력 초기화
      event.target.value = '';
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
    async submitForm() {
      if (!this.isFormValid || this.isLoading) {
        return;
      }

      this.isLoading = true;

      try {
        // 로컬 스토리지에서 사용자 정보 가져오기
        const userInfo = JSON.parse(localStorage.getItem('user'));
        if (!userInfo || !userInfo.userId) {
          alert('로그인이 필요합니다.');
          return;
        }

        // 1단계: 상품 정보만 먼저 등록 (파일명 배열 전송)
        const productData = {
          sellerId: userInfo.userId,
          title: this.item.title,
          description: this.item.content,
          price: this.item.tradeType === 'sell' ? this.item.price : 0,
          files: this.uploadedImages.map(img => img.name) // 파일명 배열
        };

        console.log('상품 등록 요청 데이터:', productData);

        // API 호출하여 S3 presigned URL 받기
        const response = await createProduct(productData);
        console.log('상품 등록 성공, S3 URLs 받음:', response);

        // 2단계: S3에 직접 파일 업로드
        if (response.urls && response.urls.length > 0) {
          const uploadPromises = response.urls.map(async (presignedUrl, index) => {
            if (this.uploadedImages[index]) {
              console.log(`파일 ${index + 1} S3 업로드 시작:`, this.uploadedImages[index].name);
              return await uploadToS3(presignedUrl, this.uploadedImages[index].file);
            }
          });

          // 모든 파일 업로드 완료 대기
          await Promise.all(uploadPromises);
          console.log('모든 파일 S3 업로드 완료');
        }

        alert('상품이 성공적으로 등록되었습니다!');
        
        // 성공 후 폼 초기화 또는 페이지 이동
        this.resetForm();
        
      } catch (error) {
        console.error('상품 등록 실패:', error);
        
        // 에러 메시지 처리
        let errorMessage = '상품 등록에 실패했습니다.';
        if (error.response?.data?.message) {
          errorMessage = error.response.data.message;
        } else if (error.message) {
          errorMessage = error.message;
        }
        
        alert(errorMessage);
      } finally {
        this.isLoading = false;
      }
    },

    resetForm() {
      this.item = {
        title: "",
        content: "",
        tradeType: "sell",
        price: null,
      };
      this.uploadedImages = [];
      this.selectedFiles = [];
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
