<template>
  <div class="payment-container">
    <div class="payment-detail-view">
      <!-- 상단 헤더 -->
      <header class="payment-header">
        <div class="header-spacer"></div>
        <h1 class="header-title">결제하기</h1>
        <button @click="closeAndGoList" class="close-button" aria-label="닫기">
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
            <line x1="18" y1="6" x2="6" y2="18"></line>
            <line x1="6" y1="6" x2="18" y2="18"></line>
          </svg>
        </button>
      </header>

      <!-- 메인 콘텐츠 -->
      <main class="payment-content">
        <!-- 주문 정보 카드 -->
        <section class="order-info-card">
          <div class="card-header">
            <h2>주문 정보</h2>
          </div>
          <div class="product-summary">
            <div class="product-image">
              <img
                :src="orderInfo.productImage"
                :alt="orderInfo.productTitle"
              />
            </div>
            <div class="product-details">
              <h3 class="product-title">{{ orderInfo.productTitle }}</h3>
              <p class="product-price">{{ formatPrice(orderInfo.price) }}원</p>
              <p class="seller-info">
                판매자: {{ orderInfo.sellerName }}
              </p>
              <p class="seller-info address">
                {{ orderInfo.sellerLocation }}
              </p>
            </div>
          </div>
        </section>

        <!-- 결제 정보 카드 -->
        <section class="payment-info-card">
          <div class="card-header">
            <h2>결제 정보</h2>
          </div>
          <div class="payment-details">
            <div class="detail-row">
              <span class="label">상품 금액</span>
              <span class="value">{{ formatPrice(orderInfo.price) }}원</span>
            </div>
            <div class="detail-row">
              <span class="label">배송비</span>
              <span class="value">무료</span>
            </div>
            <div class="detail-row total-row">
              <span class="label">총 결제 금액</span>
              <span class="value total-amount"
                >{{ formatPrice(orderInfo.price) }}원</span
              >
            </div>
          </div>
        </section>

        <!-- 구매자 정보 카드 -->
        <section class="buyer-info-card">
          <div class="card-header">
            <h2>구매자 정보</h2>
          </div>
          <div class="buyer-details">
            <div class="detail-row">
              <span class="label">이름</span>
              <span class="value">{{ buyerInfo.name }}</span>
            </div>
            <div class="detail-row">
              <span class="label">연락처</span>
              <span class="value">{{ buyerInfo.phone }}</span>
            </div>
            <div class="detail-row">
              <span class="label">배송 주소</span>
              <span class="value">{{ buyerInfo.address }}</span>
            </div>
          </div>
        </section>

        <!-- 결제 수단 선택 -->
        <section class="payment-method-card">
          <div class="card-header">
            <h2>결제 수단</h2>
          </div>
          <div class="payment-methods">
            <div
              class="method-item"
              :class="{ active: selectedMethod === 'card' }"
              @click="selectPaymentMethod('card')"
            >
              <div class="method-icon">
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
                  <rect x="1" y="4" width="22" height="16" rx="2" ry="2"></rect>
                  <line x1="1" y1="10" x2="23" y2="10"></line>
                </svg>
              </div>
              <div class="method-info">
                <span class="method-name">카드 결제</span>
                <span class="method-desc">신용카드, 체크카드</span>
              </div>
              <div
                class="method-radio"
                :class="{ active: selectedMethod === 'card' }"
              >
                <div class="radio-dot"></div>
              </div>
            </div>

            <div
              class="method-item"
              :class="{ active: selectedMethod === 'account' }"
              @click="selectPaymentMethod('account')"
            >
              <div class="method-icon">
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
                  <rect x="2" y="3" width="20" height="14" rx="2" ry="2"></rect>
                  <line x1="8" y1="21" x2="16" y2="21"></line>
                  <line x1="12" y1="17" x2="12" y2="21"></line>
                </svg>
              </div>
              <div class="method-info">
                <span class="method-name">계좌이체</span>
                <span class="method-desc">실시간 계좌이체</span>
              </div>
              <div
                class="method-radio"
                :class="{ active: selectedMethod === 'account' }"
              >
                <div class="radio-dot"></div>
              </div>
            </div>
          </div>
        </section>

        <!-- 약관 동의 -->
        <section class="terms-card">
          <div class="terms-checkbox">
            <input type="checkbox" id="terms" v-model="termsAgreed" />
            <label for="terms">
              <span class="checkbox-custom" :class="{ checked: termsAgreed }">
                <svg
                  v-if="termsAgreed"
                  xmlns="http://www.w3.org/2000/svg"
                  width="16"
                  height="16"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="currentColor"
                  stroke-width="2"
                  stroke-linecap="round"
                  stroke-linejoin="round"
                >
                  <polyline points="20 6 9 17 4 12"></polyline>
                </svg>
              </span>
              <span class="terms-text">
                <strong>구매 약관</strong> 및
                <strong>개인정보 처리방침</strong>에 동의합니다.
              </span>
            </label>
          </div>
        </section>
      </main>

      <!-- 하단 결제 버튼 -->
      <footer class="payment-footer">
        <div class="payment-summary">
          <span class="total-label">총 결제 금액</span>
          <span class="total-price">{{ formatPrice(orderInfo.price) }}원</span>
        </div>
        <button
          class="payment-button"
          :class="{ disabled: !canProceedPayment }"
          :disabled="!canProceedPayment"
          @click="proceedPayment"
        >
          {{ formatPrice(orderInfo.price) }}원 결제하기
        </button>
      </footer>

      <!-- 로딩 오버레이 -->
      <div v-if="isLoading" class="loading-overlay">
        <div class="loading-spinner"></div>
        <p>결제 처리 중...</p>
      </div>
    </div>
  </div>
</template>

<script>
import { requestPayment } from "../config/tossPayments.js";

export default {
  name: "PaymentDetailView",
  data() {
    return {
      selectedMethod: "card",
      termsAgreed: false,
      isLoading: false,
      isLoadingOrder: true,
      orderInfo: {
        orderId: null,
        productId: null,
        productTitle: "",
        productImage: "https://placehold.co/300x300/333333/FFF?text=Loading",
        price: 0,
        sellerName: "",
        sellerLocation: "",
        status: "",
        amount: 0,
        tossOrderId: null  // 토스페이먼츠용 orderId 추가
      },
      buyerInfo: {
        name: "",
        phone: "",
        address: "",
      },
    };
  },
  computed: {
    canProceedPayment() {
      return this.termsAgreed && !this.isLoading;
    },
  },
  methods: {
    formatPrice(price) {
      return price.toLocaleString("ko-KR");
    },
    closeAndGoList() {
      // 아이템 리스트(홈) 화면으로 이동
      this.$router.push('/home');
    },

    // ===== orderId로 주문 정보 조회 =====
    async loadOrderInfo(orderId) {
      try {
        console.log('주문 정보 로딩 시작 - orderId:', orderId);
        
        const baseUrl = import.meta.env.VITE_BASE_URL || 'http://localhost:8080';
        const response = await fetch(`${baseUrl}/orders/${orderId}`);
        if (!response.ok) {
          throw new Error('주문 정보를 불러올 수 없습니다.');
        }
        
        const orderData = await response.json();
        console.log('주문 정보 로드 완료:', orderData);
        
        // 주문 기본 정보 설정
        this.orderInfo.orderId = orderData.orderId;
        this.orderInfo.productId = orderData.productId;
        this.orderInfo.price = orderData.amount;
        this.orderInfo.amount = orderData.amount;
        this.orderInfo.status = orderData.status;
        this.orderInfo.tossOrderId = orderData.tossOrderId; // 토스페이먼츠용 orderId 설정
        
        // 상품 정보 로드 (productId 기반)
        await this.loadProductInfo(orderData.productId);
        
      } catch (error) {
        console.error('주문 정보 로드 실패:', error);
        alert('주문 정보를 불러오는데 실패했습니다: ' + error.message);
        this.$router.go(-1);
      }
    },

    // ===== 판매자 정보 조회 (결제용) =====
    async loadSellerInfo(sellerId) {
      try {
        console.log('판매자 정보 로딩 - sellerId:', sellerId);
        
        const baseUrl = import.meta.env.VITE_BASE_URL || 'http://localhost:8080';
        const response = await fetch(`${baseUrl}/payment/users/${sellerId}`);
        if (!response.ok) {
          throw new Error('판매자 정보를 불러올 수 없습니다.');
        }
        
        const sellerData = await response.json();
        console.log('판매자 정보 로드 완료:', sellerData);
        
        // 판매자 실제 이름 설정
        this.orderInfo.sellerName = sellerData.name || `판매자 ${sellerId}`;
        
        console.log('판매자 정보 설정 완료');
        
      } catch (error) {
        console.error('판매자 정보 로드 실패:', error);
        // 기본값 사용
        this.orderInfo.sellerName = `판매자 ${sellerId}`;
      }
    },

    // ===== 상품 정보 조회 =====
    async loadProductInfo(productId) {
      try {
        console.log('상품 정보 로딩 - productId:', productId);
        
        const baseUrl = import.meta.env.VITE_BASE_URL || 'http://localhost:8080';
        const response = await fetch(`${baseUrl}/products/payment/${productId}`);
        if (!response.ok) {
          throw new Error('상품 정보를 불러올 수 없습니다.');
        }
        
        const productData = await response.json();
        console.log('상품 정보 로드 완료:', productData);
        
        this.orderInfo.productTitle = productData.title;
        // 첫 번째 이미지 URL 사용 (imageUrls 배열에서)
        this.orderInfo.productImage = (productData.imageUrls && productData.imageUrls.length > 0) 
          ? productData.imageUrls[0] 
          : "https://placehold.co/300x300/333333/FFF?text=No+Image";
        
        // 판매자 실제 이름 가져오기
        await this.loadSellerInfo(productData.sellerId);
        
        // 위치 정보 조합
        let locationParts = [];
        if (productData.siDo) locationParts.push(productData.siDo);
        if (productData.siGunGu) locationParts.push(productData.siGunGu);
        if (productData.eupMyeonDong) locationParts.push(productData.eupMyeonDong);
        if (productData.apartmentName) locationParts.push(productData.apartmentName);
        this.orderInfo.sellerLocation = locationParts.length > 0 ? locationParts.join(' ') : '위치 정보 없음';
        
        console.log('상품 정보 설정 완료');
        
      } catch (error) {
        console.error('상품 정보 로드 실패:', error);
        // 기본값 사용
        this.orderInfo.productTitle = "상품 정보 로드 실패";
        this.orderInfo.productImage = "https://placehold.co/300x300/333333/FFF?text=Error";
        this.orderInfo.sellerName = "알 수 없음";
        this.orderInfo.sellerLocation = "알 수 없음";
      }
    },

    // ===== 구매자 정보 로드 (백엔드 API 호출) =====
    async loadBuyerInfo() {
      try {
        // localStorage에서 userId 가져오기
        const userInfo = JSON.parse(localStorage.getItem('user') || '{}');
        const userId = userInfo.userId || userInfo.id;
        
        if (!userId) {
          throw new Error('사용자 ID를 찾을 수 없습니다.');
        }

        console.log('사용자 정보 API 호출 - userId:', userId);
        
        // 백엔드에서 사용자 정보 조회
        const baseUrl = import.meta.env.VITE_BASE_URL || 'http://localhost:8080';
        const response = await fetch(`${baseUrl}/general-login/users/${userId}`);
        
        if (!response.ok) {
          throw new Error('사용자 정보를 불러올 수 없습니다.');
        }
        
        const userData = await response.json();
        console.log('백엔드에서 받은 사용자 정보:', userData);
        
        // 사용자 정보 매핑
        this.buyerInfo.name = userData.name || '구매자';
        this.buyerInfo.phone = userData.phoneNumber || '010-0000-0000';
        
        // 주소 정보를 아파트명, 동, 호수로 조합
        let addressParts = [];
        // if (userData.siDo) addressParts.push(userData.siDo);
        // if (userData.siGunGu) addressParts.push(userData.siGunGu);
        // if (userData.eupMyeonDong) addressParts.push(userData.eupMyeonDong);
        if (userData.apartmentName) addressParts.push(userData.apartmentName);
        if (userData.buildingDong) addressParts.push(`${userData.buildingDong}동`);
        if (userData.buildingHo) addressParts.push(`${userData.buildingHo}호`);
        
        this.buyerInfo.address = addressParts.length > 0 ? addressParts.join(' ') : '주소 정보 없음';
        
        console.log('구매자 정보 로드 완료:', this.buyerInfo);
        
      } catch (error) {
        console.error('구매자 정보 로드 실패:', error);
        this.buyerInfo.name = '구매자';
        this.buyerInfo.phone = '010-0000-0000';
        this.buyerInfo.address = '주소 정보 없음';
      }
    },
    selectPaymentMethod(method) {
      this.selectedMethod = method;
    },
    async proceedPayment() {
      if (!this.canProceedPayment) return;

      this.isLoading = true;

      try {
        // 토스페이먼츠용 orderId 사용 (tossOrderId)
        const tossOrderId = this.orderInfo.tossOrderId;
        
        if (!tossOrderId) {
          throw new Error('토스페이먼츠 주문 정보가 없습니다.');
        }

        // 토스페이먼츠 결제위젯 초기화
        await this.initializeTossWidget(tossOrderId);
      } catch (error) {
        console.error("결제 처리 중 오류:", error);
        alert("결제 처리 중 오류가 발생했습니다: " + error.message);
      } finally {
        this.isLoading = false;
      }
    },

    async createOrder() {
      // localStorage에서 현재 로그인한 사용자 정보 가져오기
      const userInfo = JSON.parse(localStorage.getItem('user') || '{}');
      const buyerId = userInfo.userId;
      
      if (!buyerId) {
        throw new Error('로그인 정보를 찾을 수 없습니다.');
      }

      // 백엔드 API 호출
      const baseUrl = import.meta.env.VITE_BASE_URL || 'http://localhost:8080';
      const response = await fetch(`${baseUrl}/orders`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          productId: this.orderInfo.productId,
          buyerId: buyerId, // 로그인한 사용자 ID 사용
        }),
      });

      if (!response.ok) {
        throw new Error("주문 생성에 실패했습니다.");
      }


      return await response.json();
    },

    async initializeTossWidget(orderId) {
      try {
        console.log("토스페이먼츠 결제 요청 시작:", {
          orderId,
          amount: this.orderInfo.price,
          orderName: this.orderInfo.productTitle,
          customerName: this.buyerInfo.name,
        });

        // 토스페이먼츠 결제 요청
        await requestPayment({
          amount: this.orderInfo.price,
          orderId: orderId,
          orderName: this.orderInfo.productTitle,
          customerName: this.buyerInfo.name,
          customerEmail: "customer@example.com",
        });

        console.log("토스페이먼츠 결제창 호출 완료");
      } catch (error) {
        console.error("토스페이먼츠 결제 요청 실패:", error);
        throw error;
      }
    },
  },

  async mounted() {
    // 토스페이먼츠 SDK는 requestPayment 함수에서 자동으로 로드됩니다
    
    // 라우트 파라미터에서 orderId 가져오기
    const orderId = this.$route.params.orderId;
    
    if (!orderId) {
      alert('주문 정보가 없습니다.');
      this.$router.go(-1);
      return;
    }
    
    try {
      // 구매자 정보 로드
      this.loadBuyerInfo();
      
      // 주문 정보 로드 (상품 정보 포함)
      await this.loadOrderInfo(orderId);
      
    } catch (error) {
      console.error('페이지 초기화 실패:', error);
      alert('페이지를 불러오는데 실패했습니다.');
      this.$router.go(-1);
    } finally {
      this.isLoadingOrder = false;
    }
  },
};
</script>

<style scoped>
.payment-container {
  width: 100%;
  max-width: 390px;
  margin: 0 auto;
  min-height: 100vh;
  background-color: #f8f9fa;
  box-shadow: rgba(100, 100, 111, 0.2) 0px 7px 29px 0px;
}

.payment-detail-view {
  display: flex;
  flex-direction: column;
  height: 100vh;
  background-color: #f8f9fa;
  position: relative;
  /* 고정 헤더 높이만큼 상단 여백 확보 */
  padding-top: 64px;
}

/* 헤더 스타일 */
.payment-header {
  position: fixed;
  top: 0;
  left: 50%;
  transform: translateX(-50%);
  width: 100%;
  max-width: 390px;
  z-index: 50;
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 12px 16px;
  height: 64px;
  background-color: #4682B4;
  color: #fff;
  box-shadow: 0 4px 12px rgba(0,0,0,0.12);
  border-bottom: none;
}

.close-button {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 40px;
  height: 40px;
  border-radius: 50%;
  background-color: rgba(255,255,255,0.18);
  color: #fff;
  transition: background-color 0.2s ease;
}

.close-button:hover {
  background-color: rgba(255,255,255,0.28);
}

.header-title {
  font-size: 18px;
  font-weight: 600;
  color: #fff;
}

.header-spacer {
  width: 40px;
}

/* 메인 콘텐츠 */
.payment-content {
  flex: 1;
  overflow-y: auto;
  padding: 20px;
}

/* 카드 공통 스타일 */
.order-info-card,
.payment-info-card,
.buyer-info-card,
.payment-method-card,
.terms-card {
  background-color: white;
  border-radius: 12px;
  margin-bottom: 16px;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.06);
  overflow: hidden;
}

.card-header {
  padding: 20px 20px 16px;
  border-bottom: 1px solid #f8f9fa;
}

.card-header h2 {
  font-size: 16px;
  font-weight: 600;
  color: #333;
  margin: 0;
}

/* 주문 정보 카드 */
.product-summary {
  display: flex;
  padding: 20px;
  gap: 16px;
}

.product-image {
  width: 80px;
  height: 80px;
  border-radius: 8px;
  overflow: hidden;
  flex-shrink: 0;
}

.product-image img {
  width: 100%;
  height: 100%;
  object-fit: cover;
}

.product-details {
  flex: 1;
  display: flex;
  flex-direction: column;
  justify-content: center;
}

.product-title {
  font-size: 16px;
  font-weight: 600;
  color: #333;
  margin: 0 0 8px 0;
  line-height: 1.4;
}

.product-price {
  font-size: 18px;
  font-weight: 700;
  color: #2C7A7B;
  margin: 0 0 4px 0;
}

.seller-info {
  font-size: 14px;
  color: #666;
  margin: 0;
  margin-top: 6px;
  font-weight: 500;
}

.seller-info.address {
  margin-top: 8px;
  font-size: 11.5px;
  font-weight: 300;
}

/* 결제 정보 카드 */
.payment-details {
  padding: 20px;
}

.detail-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12px 0;
  border-bottom: 1px solid #f8f9fa;
}

.detail-row:last-child {
  border-bottom: none;
}

.total-row {
  padding-top: 16px;
  border-top: 1px solid #e9ecef;
  margin-top: 8px;
}

.label {
  font-size: 14px;
  color: #666;
}

.value {
  font-size: 14px;
  font-weight: 500;
  color: #333;
}

.total-amount {
  font-size: 16px;
  font-weight: 700;
  color: #2C7A7B;
}

/* 구매자 정보 카드 */
.buyer-details {
  padding: 20px;
}

/* 결제 수단 카드 */
.payment-methods {
  padding: 20px;
}

.method-item {
  display: flex;
  align-items: center;
  padding: 16px;
  border: 2px solid #f0f0f0;
  border-radius: 8px;
  margin-bottom: 12px;
  cursor: pointer;
  transition: all 0.2s;
}

.method-item:last-child {
  margin-bottom: 0;
}

.method-item.active {
  border-color: #4682b4;
  background-color: #f0f7ff;
}

.method-icon {
  width: 40px;
  height: 40px;
  display: flex;
  align-items: center;
  justify-content: center;
  background-color: #f8f9fa;
  border-radius: 8px;
  margin-right: 16px;
  color: #666;
}

.method-item.active .method-icon {
  background-color: #4682b4;
  color: white;
}

.method-info {
  flex: 1;
  display: flex;
  flex-direction: column;
}

.method-name {
  font-size: 16px;
  font-weight: 500;
  color: #333;
  margin-bottom: 4px;
}

.method-desc {
  font-size: 14px;
  color: #666;
}

.method-radio {
  width: 20px;
  height: 20px;
  border: 2px solid #ddd;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: all 0.2s;
}

.method-radio.active {
  border-color: #4682b4;
}

.radio-dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background-color: #4682b4;
  opacity: 0;
  transition: opacity 0.2s;
}

.method-radio.active .radio-dot {
  opacity: 1;
}

/* 약관 동의 카드 */
.terms-card {
  padding: 20px;
}

.terms-checkbox {
  display: flex;
  align-items: flex-start;
  gap: 12px;
}

.terms-checkbox input[type="checkbox"] {
  display: none;
}

.checkbox-custom {
  width: 20px;
  height: 20px;
  border: 2px solid #ddd;
  border-radius: 4px;
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
  transition: all 0.2s;
  cursor: pointer;
}

.checkbox-custom.checked {
  background-color: #4682b4;
  border-color: #4682b4;
  color: white;
}

.terms-text {
  font-size: 14px;
  line-height: 1.5;
  color: #666;
}

.terms-text strong {
  color: #333;
}

/* 하단 결제 버튼 */
.payment-footer {
  background-color: white;
  border-top: 1px solid #f0f0f0;
  padding: 12px 20px;
  box-shadow: 0 -2px 8px rgba(0, 0, 0, 0.06);
  flex-shrink: 0;
}

.payment-summary {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 8px;
}

.total-label {
  font-size: 13px;
  color: #666;
}

.total-price {
  font-size: 16px;
  font-weight: 700;
  color: #0097A7;
}

.payment-button {
  width: 100%;
  background-color: #4682b4;
  color: white;
  border: none;
  border-radius: 8px;
  padding: 14px;
  font-size: 15px;
  font-weight: 600;
  cursor: pointer;
  transition: background-color 0.2s;
}

.payment-button:hover:not(.disabled) {
  background-color: #3a6b9a;
}

.payment-button.disabled {
  background-color: #8888;
  cursor: not-allowed;
}

/* 로딩 오버레이 */
.loading-overlay {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: rgba(0, 0, 0, 0.5);
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  z-index: 1000;
  color: white;
}

.loading-spinner {
  width: 40px;
  height: 40px;
  border: 4px solid rgba(255, 255, 255, 0.3);
  border-top: 4px solid white;
  border-radius: 50%;
  animation: spin 1s linear infinite;
  margin-bottom: 16px;
}

@keyframes spin {
  0% {
    transform: rotate(0deg);
  }
  100% {
    transform: rotate(360deg);
  }
}

.loading-overlay p {
  font-size: 16px;
  margin: 0;
}

/* 스크롤바 숨기기 */
.payment-content::-webkit-scrollbar {
  display: none;
}

.payment-content {
  -ms-overflow-style: none;
  scrollbar-width: none;
}
</style>
