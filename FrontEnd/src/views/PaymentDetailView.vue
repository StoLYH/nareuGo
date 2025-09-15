<template>
  <div class="payment-container">
    <div class="payment-detail-view">
      <!-- 상단 헤더 -->
      <header class="payment-header">
        <button @click="goBack" class="back-button">
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
            <polyline points="15 18 9 12 15 6"></polyline>
          </svg>
        </button>
        <h1 class="header-title">결제하기</h1>
        <div class="header-spacer"></div>
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
                {{ orderInfo.sellerName }} · {{ orderInfo.sellerLocation }}
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
      orderInfo: {
        productId: 1,
        productTitle: "소니 Wh-1000xm5 실버 팝니다.",
        productImage: "https://placehold.co/300x300/333333/FFF?text=Product",
        price: 360000,
        sellerName: "감성탐방러",
        sellerLocation: "102동 501호",
      },
      buyerInfo: {
        name: "홍길동",
        phone: "010-1234-5678",
        address: "OO마을 12단지 102동 501호",
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
    goBack() {
      this.$router.go(-1);
    },
    selectPaymentMethod(method) {
      this.selectedMethod = method;
    },
    async proceedPayment() {
      if (!this.canProceedPayment) return;

      this.isLoading = true;

      try {
        // 백엔드가 실행 중이면 실제 주문 생성, 아니면 테스트용 주문 ID 사용
        let orderId;
        try {
          const orderResponse = await this.createOrder();
          orderId = orderResponse.orderId;
        } catch (error) {
          console.warn("백엔드 API 호출 실패, 테스트 모드로 진행:", error);
          // 테스트용 주문 ID 생성 (타임스탬프 기반)
          orderId = `test_${Date.now()}`;
        }

        // 토스페이먼츠 결제위젯 초기화
        await this.initializeTossWidget(orderId);
      } catch (error) {
        console.error("결제 처리 중 오류:", error);
        alert("결제 처리 중 오류가 발생했습니다: " + error.message);
      } finally {
        this.isLoading = false;
      }
    },

    async createOrder() {
      // 백엔드 API 호출
      const response = await fetch("/api/orders", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          productId: this.orderInfo.productId,
          buyerId: 1, // 실제로는 로그인한 사용자 ID
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

  mounted() {
    // 토스페이먼츠 SDK는 requestPayment 함수에서 자동으로 로드됩니다
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
}

/* 헤더 스타일 */
.payment-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 16px 20px;
  background-color: white;
  border-bottom: 1px solid #f0f0f0;
  position: sticky;
  top: 0;
  z-index: 10;
}

.back-button {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 40px;
  height: 40px;
  border-radius: 50%;
  background-color: #f8f8f8;
  color: #666;
  transition: background-color 0.2s;
}

.back-button:hover {
  background-color: #f0f0f0;
}

.header-title {
  font-size: 18px;
  font-weight: 600;
  color: #333;
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
  color: #4682b4;
  margin: 0 0 4px 0;
}

.seller-info {
  font-size: 14px;
  color: #666;
  margin: 0;
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
  color: #4682b4;
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
  color: #4682b4;
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
  background-color: var(--disabled, #dcb6b6);
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
