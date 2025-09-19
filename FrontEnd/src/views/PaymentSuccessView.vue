<template>
  <div class="payment-container">
    <div class="payment-success-view">
      <!-- 헤더 -->
      <header class="success-header">
        <h1 class="header-title">결제 완료</h1>
      </header>

      <!-- 메인 콘텐츠 -->
      <main class="success-content">
        <!-- 로딩 상태 -->
        <div v-if="isLoading" class="loading-container">
          <div class="loading-spinner"></div>
          <p>결제 정보를 불러오는 중...</p>
        </div>

        <!-- 결제 완료 내용 -->
        <div v-else>
          <!-- 성공 아이콘 -->
          <div class="success-icon">
            <svg
              xmlns="http://www.w3.org/2000/svg"
              width="80"
              height="80"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              stroke-width="2"
              stroke-linecap="round"
              stroke-linejoin="round"
            >
              <path d="M22 11.08V12a10 10 0 1 1-5.93-9.14"></path>
              <polyline points="22 4 12 14.01 9 11.01"></polyline>
            </svg>
          </div>

          <!-- 성공 메시지 -->
          <div class="success-message">
            <h2>결제가 완료되었습니다!</h2>
            <p>주문이 정상적으로 처리되었습니다.</p>
          </div>

          <!-- 결제 정보 카드 -->
          <section class="payment-info-card">
            <div class="card-header">
              <h3>결제 정보</h3>
            </div>
            <div class="payment-details">
              <div class="detail-row">
                <span class="label">주문 번호</span>
                <span class="value">{{ paymentInfo.orderId }}</span>
              </div>
              <div class="detail-row">
                <span class="label">결제 금액</span>
                <span class="value"
                  >{{ formatPrice(paymentInfo.amount) }}원</span
                >
              </div>
              <div class="detail-row">
                <span class="label">결제 수단</span>
                <span class="value">{{ paymentInfo.method }}</span>
              </div>
              <div class="detail-row">
                <span class="label">결제 일시</span>
                <span class="value">{{ formatDate(paymentInfo.paidAt) }}</span>
              </div>
            </div>
          </section>

          <!-- 상품 정보 카드 -->
          <section class="product-info-card">
            <div class="card-header">
              <h3>주문 상품</h3>
            </div>
            <div class="product-summary">
              <div class="product-image">
                <img :src="productInfo.image" :alt="productInfo.title" />
              </div>
              <div class="product-details">
                <h4 class="product-title">{{ productInfo.title }}</h4>
                <p class="product-price">
                  {{ formatPrice(productInfo.price) }}원
                </p>
                <p class="seller-info">
                  판매자: {{ productInfo.sellerName }}
                  {{ productInfo.sellerLocation }}
                </p>
              </div>
            </div>
          </section>

          <!-- 다음 단계 안내 -->
          <section class="next-steps-card">
            <div class="card-header">
              <h3>다음 단계</h3>
            </div>
            <div class="steps-list">
              <div class="step-item">
                <div class="step-icon">
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
                      d="M21 16V8a2 2 0 0 0-1-1.73l-7-4a2 2 0 0 0-2 0l-7 4A2 2 0 0 0 3 8v8a2 2 0 0 0 1 1.73l7 4a2 2 0 0 0 2 0l7-4A2 2 0 0 0 21 16z"
                    ></path>
                    <polyline points="3.27 6.96 12 12.01 20.73 6.96"></polyline>
                    <line x1="12" y1="22.08" x2="12" y2="12"></line>
                  </svg>
                </div>
                <div class="step-content">
                  <h4>배송 준비</h4>
                  <p>판매자가 상품을 포장하고 배송을 준비합니다.</p>
                </div>
              </div>

              <div class="step-item">
                <div class="step-icon">
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
                    <circle cx="12" cy="12" r="10"></circle>
                    <polyline points="12 6 12 12 16 14"></polyline>
                  </svg>
                </div>
                <div class="step-content">
                  <h4>배송 시작</h4>
                  <p>로봇이 상품을 픽업하여 배송을 시작합니다.</p>
                </div>
              </div>

              <div class="step-item">
                <div class="step-icon">
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
                    <path d="M22 11.08V12a10 10 0 1 1-5.93-9.14"></path>
                    <polyline points="22 4 12 14.01 9 11.01"></polyline>
                  </svg>
                </div>
                <div class="step-content">
                  <h4>배송 완료</h4>
                  <p>상품이 안전하게 배송됩니다.</p>
                </div>
              </div>
            </div>
          </section>
        </div>
      </main>

      <!-- 하단 버튼들 -->
      <footer class="success-footer">
        <!-- <button class="secondary-button" @click="goToOrderList">
          주문 내역 보기
        </button> -->
        <button class="primary-button" @click="goToHome">홈으로 가기</button>
      </footer>
    </div>
  </div>
</template>

<script>
export default {
  name: "PaymentSuccessView",
  data() {
    return {
      isLoading: true,
      paymentInfo: {
        orderId: "",
        tossOrderId: "",
        amount: 0,
        method: "",
        paidAt: null,
      },
      productInfo: {
        title: "",
        image: "https://placehold.co/300x300/333333/FFF?text=Loading",
        price: 0,
        sellerName: "",
        sellerLocation: "",
      },
    };
  },
  async mounted() {
    await this.loadPaymentResult();
  },
  methods: {
    async loadPaymentResult() {
      try {
        // URL 파라미터에서 결제 정보 추출
        const urlParams = new URLSearchParams(window.location.search);
        const orderId = urlParams.get("orderId");
        const paymentKey = urlParams.get("paymentKey");
        const amount = urlParams.get("amount");

        if (!orderId || !paymentKey || !amount) {
          throw new Error("결제 정보가 부족합니다.");
        }

        // 주문 정보 로드
        await this.loadOrderInfo(orderId);

        // 백엔드로 결제 승인 요청 전송 (실패해도 화면 유지)
        try {
          await this.confirmPaymentBackend({
            paymentKey,
            orderId, // tossOrderId (string)
            amount: Number(amount),
          });
        } catch (e) {
          console.error("결제 승인 API 실패:", e);
        }

        // 결제 정보 설정
        this.paymentInfo.orderId = orderId;
        this.paymentInfo.amount = parseInt(amount);
        this.paymentInfo.method = "카드 결제"; // 토스페이먼츠에서 실제 결제 수단 정보를 받아올 수 있음
        this.paymentInfo.paidAt = new Date();
      } catch (error) {
        console.error("결제 결과 로드 실패:", error);
        alert("결제 정보를 불러오는데 실패했습니다: " + error.message);
        // 라우터 무한 리다이렉트 방지: 에러 시에도 현재 페이지 유지
      } finally {
        this.isLoading = false;
      }
    },
    async confirmPaymentBackend(payload) {
      const baseUrl = import.meta.env.VITE_BASE_URL || "http://localhost:8080";
      const res = await fetch(`${baseUrl}/payments/confirm`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
      if (!res.ok) {
        const text = await res.text();
        throw new Error(`결제 승인 실패: ${res.status} ${text}`);
      }
    },

    async loadOrderInfo(orderId) {
      try {
        const baseUrl =
          import.meta.env.VITE_BASE_URL || "http://localhost:8080";
        const response = await fetch(`${baseUrl}/orders/toss/${orderId}`);

        if (!response.ok) {
          throw new Error(`주문 정보 조회 실패: ${response.status}`);
        }

        const orderData = await response.json();
        console.log("주문 정보 로드 완료:", orderData);

        // 주문 기본 정보 설정
        this.paymentInfo.tossOrderId = orderData.tossOrderId;

        // 상품 정보 로드
        await this.loadProductInfo(orderData.productId);
      } catch (error) {
        console.error("주문 정보 로드 실패:", error);
        throw error;
      }
    },

    async loadProductInfo(productId) {
      try {
        const baseUrl =
          import.meta.env.VITE_BASE_URL || "http://localhost:8080";
        const response = await fetch(
          `${baseUrl}/products/payment/${productId}`
        );

        if (!response.ok) {
          throw new Error(`상품 정보 조회 실패: ${response.status}`);
        }

        const productData = await response.json();
        console.log("상품 정보 로드 완료:", productData);

        // 상품 정보 설정
        this.productInfo.title = productData.title;
        this.productInfo.price = productData.price;

        // 이미지 URL 처리
        if (productData.imageUrl) {
          const baseUrl =
            import.meta.env.VITE_BASE_URL || "http://localhost:8080";
          this.productInfo.image = productData.imageUrl.startsWith("http")
            ? productData.imageUrl
            : `${baseUrl}${productData.imageUrl}`;
        }

        // 판매자 정보 로드
        await this.loadSellerInfo(productData.sellerId);
      } catch (error) {
        console.error("상품 정보 로드 실패:", error);
        throw error;
      }
    },

    async loadSellerInfo(sellerId) {
      try {
        const baseUrl =
          import.meta.env.VITE_BASE_URL || "http://localhost:8080";
        const response = await fetch(`${baseUrl}/payment/users/${sellerId}`);

        if (!response.ok) {
          throw new Error(`판매자 정보 조회 실패: ${response.status}`);
        }

        const sellerData = await response.json();
        console.log("판매자 정보 로드 완료:", sellerData);

        // 판매자 정보 설정
        this.productInfo.sellerName = sellerData.name || `판매자 ${sellerId}`;
        const verified = sellerData.verifiedAddress || null;
        const composed = this.composeAddressFromFields(sellerData);
        this.productInfo.sellerLocation =
          verified || composed || "위치 정보 없음";
      } catch (error) {
        console.error("판매자 정보 로드 실패:", error);
        // 실패 시 기본값 설정
        this.productInfo.sellerName = `판매자 ${sellerId}`;
        this.productInfo.sellerLocation = "위치 정보 없음";
      }
    },
    composeAddressFromFields(user) {
      const parts = [];
      if (user.siDo) parts.push(user.siDo);
      if (user.siGunGu) parts.push(user.siGunGu);
      if (user.eupMyeonDong) parts.push(user.eupMyeonDong);
      if (user.apartmentName) parts.push(user.apartmentName);
      return parts.length ? parts.join(" ") : null;
    },

    formatAddress(address) {
      if (!address) return null;

      try {
        const addressObj =
          typeof address === "string" ? JSON.parse(address) : address;
        return `${addressObj.dong || ""} ${addressObj.ho || ""}`.trim() || null;
      } catch (error) {
        console.error("주소 파싱 오류:", error);
        return typeof address === "string" ? address : null;
      }
    },

    formatPrice(price) {
      return price.toLocaleString("ko-KR");
    },
    formatDate(date) {
      return new Date(date).toLocaleString("ko-KR", {
        year: "numeric",
        month: "2-digit",
        day: "2-digit",
        hour: "2-digit",
        minute: "2-digit",
      });
    },
    goToOrderList() {
      // 주문 내역 페이지로 이동
      this.$router.push("/orders");
    },
    goToHome() {
      // 홈 페이지로 이동
      this.$router.push("/home");
    },
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

.payment-success-view {
  display: flex;
  flex-direction: column;
  height: 100vh;
  background-color: #f8f9fa;
}

/* 헤더 */
.success-header {
  padding: 16px 20px;
  background-color: white;
  border-bottom: 1px solid #f0f0f0;
  text-align: center;
}

.header-title {
  font-size: 18px;
  font-weight: 600;
  color: #333;
  margin: 0;
}

/* 메인 콘텐츠 */
.success-content {
  flex: 1;
  overflow-y: auto;
  padding: 20px;
}

/* 성공 아이콘 */
.success-icon {
  display: flex;
  justify-content: center;
  margin: 40px 0 24px;
  color: #28a745;
}

/* 성공 메시지 */
.success-message {
  text-align: center;
  margin-bottom: 32px;
}

.success-message h2 {
  font-size: 24px;
  font-weight: 700;
  color: #333;
  margin: 0 0 8px 0;
}

.success-message p {
  font-size: 16px;
  color: #666;
  margin: 0;
}

/* 카드 공통 스타일 */
.payment-info-card,
.product-info-card,
.next-steps-card {
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

.card-header h3 {
  font-size: 16px;
  font-weight: 600;
  color: #333;
  margin: 0;
}

/* 결제 정보 */
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

.label {
  font-size: 14px;
  color: #666;
}

.value {
  font-size: 14px;
  font-weight: 500;
  color: #333;
}

/* 상품 정보 */
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

/* 다음 단계 */
.steps-list {
  padding: 20px;
}

.step-item {
  display: flex;
  align-items: flex-start;
  gap: 16px;
  margin-bottom: 24px;
}

.step-item:last-child {
  margin-bottom: 0;
}

.step-icon {
  width: 40px;
  height: 40px;
  display: flex;
  align-items: center;
  justify-content: center;
  background-color: #f8f9fa;
  border-radius: 8px;
  color: #666;
  flex-shrink: 0;
}

.step-content h4 {
  font-size: 16px;
  font-weight: 600;
  color: #333;
  margin: 0 0 4px 0;
}

.step-content p {
  font-size: 14px;
  color: #666;
  margin: 0;
  line-height: 1.4;
}

/* 하단 버튼들 */
.success-footer {
  background-color: white;
  border-top: 1px solid #f0f0f0;
  padding: 12px 20px;
  box-shadow: 0 -2px 8px rgba(0, 0, 0, 0.06);
  display: flex;
  gap: 12px;
  flex-shrink: 0;
}

.secondary-button {
  flex: 1;
  background-color: #f8f9fa;
  color: #666;
  border: 1px solid #e9ecef;
  border-radius: 8px;
  padding: 14px;
  font-size: 15px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s;
}

.secondary-button:hover {
  background-color: #e9ecef;
}

.primary-button {
  flex: 1;
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

.primary-button:hover {
  background-color: #3a6b9a;
}

/* 로딩 상태 */
.loading-container {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 60px 20px;
  text-align: center;
}

.loading-spinner {
  width: 40px;
  height: 40px;
  border: 4px solid #f3f3f3;
  border-top: 4px solid #4682b4;
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

.loading-container p {
  font-size: 16px;
  color: #666;
  margin: 0;
}

/* 스크롤바 숨기기 */
.success-content::-webkit-scrollbar {
  display: none;
}

.success-content {
  -ms-overflow-style: none;
  scrollbar-width: none;
}
</style>
