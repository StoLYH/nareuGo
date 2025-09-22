<template>
  <div class="payment-container">
    <div class="payment-fail-view">
      <!-- 헤더 -->
      <header class="fail-header">
        <h1 class="header-title">결제 실패</h1>
      </header>

      <!-- 메인 콘텐츠 -->
      <main class="fail-content">
        <!-- 실패 아이콘 -->
        <div class="fail-icon">
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
            <circle cx="12" cy="12" r="10"></circle>
            <line x1="15" y1="9" x2="9" y2="15"></line>
            <line x1="9" y1="9" x2="15" y2="15"></line>
          </svg>
        </div>

        <!-- 실패 메시지 -->
        <div class="fail-message">
          <h2>결제에 실패했습니다</h2>
          <p>{{ errorMessage }}</p>
        </div>

        <!-- 오류 정보 카드 -->
        <section class="error-info-card">
          <div class="card-header">
            <h3>오류 정보</h3>
          </div>
          <div class="error-details">
            <div class="detail-row">
              <span class="label">오류 코드</span>
              <span class="value">{{ errorInfo.code }}</span>
            </div>
            <div class="detail-row">
              <span class="label">오류 메시지</span>
              <span class="value">{{ errorInfo.message }}</span>
            </div>
            <div class="detail-row">
              <span class="label">발생 시간</span>
              <span class="value">{{ formatDate(errorInfo.timestamp) }}</span>
            </div>
          </div>
        </section>

        <!-- 해결 방법 안내 -->
        <section class="solution-card">
          <div class="card-header">
            <h3>해결 방법</h3>
          </div>
          <div class="solution-list">
            <div class="solution-item">
              <div class="solution-icon">
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
                  <path d="M9 12l2 2 4-4"></path>
                  <path d="M21 12c-1 0-3-1-3-3s2-3 3-3 3 1 3 3-2 3-3 3"></path>
                  <path d="M3 12c1 0 3-1 3-3s-2-3-3-3-3 1-3 3 2 3 3 3"></path>
                  <path d="M12 3c0 1-1 3-3 3s-3-2-3-3 1-3 3-3 3 2 3 3"></path>
                  <path d="M12 21c0-1 1-3 3-3s3 2 3 3-1 3-3 3-3-2-3-3"></path>
                </svg>
              </div>
              <div class="solution-content">
                <h4>카드 정보 확인</h4>
                <p>카드 번호, 유효기간, CVC 번호를 다시 확인해주세요.</p>
              </div>
            </div>

            <div class="solution-item">
              <div class="solution-icon">
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
                    d="M22 16.92v3a2 2 0 0 1-2.18 2 19.79 19.79 0 0 1-8.63-3.07 19.5 19.5 0 0 1-6-6 19.79 19.79 0 0 1-3.07-8.67A2 2 0 0 1 4.11 2h3a2 2 0 0 1 2 1.72 12.84 12.84 0 0 0 .7 2.81 2 2 0 0 1-.45 2.11L8.09 9.91a16 16 0 0 0 6 6l1.27-1.27a2 2 0 0 1 2.11-.45 12.84 12.84 0 0 0 2.81.7A2 2 0 0 1 22 16.92z"
                  ></path>
                </svg>
              </div>
              <div class="solution-content">
                <h4>카드사 문의</h4>
                <p>카드사에 결제 제한 여부를 확인해보세요.</p>
              </div>
            </div>

            <div class="solution-item">
              <div class="solution-icon">
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
              <div class="solution-content">
                <h4>다른 결제 수단</h4>
                <p>계좌이체나 다른 카드로 결제를 시도해보세요.</p>
              </div>
            </div>
          </div>
        </section>
      </main>

      <!-- 하단 버튼들 -->
      <footer class="fail-footer">
        <button class="secondary-button" @click="goBack">이전으로</button>
        <button class="primary-button" @click="retryPayment">
          다시 결제하기
        </button>
      </footer>
    </div>
  </div>
</template>

<script>
export default {
  name: "PaymentFailView",
  data() {
    return {
      errorMessage: "결제 처리 중 오류가 발생했습니다.",
      errorInfo: {
        code: "PAYMENT_FAILED",
        message: "카드 승인이 거절되었습니다.",
        timestamp: new Date(),
      },
    };
  },
  methods: {
    formatDate(date) {
      return new Date(date).toLocaleString("ko-KR", {
        year: "numeric",
        month: "2-digit",
        day: "2-digit",
        hour: "2-digit",
        minute: "2-digit",
      });
    },
    goBack() {
      this.$router.go(-1);
    },
    retryPayment() {
      // 결제 페이지로 다시 이동
      this.$router.push("/payment");
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

.payment-fail-view {
  display: flex;
  flex-direction: column;
  height: 100vh;
  background-color: #f8f9fa;
  /* 고정 헤더 높이만큼 상단 여백 확보 */
  padding-top: 64px;
}

/* 헤더 */
.fail-header {
  position: fixed;
  top: 0;
  left: 50%;
  transform: translateX(-50%);
  width: 100%;
  max-width: 390px;
  z-index: 50;
  padding: 12px 16px;
  height: 64px;
  background-color: #4682B4;
  color: #fff;
  display: flex;
  align-items: center;
  justify-content: center;
  box-shadow: 0 4px 12px rgba(0,0,0,0.12);
  border-bottom: none;
}

.header-title {
  font-size: 18px;
  font-weight: 600;
  color: #fff;
  margin: 0;
}

/* 메인 콘텐츠 */
.fail-content {
  flex: 1;
  overflow-y: auto;
  padding: 20px;
}

/* 실패 아이콘 */
.fail-icon {
  display: flex;
  justify-content: center;
  margin: 40px 0 24px;
  color: var(--warning, #fc6d6d);
}

/* 실패 메시지 */
.fail-message {
  text-align: center;
  margin-bottom: 32px;
}

.fail-message h2 {
  font-size: 24px;
  font-weight: 700;
  color: #333;
  margin: 0 0 8px 0;
}

.fail-message p {
  font-size: 16px;
  color: #666;
  margin: 0;
}

/* 카드 공통 스타일 */
.error-info-card,
.solution-card {
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

/* 오류 정보 */
.error-details {
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

/* 해결 방법 */
.solution-list {
  padding: 20px;
}

.solution-item {
  display: flex;
  align-items: flex-start;
  gap: 16px;
  margin-bottom: 24px;
}

.solution-item:last-child {
  margin-bottom: 0;
}

.solution-icon {
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

.solution-content h4 {
  font-size: 16px;
  font-weight: 600;
  color: #333;
  margin: 0 0 4px 0;
}

.solution-content p {
  font-size: 14px;
  color: #666;
  margin: 0;
  line-height: 1.4;
}

/* 하단 버튼들 */
.fail-footer {
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

/* 스크롤바 숨기기 */
.fail-content::-webkit-scrollbar {
  display: none;
}

.fail-content {
  -ms-overflow-style: none;
  scrollbar-width: none;
}
</style>
