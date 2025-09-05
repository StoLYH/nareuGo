<template>
  <div class="delivery-cards">
    <!-- 섹션 제목 -->
    <div class="section-title">나르고 위치 조회</div>
    
    <div class="cards-container">
      <!-- 택배 현황 카드 1 -->
      <div class="delivery-card">
        <div class="card-header">
          <div class="delivery-icon">📦</div>
          <div class="delivery-info">
            <div class="delivery-company">CJ대한통운</div>
            <div class="delivery-number">#HWDSF776567DS</div>
          </div>
        </div>
        <div class="card-content">
          <div class="status-info">
            <div class="status-text">배송 중</div>
            <div class="status-time">09/02 10:16</div>
          </div>
          <div class="progress-bar">
            <div class="progress-step completed">
              <div class="step-dot"></div>
              <div class="step-label">접수완료</div>
            </div>
            <div class="progress-line completed"></div>
            <div class="progress-step completed">
              <div class="step-dot"></div>
              <div class="step-label">배달시작</div>
            </div>
            <div class="progress-line completed"></div>
            <div class="progress-step completed">
              <div class="step-dot"></div>
              <div class="step-label">배달중</div>
            </div>
            <div class="progress-line completed"></div>
            <div class="progress-step active">
              <div class="step-dot"></div>
              <div class="step-label">완료</div>
            </div>
          </div>
        </div>
      </div>

      <!-- 택배 현황 카드 2 -->
      <div class="delivery-card">
        <div class="card-header">
          <div class="delivery-icon">📦</div>
          <div class="delivery-info">
            <div class="delivery-company">한진택배</div>
            <div class="delivery-number">#HJ9876543210987</div>
          </div>
        </div>
        <div class="card-content">
          <div class="status-info">
            <div class="status-text">집하장 도착</div>
            <div class="status-time">09/02 09:15</div>
          </div>
          <div class="progress-bar">
            <div class="progress-step completed">
              <div class="step-dot"></div>
              <div class="step-label">접수완료</div>
            </div>
            <div class="progress-line"></div>
            <div class="progress-step">
              <div class="step-dot"></div>
              <div class="step-label">배달시작</div>
            </div>
            <div class="progress-line"></div>
            <div class="progress-step">
              <div class="step-dot"></div>
              <div class="step-label">배달중</div>
            </div>
            <div class="progress-line"></div>
            <div class="progress-step">
              <div class="step-dot"></div>
              <div class="step-label">완료</div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 최근 배달 내역 섹션 -->
    <div class="delivery-history-section">
      <div class="history-title">최근 배달 내역</div>
      
      <div class="history-cards">
        <!-- 배달 완료 카드 1 -->
        <div class="history-card">
          <div class="card-header">
            <div class="delivery-icon">📦</div>
            <div class="delivery-info">
              <div class="delivery-number">#S65F1ES51F2FD</div>
              <div class="delivery-date">09/02 10:16</div>
            </div>
          </div>
        </div>

        <!-- 배달 완료 카드 2 -->
        <div class="history-card">
          <div class="card-header">
            <div class="delivery-icon">📦</div>
            <div class="delivery-info">
              <div class="delivery-number">#GB1BF56T1BTF2</div>
              <div class="delivery-date">08/22 08:35</div>
            </div>
          </div>
        </div>

        <!-- 배달 완료 카드 3 -->
        <div class="history-card">
          <div class="card-header">
            <div class="delivery-icon">📦</div>
            <div class="delivery-info">
              <div class="delivery-number">#G1R56DG156DR</div>
              <div class="delivery-date">08/20 17:45</div>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
// 택배 현황 카드 컴포넌트
</script>

<style scoped>
.delivery-cards {
  padding: 24px 20px;
  background-color: white;
  margin-top: -10px;
  border-radius: 24px 24px 0 0;
  min-height: calc(100vh - 320px);
}

.section-title {
  font-size: 16px;
  font-weight: 600;
  color: #2c3e50;
  margin-bottom: 20px;
  margin-top: 20px;
  text-align: left;
}

.cards-container {
  display: flex;
  flex-direction: column;
  gap: 20px;
  margin-bottom: 40px;
}

.delivery-card {
  background-color: white;
  border: 1px solid #f1f3f4;
  border-radius: 16px;
  padding: 20px;
  box-shadow: 0 2px 12px rgba(0, 0, 0, 0.08);
  transition: all 0.3s ease;
}

.delivery-card:hover {
  transform: translateY(-2px);
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.12);
}

.card-header {
  display: flex;
  align-items: center;
  margin-bottom: 20px;
}

.delivery-icon {
  font-size: 24px;
  margin-right: 16px;
}

.delivery-info {
  flex: 1;
}

.delivery-company {
  font-size: 16px;
  font-weight: 600;
  color: #2c3e50;
  margin-bottom: 6px;
}

.delivery-number {
  font-size: 14px;
  color: #6c757d;
  font-family: 'Courier New', monospace;
  font-weight: 500;
}

.card-content {
  margin-bottom: 0;
}

.status-info {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 20px;
}

.status-text {
  font-size: 15px;
  font-weight: 600;
  color: #007bff;
}

.status-time {
  font-size: 13px;
  color: #6c757d;
  font-weight: 500;
}

.progress-bar {
  display: flex;
  align-items: center;
  justify-content: space-between;
  position: relative;
}

.progress-step {
  display: flex;
  flex-direction: column;
  align-items: center;
  flex: 1;
  position: relative;
}

.step-dot {
  width: 12px;
  height: 12px;
  border-radius: 50%;
  background-color: #e9ecef;
  border: 2px solid #e9ecef;
  margin-bottom: 8px;
  transition: all 0.3s ease;
}

.progress-step.completed .step-dot {
  background-color: #007bff;
  border-color: #007bff;
}

.progress-step.active .step-dot {
  background-color: #007bff;
  border-color: #007bff;
  box-shadow: 0 0 0 4px rgba(0, 123, 255, 0.2);
}

.step-label {
  font-size: 10px;
  color: #6c757d;
  font-weight: 500;
  text-align: center;
  line-height: 1.1;
  white-space: nowrap;
}

.progress-step.completed .step-label {
  color: #007bff;
  font-weight: 600;
}

.progress-step.active .step-label {
  color: #007bff;
  font-weight: 600;
}

.progress-line {
  height: 2px;
  background-color: #e9ecef;
  flex: 1;
  margin: 0 8px;
  margin-top: -16px;
  transition: all 0.3s ease;
}

.progress-line.completed {
  background-color: #007bff;
}

/* 최근 배달 내역 섹션 */
.delivery-history-section {
  margin-top: 20px;
}

.history-title {
  font-size: 16px;
  font-weight: 600;
  color: #2c3e50;
  margin-bottom: 16px;
  text-align: left;
}

.history-cards {
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.history-card {
  background-color: white;
  border: 1px solid #f1f3f4;
  border-radius: 12px;
  padding: 16px;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.08);
  transition: all 0.3s ease;
}

.history-card:hover {
  transform: translateY(-2px);
  box-shadow: 0 4px 16px rgba(0, 0, 0, 0.12);
}

.history-card .card-header {
  margin-bottom: 0;
  gap: 12px;
}

.history-card .delivery-icon {
  font-size: 20px;
  margin-right: 0;
}

.history-card .delivery-number {
  font-size: 14px;
  color: #2c3e50;
  font-family: 'Courier New', monospace;
  font-weight: 500;
  margin-bottom: 4px;
}

.history-card .delivery-date {
  font-size: 12px;
  color: #6c757d;
  font-weight: 400;
}
</style>
