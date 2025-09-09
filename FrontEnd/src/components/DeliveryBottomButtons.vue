<template>
  <div class="delivery-bottom-buttons">
    <div class="buttons-container">
      <!-- 새 택배 등록 버튼 -->
      <button class="main-action-btn">
        <div class="btn-icon">➕</div>
        <div class="btn-text">새 택배 등록</div>
      </button>

      <!-- 빠른 조회 버튼 -->
      <button class="secondary-btn">
        <div class="btn-icon">🔍</div>
        <div class="btn-text">빠른 조회</div>
      </button>

      <!-- 알림 설정 버튼 -->
      <button class="secondary-btn">
        <div class="btn-icon">🔔</div>
        <div class="btn-text">알림 설정</div>
      </button>

      <!-- 배송 통계 버튼 -->
      <button class="secondary-btn">
        <div class="btn-icon">📊</div>
        <div class="btn-text">배송 통계</div>
      </button>
    </div>
  </div>
</template>

<script setup>
// 하단 버튼 컴포넌트
</script>

<style scoped>
.delivery-bottom-buttons {
  padding: 20px;
  background-color: white;
  border-top: 1px solid #e9ecef;
}

.buttons-container {
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.main-action-btn {
  width: 100%;
  background-color: #007bff;
  color: white;
  border: none;
  border-radius: 12px;
  padding: 16px 20px;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 12px;
  font-size: 16px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.3s ease;
  box-shadow: 0 4px 16px rgba(0, 123, 255, 0.4);
}

.main-action-btn:hover {
  background-color: #0056b3;
  transform: translateY(-2px);
  box-shadow: 0 6px 20px rgba(0, 123, 255, 0.5);
}

.main-action-btn:active {
  transform: translateY(0);
  box-shadow: 0 4px 16px rgba(0, 123, 255, 0.4);
}

.secondary-btn {
  width: 100%;
  background-color: white;
  color: #495057;
  border: 2px solid #007bff;
  border-radius: 12px;
  padding: 14px 20px;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 12px;
  font-size: 15px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.3s ease;
  box-shadow: 0 2px 8px rgba(0, 123, 255, 0.2);
}

.secondary-btn:hover {
  background-color: #007bff;
  color: white;
  transform: translateY(-2px);
  box-shadow: 0 4px 12px rgba(0, 123, 255, 0.3);
}

.secondary-btn:active {
  transform: translateY(0);
  box-shadow: 0 2px 8px rgba(0, 123, 255, 0.2);
}

.btn-icon {
  font-size: 18px;
}

.btn-text {
  font-weight: inherit;
}
</style>