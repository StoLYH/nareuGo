<template>
  <div class="pending-container">
    <header class="pending-header">
      <button @click="goBack" class="back-button" aria-label="뒤로가기">
        <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
          <polyline points="15 18 9 12 15 6"></polyline>
        </svg>
      </button>
      <h1 class="header-title">결제 대기</h1>
      <div class="header-spacer"></div>
    </header>

    <main class="pending-content">
      <section class="card">
        <div class="card-header">
          <h3>판매자 승인 대기 중</h3>
        </div>
        <div class="card-body">
          <p class="desc">판매자가 결제 요청을 확인하고 승인하면 결제를 진행할 수 있어요.</p>
          <div class="status-row">
            <span class="label">주문 번호</span>
            <span class="value">{{ orderId }}</span>
          </div>
          <div class="status-row">
            <span class="label">상태</span>
            <span class="value" :class="{ approved: isApproved }">
              {{ isApproved ? '승인 완료' : '승인 대기' }}
            </span>
          </div>
        </div>
      </section>

      <button class="primary-button" :disabled="!isApproved" @click="goToDetail">
        {{ isApproved ? '결제 상세로 이동' : '승인 대기 중...' }}
      </button>
    </main>

    <!-- 승인 모달 -->
    <transition name="fade-zoom">
      <div v-if="showApprovalModal" class="modal-overlay" @click.self="closeModal">
        <div class="modal-card">
          <div class="modal-icon" aria-hidden="true">
            <svg width="28" height="28" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M22 11.08V12a10 10 0 1 1-5.93-9.14" stroke="#22C55E" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
              <polyline points="22 4 12 14.01 9 11.01" stroke="#22C55E" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
            </svg>
          </div>
          <h3>판매자가 결제를 승인했어요</h3>
          <p>이제 결제를 진행할 수 있습니다.</p>
        </div>
      </div>
    </transition>
  </div>
</template>

<script setup>
import { ref, onMounted } from 'vue'
import { useRoute, useRouter } from 'vue-router'

const route = useRoute()
const router = useRouter()

const orderId = ref(route.params.orderId)
const isApproved = ref(false)
const showApprovalModal = ref(false)

onMounted(() => {
  // 임시 시뮬레이션: 3초 후 승인 처리 + 잠깐 모달 표시
  setTimeout(() => {
    isApproved.value = true
    showApprovalModal.value = true
    setTimeout(() => {
      showApprovalModal.value = false
    }, 2000)
  }, 5000)
})

const closeModal = () => {
  showApprovalModal.value = false
}

const goToDetail = () => {
  if (!isApproved.value) return
  router.push({ name: 'PaymentDetail', params: { orderId: orderId.value } })
}

const goBack = () => {
  router.go(-1)
}
</script>

<style scoped>
.pending-container {
  width: 100%;
  max-width: 390px;
  margin: 0 auto;
  min-height: 100vh;
  background-color: #f8f9fa;
  display: flex;
  flex-direction: column;
  padding-top: 64px; /* fixed header height */
}

.pending-header {
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
  justify-content: space-between;
  box-shadow: 0 4px 12px rgba(0,0,0,0.12);
}

.header-title {
  font-size: 18px;
  font-weight: 600;
  color: #fff;
}

.header-spacer {
  width: 40px;
}

.back-button {
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

.back-button:hover {
  background-color: rgba(255,255,255,0.28);
}

.pending-content {
  flex: 1;
  padding: 20px;
  display: flex;
  flex-direction: column;
  gap: 16px;
}

.card {
  background: #fff;
  border-radius: 12px;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.06);
  overflow: hidden;
}

.card-header {
  padding: 20px 20px 12px;
  border-bottom: 1px solid #f0f2f5;
}

.card-header h3 {
  margin: 0;
  font-size: 16px;
  font-weight: 600;
  color: #333;
}

.card-body {
  padding: 20px;
}

.desc {
  font-size: 14px;
  color: #555;
  margin-bottom: 12px;
}

.status-row {
  display: flex;
  justify-content: space-between;
  padding: 10px 0;
  border-bottom: 1px solid #f8f9fa;
}

.status-row:last-child {
  border-bottom: none;
}

.label {
  color: #666;
  font-size: 14px;
}

.value {
  color: #333;
  font-size: 14px;
  font-weight: 600;
}

.value.approved { color: #22C55E; }

.primary-button {
  width: 100%;
  background-color: #4682b4;
  color: white;
  border: none;
  border-radius: 10px;
  padding: 14px;
  font-size: 15px;
  font-weight: 600;
  cursor: pointer;
  transition: background-color 0.2s;
}

.primary-button:disabled {
  background-color: #A8C1D8;
  cursor: not-allowed;
}

.primary-button:not(:disabled):hover {
  background-color: #3a6b9a;
}

/* Modal */
.modal-overlay {
  position: fixed;
  inset: 0;
  background: rgba(0, 0, 0, 0.35);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 100;
}

.modal-card {
  width: calc(100% - 40px);
  max-width: 360px;
  background: #ffffff;
  border-radius: 14px;
  box-shadow: 0 12px 30px rgba(0, 0, 0, 0.2);
  padding: 20px;
  text-align: center;
}

.modal-icon {
  display: flex;
  justify-content: center;
  margin-bottom: 10px;
}

.fade-zoom-enter-active, .fade-zoom-leave-active {
  transition: opacity 0.2s ease, transform 0.2s ease;
}
.fade-zoom-enter-from, .fade-zoom-leave-to {
  opacity: 0;
  transform: scale(0.98);
}
</style>
