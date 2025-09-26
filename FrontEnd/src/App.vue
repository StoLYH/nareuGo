<script setup>
import { ref, onMounted, onUnmounted } from 'vue'
import SellerPickupModal from './components/SellerPickupModal.vue'
import BuyerPickupModal from './components/BuyerPickupModal.vue'

// 모달 상태 관리
const showSellerPickupModal = ref(false)
const showBuyerPickupModal = ref(false)
const sellerDeliveryData = ref(null)
const buyerDeliveryData = ref(null)

// FCM 이벤트 리스너들
const handleRobotArrivedAtSeller = (event) => {
  console.log('🤖 판매자 픽업 이벤트 수신:', event.detail)
  sellerDeliveryData.value = {
    deliveryId: event.detail.deliveryId,
    productTitle: event.detail.productTitle,
    buyerName: event.detail.buyerName,
    ...event.detail.data
  }
  showSellerPickupModal.value = true
}

const handleRobotArrivedAtBuyer = (event) => {
  console.log('🏠 구매자 픽업 이벤트 수신:', event.detail)
  buyerDeliveryData.value = {
    deliveryId: event.detail.deliveryId,
    productTitle: event.detail.productTitle,
    sellerName: event.detail.sellerName,
    ...event.detail.data
  }
  showBuyerPickupModal.value = true
}

const handleShowPickupModal = (event) => {
  console.log('📱 픽업 모달 표시 이벤트:', event.detail)
  handleRobotArrivedAtSeller(event)
}

const handleShowBuyerPickupModal = (event) => {
  console.log('📱 구매자 픽업 모달 표시 이벤트:', event.detail)
  handleRobotArrivedAtBuyer(event)
}

// 모달 닫기 핸들러들
const closeSellerPickupModal = () => {
  showSellerPickupModal.value = false
  sellerDeliveryData.value = null
}

const closeBuyerPickupModal = () => {
  showBuyerPickupModal.value = false
  buyerDeliveryData.value = null
}

const handleSellerPickupConfirmed = (data) => {
  console.log('✅ 판매자 픽업 완료:', data)
  closeSellerPickupModal()
}

const handleBuyerPickupConfirmed = (data) => {
  console.log('✅ 구매자 수령 완료:', data)
  closeBuyerPickupModal()
}

// 테스트용 함수들 (개발용)
const testSellerPickup = () => {
  const event = {
    detail: {
      deliveryId: '1',
      productTitle: '테스트 상품',
      buyerName: '구매자 테스트',
      data: {}
    }
  }
  handleRobotArrivedAtSeller(event)
}

const testBuyerPickup = () => {
  const event = {
    detail: {
      deliveryId: '1',
      productTitle: '테스트 상품',
      sellerName: '판매자 테스트',
      data: {}
    }
  }
  handleRobotArrivedAtBuyer(event)
}

// 라이프사이클
onMounted(() => {
  console.log('🚀 App.vue 마운트됨 - FCM 이벤트 리스너 등록')

  // FCM 이벤트 리스너 등록
  window.addEventListener('robotArrivedAtSeller', handleRobotArrivedAtSeller)
  window.addEventListener('robotArrivedAtBuyer', handleRobotArrivedAtBuyer)
  window.addEventListener('showPickupModal', handleShowPickupModal)
  window.addEventListener('showBuyerPickupModal', handleShowBuyerPickupModal)

  // 전역에서 접근 가능하도록 설정 (개발용)
  window.testSellerPickup = testSellerPickup
  window.testBuyerPickup = testBuyerPickup
})

onUnmounted(() => {
  console.log('🔚 App.vue 언마운트됨 - FCM 이벤트 리스너 제거')

  // FCM 이벤트 리스너 제거
  window.removeEventListener('robotArrivedAtSeller', handleRobotArrivedAtSeller)
  window.removeEventListener('robotArrivedAtBuyer', handleRobotArrivedAtBuyer)
  window.removeEventListener('showPickupModal', handleShowPickupModal)
  window.removeEventListener('showBuyerPickupModal', handleShowBuyerPickupModal)
})
</script>

<template>
  <div class="layout">
    <div class="content">
      <router-view />
    </div>

    <!-- 판매자 픽업 모달 -->
    <SellerPickupModal
      :is-visible="showSellerPickupModal"
      :delivery-data="sellerDeliveryData"
      @close="closeSellerPickupModal"
      @pickup-confirmed="handleSellerPickupConfirmed"
    />

    <!-- 구매자 픽업 모달 -->
    <BuyerPickupModal
      :is-visible="showBuyerPickupModal"
      :delivery-data="buyerDeliveryData"
      @close="closeBuyerPickupModal"
      @pickup-confirmed="handleBuyerPickupConfirmed"
    />
  </div>
</template>

<style>
/* reset */
html,
body,
div,
span,
applet,
object,
iframe,
h1,
h2,
h3,
h4,
h5,
h6,
p,
blockquote,
pre,
a,
abbr,
acronym,
address,
big,
cite,
code,
del,
dfn,
em,
img,
ins,
kbd,
q,
s,
samp,
small,
strike,
strong,
sub,
sup,
tt,
var,
b,
u,
i,
center,
dl,
dt,
dd,
ol,
ul,
li,
fieldset,
form,
label,
legend,
table,
caption,
tbody,
tfoot,
thead,
tr,
th,
td,
article,
aside,
canvas,
details,
embed,
figure,
figcaption,
footer,
header,
hgroup,
menu,
nav,
output,
ruby,
section,
summary,
time,
mark,
audio,
video {
  margin: 0;
  padding: 0;
  border: 0;
  font-size: 100%;
  font: inherit;
  vertical-align: baseline;
}
body {
  line-height: 1;
  /* 전체 페이지 배경: 헤더/로그인 버튼 그라디언트와 조화되는 라이트 블루 톤 */
  background: linear-gradient(180deg, #EAF3FB 0%, #F5F9FC 100%);
  font-family: "Pretendard-Regular", "Noto Sans KR", -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
}
ol,
ul {
  list-style: none;
}
blockquote,
q {
  quotes: none;
}
blockquote::before,
blockquote::after,
q::before,
q::after {
  content: "";
}
table {
  border-collapse: collapse;
  border-spacing: 0;
}
/* === Global caret and selection rules to avoid stray carets on non-inputs === */
/* 기본적으로 전체 페이지에서 캐럿을 숨기고, 실제 입력 가능한 요소에서만 표시합니다. */
/* 모든 요소에 대해 기본 캐럿 숨김 처리 */
*:not(input):not(textarea):not([contenteditable]):not([contenteditable="true"]) {
  caret-color: transparent;
}
/* 입력 가능한 요소에 대해서만 캐럿 표시 */
input, textarea, [contenteditable]:not([contenteditable="false"]) {
  caret-color: auto; /* 입력창에서는 정상 표시 */
}
/* 비인터랙티브 요소에 포커스가 가는 경우의 외곽선 제거 (접근성 영향 없음: 버튼/링크에는 적용하지 않음) */
:where(p, span, img, svg, strong, em, h1, h2, h3, h4, h5, h6):focus {
  outline: none;
}
/* UI 컨트롤에서 텍스트 드래그 방지 (불필요한 선택 방지) */
button, .nav-item, .slider-nav, .like-button, .chat-button {
  user-select: none;
}
/* 페이지 전반에서 비입력 요소의 텍스트 선택 및 캐럿 표시를 억제 */
html, body, .layout, .content {
  -webkit-user-select: none;
  -moz-user-select: none;
  -ms-user-select: none;
  user-select: none;
}
/* 입력/에디터 영역에서는 정상적으로 텍스트 선택 및 캐럿 표시 */
input, textarea, [contenteditable="true"], [contenteditable]:not([contenteditable="false"]) {
  -webkit-user-select: text;
  -moz-user-select: text;
  -ms-user-select: text;
  user-select: text;
}
/* 폰트 & CSS 변수 */
@import url('https://fonts.googleapis.com/css2?family=Noto+Sans+KR:wght@400;500;600;700&display=swap');

@font-face {
  font-family: "Pretendard-Regular";
  src: url("https://cdn.jsdelivr.net/gh/Project-Noonnu/noonfonts_2107@1.1/Pretendard-Regular.woff")
    format("woff");
  font-weight: 400;
  font-style: normal;
}
:root {
  --main: #4682b4;
  --disabled: #dcb6b6;
  --gray: #dbdbdb;
  --warning: #fc6d6d;
  --deepgray: #767676;
}
* {
  box-sizing: border-box;
  font-family: "Pretendard-Regular", "Noto Sans KR", -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
}
a {
  text-decoration: none;
  color: inherit;
}
button {
  background: none;
  border: none;
  padding: 0;
  margin: 0;
  cursor: pointer;
}
.a11y-hidden {
  clip: rect(1px, 1px, 1px, 1px);
  clip-path: inset(50%);
  width: 1px;
  height: 1px;
  margin: -1px;
  overflow: hidden;
  padding: 0;
  position: absolute;
}

.layout {
  width: 100%;
  max-width: 390px;
  margin: 0 auto;
  min-height: 100vh;
  display: flex;
  justify-content: center;
  align-items: flex-start;
  background-color: #f8f9fa;
  box-shadow: rgba(100, 100, 111, 0.2) 0px 7px 29px 0px;
}

.content {
  width: 100%;
  /* height: 100vh;  */
}
</style>