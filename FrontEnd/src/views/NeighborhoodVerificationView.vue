<template>
  <div class="verification-container">
    <div class="verification-view">
      <!-- 상단 헤더 -->
      <header class="verification-header">
        <button @click="goBack" class="back-button">
          <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <polyline points="15 18 9 12 15 6"></polyline>
          </svg>
        </button>
        <h1 class="header-title">동네 인증</h1>
        <div class="header-spacer"></div>
      </header>

      <!-- 메인 콘텐츠 -->
      <main class="verification-content">
        <!-- 진행 상태 표시 -->
        <section class="progress-section">
          <div class="progress-bar">
            <div class="progress-step" :class="{ active: currentStep >= 1, completed: gpsVerified }">
              <div class="step-circle">
                <svg v-if="gpsVerified" width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <polyline points="20 6 9 17 4 12" stroke="white" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                </svg>
                <span v-else>1</span>
              </div>
              <span class="step-label">현위치</span>
            </div>
            <div class="progress-line" :class="{ active: currentStep >= 2 }"></div>
            <div class="progress-step" :class="{ active: currentStep >= 2, completed: ocrVerified }">
              <div class="step-circle">
                <svg v-if="ocrVerified" width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <polyline points="20 6 9 17 4 12" stroke="white" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                </svg>
                <span v-else>2</span>
              </div>
              <span class="step-label">주소</span>
            </div>
            <div class="progress-line" :class="{ active: currentStep >= 3 }"></div>
            <div class="progress-step" :class="{ active: currentStep >= 3, completed: apartmentVerified }">
              <div class="step-circle">
                <svg v-if="apartmentVerified" width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <polyline points="20 6 9 17 4 12" stroke="white" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                </svg>
                <span v-else>3</span>
              </div>
              <span class="step-label">아파트</span>
            </div>
          </div>
        </section>

        <!-- GPS 위치 인증 섹션 -->
        <section class="verification-card" v-if="currentStep === 1">
          <div class="card-header">
            <div class="card-icon gps-icon">
              <svg width="32" height="32" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M12 2C8.13 2 5 5.13 5 9C5 14.25 12 22 12 22S19 14.25 19 9C19 5.13 15.87 2 12 2ZM12 11.5C10.62 11.5 9.5 10.38 9.5 9C9.5 7.62 10.62 6.5 12 6.5C13.38 6.5 14.5 7.62 14.5 9C14.5 10.38 13.38 11.5 12 11.5Z" fill="currentColor"/>
              </svg>
            </div>
            <h2>GPS 위치 인증</h2>
            <p>현재 위치를 확인하여 동네를 인증합니다</p>
          </div>
          
          <div class="verification-content-area">
            <div v-if="!gpsLoading && !gpsVerified" class="verification-info">
              <div class="info-item">
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <circle cx="12" cy="12" r="10" stroke="currentColor" stroke-width="2" fill="none"/>
                  <line x1="12" y1="8" x2="12" y2="12" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
                  <line x1="12" y1="16" x2="12.01" y2="16" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
                </svg>
                <span>위치 권한을 허용해주세요</span>
              </div>
              <div class="info-item">
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <circle cx="12" cy="12" r="10" stroke="currentColor" stroke-width="2" fill="none"/>
                  <line x1="12" y1="8" x2="12" y2="12" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
                  <line x1="12" y1="16" x2="12.01" y2="16" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
                </svg>
                <span>정확한 위치 확인을 위해 실외에서 진행해주세요</span>
              </div>
            </div>

            <div v-if="gpsLoading" class="loading-state">
              <div class="loading-spinner"></div>
              <p>위치를 확인하고 있습니다...</p>
            </div>

            <div v-if="gpsVerified" class="success-state">
              <!-- 지도 컴포넌트 -->
              <KakaoMapComponent 
                :latitude="currentLatitude"
                :longitude="currentLongitude"
                :address="verifiedLocation"
                :accuracy="locationAccuracy"
                height="250px"
              />
              
              <div class="success-content">
                <div class="success-icon">
                  <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                    <path d="M22 11.08V12a10 10 0 1 1-5.93-9.14" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                    <polyline points="22 4 12 14.01 9 11.01" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                  </svg>
                </div>
                <h3>위치 인증 완료!</h3>
                <p class="location-info">{{ verifiedLocation }}</p>
              </div>
            </div>

            <div v-if="gpsError" class="error-state">
              <div class="error-icon">
                <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <circle cx="12" cy="12" r="10" stroke="currentColor" stroke-width="2" fill="none"/>
                  <line x1="15" y1="9" x2="9" y2="15" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
                  <line x1="9" y1="9" x2="15" y2="15" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
                </svg>
              </div>
              <h3>위치 인증 실패</h3>
              <p class="error-message">{{ gpsError }}</p>
            </div>
          </div>

          <div class="verification-actions">
            <!-- 오류 시: '다시 시도'만 메인 위치에 표시 -->
            <button v-if="gpsError" @click="retryGpsVerification" class="primary-button">다시 시도</button>
            <!-- 정상 흐름: 시작/다음 -->
            <button v-else-if="!gpsVerified" @click="startGpsVerification" :disabled="gpsLoading" class="primary-button">
              {{ gpsLoading ? '위치 확인 중...' : '위치 인증 시작' }}
            </button>
            <button v-else @click="nextStep" class="primary-button">다음 단계</button>
          </div>
        </section>

        <!-- OCR 주소 인증 섹션 -->
        <section class="verification-card" v-if="currentStep === 2">
          <div class="card-header">
            <div class="card-icon ocr-icon">
              <svg width="32" height="32" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M14 2H6C4.9 2 4 2.9 4 4V20C4 21.1 4.89 22 5.99 22H18C19.1 22 20 21.1 20 20V8L14 2ZM18 20H6V4H13V9H18V20Z" fill="currentColor"/>
                <path d="M8 12H16M8 16H13" stroke="white" stroke-width="1.5" stroke-linecap="round"/>
              </svg>
            </div>
            <h2>신분증 주소 인증</h2>
            <p>신분증을 촬영하여 주소를 확인합니다</p>
          </div>

          <div class="verification-content-area">
            <div v-if="!ocrLoading && !ocrVerified && !capturedImage" class="upload-area">
              <div class="upload-placeholder">
                <svg width="64" height="64" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M14.5 4H20C21.1 4 22 4.9 22 6V18C22 19.1 21.1 20 20 20H4C2.9 20 2 19.1 2 18V6C2 4.9 2.9 4 4 4H9.5" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                  <path d="M12 15L9 12L12 9L15 12L12 15Z" fill="currentColor"/>
                </svg>
                <h3>신분증을 촬영해주세요</h3>
                <p>주민등록증 또는 운전면허증의<br>주소 부분이 선명하게 보이도록 촬영해주세요</p>
              </div>
              
              <div class="upload-tips">
                <div class="tip-item">
                  <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                    <circle cx="12" cy="12" r="10" stroke="currentColor" stroke-width="2" fill="none"/>
                    <line x1="12" y1="8" x2="12" y2="12" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
                    <line x1="12" y1="16" x2="12.01" y2="16" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
                  </svg>
                  <span>개인정보는 주소 확인 후 즉시 삭제됩니다</span>
                </div>
                <div class="tip-item">
                  <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                    <circle cx="12" cy="12" r="10" stroke="currentColor" stroke-width="2" fill="none"/>
                    <line x1="12" y1="8" x2="12" y2="12" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
                    <line x1="12" y1="16" x2="12.01" y2="16" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
                  </svg>
                  <span>충분한 조명 아래에서 촬영해주세요</span>
                </div>
              </div>
            </div>

            <div v-if="capturedImage" class="image-preview">
              <div class="image-container">
                <img :src="capturedImage" alt="촬영된 신분증" />
                
                <!-- 로딩 오버레이 -->
                <div v-if="ocrLoading" class="image-overlay loading-overlay">
                  <div class="loading-spinner">
                    <div class="spinner-circle"></div>
                  </div>
                  <p>신분증을 인식하고 있습니다...</p>
                </div>
                
                <!-- 성공 오버레이 -->
                <div v-if="ocrVerified && ocrResult?.addressMatched" class="image-overlay success-overlay">
                  <div class="success-icon">
                    <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                      <circle cx="12" cy="12" r="10" fill="#22c55e" stroke="#22c55e" stroke-width="2"/>
                      <polyline points="9 12 12 15 16 10" stroke="white" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                    </svg>
                  </div>
                  <h3>주소 인증 완료!</h3>
                  <div class="parsed-address-info">
                    <div class="full-address">
                      {{ getFormattedAddress() }}
                    </div>
                  </div>
                </div>
                
                <!-- 실패 오버레이 -->
                <div v-if="ocrVerified && !ocrResult?.addressMatched" class="image-overlay failure-overlay">
                  <div class="failure-icon">
                    <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                      <circle cx="12" cy="12" r="10" fill="#ef4444" stroke="#ef4444" stroke-width="2"/>
                      <line x1="15" y1="9" x2="9" y2="15" stroke="white" stroke-width="2" stroke-linecap="round"/>
                      <line x1="9" y1="9" x2="15" y2="15" stroke="white" stroke-width="2" stroke-linecap="round"/>
                    </svg>
                  </div>
                  <h3>주소 인증 실패</h3>
                  <p>민증 주소와 GPS 위치가 일치하지 않습니다</p>
                  <div class="mismatch-info">
                    <div class="mismatch-item">
                      <span class="label">민증 주소:</span>
                      <span class="value">{{ getFormattedAddress() }}</span>
                    </div>
                    <div class="mismatch-item">
                      <!-- <span class="label">GPS 위치:</span> -->
                      <!-- <span class="value">{{ verifiedAddress || '위치 정보 없음' }}</span> -->
                    </div>
                  </div>
                </div>
              </div>
              
              <div v-if="!ocrLoading && !ocrVerified" class="image-actions">
                <button @click="retakePhoto" class="secondary-button">다시 촬영</button>
                <button @click="processOCR" class="primary-button">주소 확인</button>
              </div>
              
              <div v-if="ocrVerified" class="address-match-info">
                <p class="match-status" :class="{ success: ocrResult.addressMatched, warning: !ocrResult.addressMatched }">
                  {{ ocrResult.addressMatched ? '✅ 주소가 일치합니다' : '⚠️ 주소 확인이 필요합니다' }}
                </p>
              </div>
            </div>

            <!-- <div v-if="ocrError" class="error-state">
              <div class="error-icon">
                <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <circle cx="12" cy="12" r="10" stroke="currentColor" stroke-width="2" fill="none"/>
                  <line x1="15" y1="9" x2="9" y2="15" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
                  <line x1="9" y1="9" x2="15" y2="15" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
                </svg>
              </div>
              <h3>주소 인증 실패</h3>
              <p class="error-message">{{ ocrError }}</p>
            </div> -->
          </div>

          <div class="verification-actions">
            <!-- 오류 시: '다시 시도'를 메인 위치에 표시 -->
            <button v-if="ocrError" @click="retryOCR" class="primary-button">다시 시도</button>
            <!-- 정상 흐름 버튼들 -->
            <button v-else-if="!capturedImage && !ocrVerified" @click="capturePhoto" class="primary-button">
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M23 19C23 20.1 22.1 21 21 21H3C1.9 21 1 20.1 1 19V8C1 6.9 1.9 6 3 6H7L9 4H15L17 6H21C22.1 6 23 6.9 23 8V19Z" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                <circle cx="12" cy="13" r="4" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
              </svg>
              신분증 촬영하기
            </button>
            <button v-else-if="ocrVerified" @click="goToApartmentInfo" class="primary-button">
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <polyline points="9 18 15 12 9 6" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
              </svg>
              다음
            </button>
          </div>
        </section>

        <!-- 아파트 정보 입력 섹션 -->
        <section class="verification-card" v-if="currentStep === 3">
          <div class="card-header">
            <div class="card-icon">
              <svg width="32" height="32" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M3 21H21V9L12 2L3 9V21Z" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                <polyline points="9 21 9 12 15 12 15 21" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
              </svg>
            </div>
            <h2>아파트 정보 입력</h2>
            <p>거주하시는 아파트의 상세 정보를 입력해주세요.</p>
          </div>

          <div class="apartment-form">
            <div class="form-section">
              <!-- <h3 class="section-title">
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M3 21H21V9L12 2L3 9V21Z" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                </svg>
                거주지 정보
              </h3>
               -->
              <div class="input-group">
                <label for="apartmentName" class="input-label">
                  <span class="label-text">아파트 이름</span>
                  <span class="label-required">*</span>
                </label>
                <div class="input-wrapper">
                  <input 
                    type="text" 
                    id="apartmentName" 
                    v-model="apartmentName" 
                    placeholder="예: 청솔마을아파트"
                    :disabled="isApartmentLoading"
                    class="form-input"
                  />
                  <div class="input-icon">
                    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                      <path d="M3 21H21V9L12 2L3 9V21Z" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                    </svg>
                  </div>
                </div>
              </div>
              
              <div class="form-row">
                <div class="input-group">
                  <label for="buildingDong" class="input-label">
                    <span class="label-text">동</span>
                  </label>
                  <div class="input-wrapper">
                    <input 
                      type="number" 
                      id="buildingDong" 
                      v-model="buildingDong" 
                      placeholder="111"
                      :disabled="true"
                      class="form-input"
                    />
                    <div class="input-icon">
                      <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                        <rect x="3" y="3" width="18" height="18" rx="2" ry="2" stroke="currentColor" stroke-width="2" fill="none"/>
                        <line x1="9" y1="9" x2="9" y2="15" stroke="currentColor" stroke-width="2"/>
                        <line x1="15" y1="9" x2="15" y2="15" stroke="currentColor" stroke-width="2"/>
                      </svg>
                    </div>
                  </div>
                </div>
                
                <div class="input-group">
                  <label for="buildingHo" class="input-label">
                    <span class="label-text">호</span>
                  </label>
                  <div class="input-wrapper">
                    <input 
                      type="number" 
                      id="buildingHo" 
                      v-model="buildingHo" 
                      placeholder="901"
                      :disabled="true"
                      class="form-input"
                    />
                    <div class="input-icon">
                      <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                        <path d="M7 21H17C18.1 21 19 20.1 19 19V5C19 3.9 18.1 3 17 3H7C5.9 3 5 3.9 5 5V19C5 20.1 5.9 21 7 21Z" stroke="currentColor" stroke-width="2" fill="none"/>
                        <line x1="9" y1="7" x2="15" y2="7" stroke="currentColor" stroke-width="2"/>
                        <line x1="9" y1="11" x2="15" y2="11" stroke="currentColor" stroke-width="2"/>
                      </svg>
                    </div>
                  </div>
                </div>
              </div>
              
            </div>

            <!-- 로딩 상태 -->
            <div v-if="isApartmentLoading" class="loading-state">
              <div class="loading-spinner"></div>
              <p>인증 정보를 저장하고 있습니다...</p>
            </div>

            <!-- 완료 상태 -->
            <div v-if="apartmentVerified" class="success-state">
              <div class="success-icon">
                <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M22 11.08V12a10 10 0 1 1-5.93-9.14" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                  <polyline points="22 4 12 14.01 9 11.01" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                </svg>
              </div>
              <h3>인증이 완료되었습니다!</h3>
              <p>이제 우리 동네에서 안전하게 거래할 수 있습니다.</p>
            </div>
          </div>

          <div class="verification-actions">
            <button 
              v-if="!apartmentVerified && !isApartmentLoading" 
              @click="submitApartmentInfo" 
              class="primary-button"
              :disabled="!apartmentName"
            >
              인증 완료
            </button>
            <button 
              v-if="apartmentVerified" 
              @click="goToItemList" 
              class="primary-button"
            >
              확인
            </button>
          </div>
        </section>
      </main>

      <!-- 카메라 모달 -->
      <div v-if="showCamera" class="camera-modal">
        <div class="camera-container">
          <div class="camera-header">
            <h3>신분증 촬영</h3>
            <button @click="closeCamera" class="close-button">
              <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <line x1="18" y1="6" x2="6" y2="18" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
                <line x1="6" y1="6" x2="18" y2="18" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
              </svg>
            </button>
          </div>
          <!-- 카메라 영역 -->
          <div class="camera-content">
            <video ref="video" autoplay playsinline></video>
            <canvas ref="canvas" style="display: none;"></canvas>
            <div class="camera-overlay">
              <div class="id-card-frame">
                <div class="frame-corner top-left"></div>
                <div class="frame-corner top-right"></div>
                <div class="frame-corner bottom-left"></div>
                <div class="frame-corner bottom-right"></div>
              </div>
              <p class="camera-guide">신분증을 프레임 안에 맞춰주세요</p>
            </div>
          </div>


          <div class="camera-actions">
            <button @click="takePhoto" class="capture-button">
              <svg width="32" height="32" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <circle cx="12" cy="12" r="10" stroke="currentColor" stroke-width="2" fill="white"/>
              </svg>
            </button>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import KakaoMapComponent from '../components/KakaoMapComponent.vue';

export default {
  name: "NeighborhoodVerificationView",
  components: {
    KakaoMapComponent
  },
  data() {
    return {
      currentStep: 1,
      testMode: false, // 실제 모드로 전환
      currentLatitude: null,
      currentLongitude: null,
      verifiedLocation: null,
      verifiedAddress: null,
      locationAccuracy: null,
      isLocationVerified: false,
      isPhotoTaken: false,
      capturedImage: null,
      showCamera: false,
      stream: null,
      isProcessing: false,
      verificationResult: null,
      gpsLoading: false,
      gpsError: null,
      gpsVerified: false,
      ocrVerified: false,
      apartmentVerified: false,
      apartmentName: '',
      buildingDong: '',
      buildingHo: '',
      isApartmentLoading: false
    };
  },
  methods: {
    // 로컬스토리지에서 사용자 정보 가져오기
    getUserInfo() {
      try {
        const userInfo = localStorage.getItem('user');
        return userInfo ? JSON.parse(userInfo) : null;
      } catch (error) {
        console.error('사용자 정보 파싱 오류:', error);
        return null;
      }
    },
    
    // 파싱된 주소 정보를 한 줄로 포맷팅
    getFormattedAddress() {
      console.log('getFormattedAddress 호출됨');
      console.log('ocrResult:', this.ocrResult);
      
      if (!this.ocrResult?.addressComponents) {
        console.log('addressComponents가 없음');
        return '주소 정보를 확인할 수 없습니다';
      }
      
      const components = this.ocrResult.addressComponents;
      console.log('addressComponents:', components);
      console.log('sido:', components.sido);
      console.log('sigungu:', components.sigungu);
      console.log('dong:', components.dong);
      console.log('buildingDong:', components.buildingDong);
      console.log('buildingHo:', components.buildingHo);
      
      let address = '';
      
      // 시/도 추가 (백엔드 필드명: sido)
      if (components.sido) {
        address += components.sido;
        console.log('시도 추가:', components.sido);
      }
      
      // 시/군/구 추가 (백엔드 필드명: sigungu)
      if (components.sigungu) {
        address += (address ? ' ' : '') + components.sigungu;
        console.log('시군구 추가:', components.sigungu);
      }
      
      // 읍/면/동 추가 (백엔드 필드명: dong)
      if (components.dong) {
        address += (address ? ' ' : '') + components.dong;
        console.log('읍면동 추가:', components.dong);
      }
      
      // 동 번호 추가
      if (components.buildingDong) {
        address += (address ? ' ' : '') + components.buildingDong + '동';
        console.log('동 추가:', components.buildingDong);
      }
      
      // 호 번호 추가
      if (components.buildingHo) {
        address += (address ? ' ' : '') + components.buildingHo + '호';
        console.log('호 추가:', components.buildingHo);
      }
      
      console.log('최종 주소:', address);
      return address || '주소 정보를 확인할 수 없습니다';
    },
    goBack() {
      this.$router.go(-1);
    },
    
    nextStep() {
      this.currentStep = 2;
    },
    
    // 아파트 정보 제출
    async submitApartmentInfo() {
      if (!this.apartmentName) {
        alert('아파트 이름을 입력해주세요.');
        return;
      }
      
      // OCR에서 추출한 동/호수 정보가 없으면 기본값 설정
      if (!this.buildingDong) {
        this.buildingDong = '0';
      }
      if (!this.buildingHo) {
        this.buildingHo = '0';
      }
      
      this.isApartmentLoading = true;
      
      try {
        // 몇 초간 로딩 시뮬레이션
        await new Promise(resolve => setTimeout(resolve, 2000));
        
        // 아파트 정보를 포함한 최종 인증 완료 처리
        const userInfo = this.getUserInfo();
        if (!userInfo) {
          throw new Error('사용자 정보를 찾을 수 없습니다.');
        }
        
        // 백엔드에 최종 아파트 정보 저장
        const response = await fetch(`${import.meta.env.VITE_BASE_URL}/neighborhood/verify`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            userEmail: userInfo.email,
            ocrAddress: this.verifiedAddress,
            gpsAddress: this.verifiedLocation,
            apartmentName: this.apartmentName,
            buildingDong: parseInt(this.buildingDong),
            buildingHo: parseInt(this.buildingHo)
          })
        });
        
        const result = await response.json();
        
        if (result.success) {
          this.apartmentVerified = true;
          console.log('아파트 정보 저장 완료:', result);
        } else {
          throw new Error(result.message || '아파트 정보 저장에 실패했습니다.');
        }
        
      } catch (error) {
        console.error('아파트 정보 저장 오류:', error);
        alert('아파트 정보 저장 중 오류가 발생했습니다. 다시 시도해주세요.');
      } finally {
        this.isApartmentLoading = false;
      }
    },
    
    // ItemListView로 이동
    goToItemList() {
      this.$router.push('/items');
    },
    
    async startGpsVerification() {
      this.gpsLoading = true;
      this.gpsError = null;
      try {
        // 카카오 지도 헬퍼 사용: 현재 좌표와 주소 동시 획득
        const { latitude, longitude, fullAddress, accuracy } = await this.getCurrentLocationWithKakao();
        this.currentLatitude = latitude;
        this.currentLongitude = longitude;
        this.verifiedLocation = fullAddress;
        this.locationAccuracy = accuracy;

        this.isLocationVerified = true;
        this.gpsVerified = true;
      } catch (error) {
        console.error('위치 인증 오류:', error);
        this.gpsError = this.getGpsErrorMessage(error);
      } finally {
        this.gpsLoading = false;
      }
    },
    
    async getCurrentLocationWithKakao() {
      // 카카오맵 API를 사용하여 현재 위치와 주소 가져오기
      const { getCurrentLocationWithAddress } = await import('../config/kakaoMap.js');
      return await getCurrentLocationWithAddress();
    },
    
    getGpsErrorMessage(error) {
      // 카카오맵 API 오류 처리
      if (error.message) {
        if (error.message.includes('카카오맵 API 오류')) {
          return '카카오맵 서비스에 일시적인 문제가 발생했습니다. 잠시 후 다시 시도해주세요.';
        }
        if (error.message.includes('주소 정보를 찾을 수 없습니다')) {
          return '현재 위치의 주소 정보를 찾을 수 없습니다. 다른 장소에서 시도해주세요.';
        }
        if (error.message.includes('위치 서비스를 지원하지 않습니다')) {
          return '이 브라우저는 위치 서비스를 지원하지 않습니다.';
        }
      }
      
      // 브라우저 Geolocation API 오류 처리
      if (error.code) {
        switch (error.code) {
          case error.PERMISSION_DENIED:
            return '위치 권한이 거부되었습니다. 브라우저 설정에서 위치 권한을 허용해주세요.';
          case error.POSITION_UNAVAILABLE:
            return '위치 정보를 사용할 수 없습니다.';
          case error.TIMEOUT:
            return '위치 요청 시간이 초과되었습니다.';
          default:
            return '위치 인증 중 오류가 발생했습니다.';
        }
      }
      
      return error.message || '위치 인증 중 오류가 발생했습니다.';
    },
    
    retryGpsVerification() {
      this.gpsError = null;
      this.startGpsVerification();
    },
    
    async capturePhoto() {
      this.showCamera = true;
      try {
        this.stream = await navigator.mediaDevices.getUserMedia({
          video: { 
            facingMode: 'environment',
            width: { ideal: 1280 },
            height: { ideal: 720 }
          }
        });
        this.$refs.video.srcObject = this.stream;
      } catch (error) {
        console.error('카메라 접근 오류:', error);
        alert('카메라에 접근할 수 없습니다. 권한을 확인해주세요.');
        this.closeCamera();
      }
    },
    
    takePhoto() {
      const video = this.$refs.video;
      const canvas = this.$refs.canvas;
      const context = canvas.getContext('2d');
      
      canvas.width = video.videoWidth;
      canvas.height = video.videoHeight;
      context.drawImage(video, 0, 0);
      
      this.capturedImage = canvas.toDataURL('image/jpeg', 0.8);
      this.closeCamera();
    },
    
    closeCamera() {
      if (this.stream) {
        this.stream.getTracks().forEach(track => track.stop());
        this.stream = null;
      }
      this.showCamera = false;
    },
    
    retakePhoto() {
      this.capturedImage = null;
      this.ocrError = null;
      this.ocrVerified = false;
      this.ocrResult = null;
      this.capturePhoto();
    },
    
    async processOCR() {
      this.ocrLoading = true;
      this.ocrError = null;
      
      try {
        // 이미지를 Base64로 변환 (이미 Base64 형태)
        const imageData = this.capturedImage;
        const imageFormat = 'jpg'; // 카메라에서 촬영한 이미지는 보통 jpg
        
        // 백엔드 OCR API 호출
        const response = await fetch(`${import.meta.env.VITE_BASE_URL}/ocr/verify-address`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            userId: this.getUserInfo()?.userId || null,
            imageData: imageData,
            imageFormat: imageFormat,
            latitude: this.currentLatitude,
            longitude: this.currentLongitude,
            gpsAddress: this.verifiedLocation
          })
        });
        
        const result = await response.json();
        
        if (result.success) {
          this.ocrResult = result;
          this.ocrVerified = true; // OCR 처리가 성공하면 항상 true로 설정
          this.verifiedAddress = result.extractedAddress;
          
          // console.log('OCR 처리 성공:', {
          //   extractedAddress: result.extractedAddress,
          //   matchScore: result.matchScore,
          //   addressMatched: result.addressMatched
          // });
          
          // OCR 처리 완료 후 상태만 업데이트 (자동 완료 처리 제거)
          if (result.addressMatched) {
            // 주소 인증 성공 시 OCR 결과 저장
            // OCR에서 추출한 동, 호수 정보를 자동으로 입력 (아파트명은 제외)
            if (result.addressComponents) {
              // this.apartmentName = ''; // 아파트명은 빈칸으로 유지
              this.buildingDong = result.addressComponents.buildingDong || '';
              this.buildingHo = result.addressComponents.buildingHo || '';
            }
            // 인증 완료 상태로 설정하지만 3단계로 이동하지 않음
          } else {
            // 주소 인증 실패 시 오류 메시지 표시
            this.ocrError = '주소 인증에 실패했습니다. 위치와 신분증 주소가 일치하지 않습니다.';
          }
          
        } else {
          throw new Error(result.errorMessage || 'OCR 처리에 실패했습니다.');
        }
        
      } catch (error) {
        console.error('OCR 처리 오류:', error);
        this.ocrError = error.message || '신분증 인식에 실패했습니다. 다시 시도해주세요.';
      } finally {
        this.ocrLoading = false;
      }
    },
    
    simulateDelay(ms) {
      return new Promise(resolve => setTimeout(resolve, ms));
    },
    
    async completeVerification() {
      try {
        // 동네 인증 정보를 백엔드에 저장
        const response = await fetch(`${import.meta.env.VITE_BASE_URL}/neighborhood/verify`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            'Authorization': `Bearer ${this.$store.state.auth.token || localStorage.getItem('token')}`
          },
          body: JSON.stringify({
            latitude: this.currentLatitude,
            longitude: this.currentLongitude,
            gpsAddress: this.verifiedLocation,
            ocrAddress: this.verifiedAddress,
            matchScore: this.ocrResult.matchScore,
            addressMatched: this.ocrResult.addressMatched
          })
        });

        const result = await response.json();
        
        if (result.success || response.ok) {
          // 인증 상태를 전역 상태에 저장
          const authStore = this.$store || (await import('@/stores/auth')).useAuthStore();
          if (authStore && authStore.updateNeighborhoodVerification) {
            await authStore.updateNeighborhoodVerification({
              verified: true,
              location: this.verifiedLocation,
              address: this.verifiedAddress
            });
          }
          
          // 로컬 스토리지에도 저장
          const userInfo = JSON.parse(localStorage.getItem('userInfo') || '{}');
          userInfo.neighborhoodVerified = true;
          userInfo.verifiedLocation = this.verifiedLocation;
          userInfo.verifiedAddress = this.verifiedAddress;
          localStorage.setItem('userInfo', JSON.stringify(userInfo));
          
          // 3단계(아파트 정보 입력)로 이동
          this.ocrVerified = true;
          this.currentStep = 3;
          
          console.log('동네 인증 완료 및 DB 저장 성공');
        } else {
          throw new Error(result.message || '인증 정보 저장에 실패했습니다.');
        }
      } catch (error) {
        console.error('동네 인증 완료 처리 오류:', error);
        alert('인증 정보 저장 중 오류가 발생했습니다. 다시 시도해주세요.');
      }
    },
    
    goToHome() {
      this.$router.push('/items');
    },
    
    async handleVerificationComplete() {
      try {
        // 위치 인증과 주소 인증 모두 완료되었는지 확인
        const locationVerified = this.locationVerified;
        let addressVerified = this.ocrVerified;
        
        // 테스트 모드에서는 강제로 인증 성공 처리
        if (this.testMode) {
          addressVerified = true;
          console.log('🧪 테스트 모드: 주소 인증 강제 성공');
        }
        
        // console.log('인증 상태 확인:', {
        //   locationVerified,
        //   addressVerified,
        //   matchScore: this.ocrResult?.matchScore,
        //   testMode: this.testMode
        // });
        
        if (locationVerified && addressVerified) {
          // 인증 성공
          this.showSuccessMessage();
          
          // 3초 후 아이템 리스트로 이동
          setTimeout(() => {
            this.$router.push('/items');
          }, 3000);
          
        } else {
          // 인증 실패
          this.showFailureMessage();
        }
        
      } catch (error) {
        console.error('인증 완료 처리 오류:', error);
        this.showFailureMessage();
      }
    },
    
    showSuccessMessage() {
      // 성공 메시지 표시 (기존 ocrVerified 상태 활용)
      this.currentStep = 3; // 완료 단계로 이동
      console.log('✅ 인증 완료! 아이템 리스트로 이동합니다...');
    },
    
    showFailureMessage() {
      // 실패 메시지 표시
      this.ocrError = '주소 인증에 실패했습니다. 위치와 신분증 주소가 일치하지 않습니다.';
      console.log('❌ 인증 실패: 주소가 일치하지 않습니다.');
    },
    
    retryOCR() {
      this.ocrError = null;
      this.ocrVerified = false;
      this.ocrResult = null;
      this.capturedImage = null;
    },
    
    goToApartmentInfo() {
      // OCR에서 추출한 동, 호수 정보를 자동으로 입력 (아파트명은 제외)
      if (this.ocrResult && this.ocrResult.addressComponents) {
        // this.apartmentName = ''; // 아파트명은 빈칸으로 유지
        this.buildingDong = this.ocrResult.addressComponents.buildingDong || '';
        this.buildingHo = this.ocrResult.addressComponents.buildingHo || '';
      }
      
      // 3단계로 이동
      this.currentStep = 3;
    }
  },
  
  beforeUnmount() {
    this.closeCamera();
  }
};
</script>

<style scoped>
.verification-container {
  width: 100%;
  max-width: 390px;
  margin: 0 auto;
  min-height: 100vh;
  background-color: #f8f9fa;
  box-shadow: rgba(100, 100, 111, 0.2) 0px 7px 29px 0px;
}

.verification-view {
  display: flex;
  flex-direction: column;
  height: 100vh;
  background-color: #f8f9fa;
  /* 고정 헤더 높이만큼 상단 여백 확보 */
  padding-top: 64px;
}

/* 헤더 스타일 */
.verification-header {
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
  border: none;
  cursor: pointer;
}

.back-button:hover {
  background-color: rgba(255,255,255,0.28);
}

.header-title {
  font-size: 18px;
  font-weight: 600;
  color: #fff;
  margin: 0;
}

.header-spacer {
  width: 40px;
}

/* 메인 콘텐츠 */
.verification-content {
  flex: 1;
  overflow-y: auto;
  padding: 20px;
}

/* 진행 상태 */
.progress-section {
  margin-bottom: 24px;
}

.progress-bar {
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 20px;
  background-color: white;
  border-radius: 12px;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.06);
}

.progress-step {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 8px;
}

.step-circle {
  width: 40px;
  height: 40px;
  border-radius: 50%;
  background-color: #e9ecef;
  color: #666;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: 600;
  transition: all 0.3s ease;
}

.progress-step.active .step-circle {
  background-color: #4682b4;
  color: white;
}

.progress-step.completed .step-circle {
  background-color: #28a745;
  color: white;
}

.step-label {

  font-size: 12px;
  color: #666;
  font-weight: 500;
}

.progress-step.active .step-label {
  color: #4682b4;
  font-weight: 600;
}

.progress-line {
  width: 60px;
  height: 2px;
  background-color: #e9ecef;
  margin: 0 16px;
  transition: all 0.3s ease;
}

.progress-line.active {
  background-color: #4682b4;
}

/* 인증 카드 */
.verification-card, .completion-card {
  background-color: white;
  border-radius: 12px;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.06);
  overflow: hidden;
}

.card-header {
  padding: 24px 20px 20px;
  text-align: center;
  border-bottom: 1px solid #f8f9fa;
}

.card-icon {
  width: 64px;
  height: 64px;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  margin: 0 auto 16px;
}

.gps-icon {
  background-color: #e3f2fd;
  color: #1976d2;
}

.ocr-icon {
  background-color: #f3e5f5;
  color: #7b1fa2;
}

.card-header h2 {
  font-size: 20px;
  font-weight: 600;
  color: #333;
  margin: 0 0 8px 0;
}

.card-header p {
  font-size: 14px;
  color: #666;
  margin: 0;
}

.verification-content-area {
  padding: 20px;
  min-height: 200px;
  display: flex;
  flex-direction: column;
  justify-content: center;
}

/* 정보 아이템 */
.verification-info {
  display: flex;
  flex-direction: column;
  gap: 16px;
}

.info-item {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 16px;
  background-color: #f8f9fa;
  border-radius: 8px;
}

.info-item svg {
  color: #4682b4;
  flex-shrink: 0;
}

.info-item span {
  font-size: 14px;
  color: #666;
  line-height: 1.4;
}

/* 로딩 상태 */
.loading-state {
  text-align: center;
  padding: 40px 20px;
}

.loading-spinner {
  width: 40px;
  height: 40px;
  border: 4px solid #f3f3f3;
  border-top: 4px solid #4682b4;
  border-radius: 50%;
  animation: spin 1s linear infinite;
  margin: 0 auto 16px;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}

.loading-state p {
  font-size: 16px;
  color: #666;
  margin: 0;
}

/* 성공 상태 */
.success-state {
  text-align: center;
  padding: 1rem;
}

.success-content {
  padding: 1.5rem 0;
}

.success-icon {
  color: var(--main);
  margin-bottom: 1rem;
}

.success-content h3 {
  color: var(--main);
  margin-bottom: 0.5rem;
  font-size: 1.25rem;
  font-weight: 600;
}

.location-info {
  color: var(--deepgray);
  font-size: 0.95rem;
  line-height: 1.4;
  background: #f8f9fa;
  padding: 1rem;
  border-radius: 8px;
  margin-top: 1rem;
}

/* OCR 결과 스타일링 */
.address-match-info {
  text-align: left;
  background: #f8f9fa;
  padding: 1.5rem;
  border-radius: 12px;
  margin-top: 1rem;
}

.extracted-address {
  font-size: 0.95rem;
  color: var(--deepgray);
  margin-bottom: 0.5rem;
  font-weight: 500;
}

.match-score {
  font-size: 0.9rem;
  color: var(--deepgray);
  margin-bottom: 0.75rem;
}

.match-status {
  font-size: 1rem;
  font-weight: 600;
  padding: 0.75rem 1rem;
  border-radius: 8px;
  text-align: center;
}

.match-status.success {
  background: #d4edda;
  color: #155724;
  border: 1px solid #c3e6cb;
}

.match-status.warning {
  background: #fff3cd;
  color: #856404;
  border: 1px solid #ffeaa7;
}

/* 이미지 컨테이너 및 오버레이 스타일 */
.image-container {
  position: relative;
  display: inline-block;
  width: 100%;
}

.image-overlay {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  border-radius: 8px;
  backdrop-filter: blur(2px);
}

.loading-overlay {
  background: rgba(255, 255, 255, 0.9);
}

.success-overlay {
  background: rgba(76, 175, 80, 0.9);
  color: white;
}

.success-overlay .success-icon {
  color: white;
  margin-bottom: 0.5rem;
}

.success-overlay h3 {
  color: white;
  font-size: 1.1rem;
  font-weight: 600;
  margin: 0;
}

/* 아파트 정보 입력 폼 스타일 */
.apartment-form {
  padding: 0;
}

.form-section {
  margin-bottom: 2rem;
  padding: 0 1rem;
}

.section-title {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  font-size: 1.1rem;
  font-weight: 600;
  color: var(--main);
  margin-bottom: 1.5rem;
  padding-bottom: 0.75rem;
  border-bottom: 2px solid var(--main);
  opacity: 0.8;
}

.section-title svg {
  color: var(--main);
}

.input-group {
  margin-bottom: 1.5rem;
}

.input-label {
  display: flex;
  align-items: center;
  gap: 0.25rem;
  font-size: 0.9rem;
  font-weight: 500;
  color: var(--deepgray);
  margin-bottom: 0.5rem;
}

.label-text {
  color: #333;
}

.label-required {
  color:  rgba(76, 175, 80, 0.9);
  font-weight: 600;
}

.input-wrapper {
  position: relative;
  display: flex;
  align-items: center;
}

.form-input {
  width: 100%;
  padding: 1rem 3rem 1rem 1rem;
  border: 2px solid #e9ecef;
  border-radius: 12px;
  font-size: 1rem;
  background: #fff;
  transition: all 0.3s ease;
  box-sizing: border-box;
}

.form-input:focus {
  outline: none;
  border-color: var(--main);
  box-shadow: 0 0 0 3px rgba(70, 130, 180, 0.15);
  transform: translateY(-1px);
}

.form-input:disabled {
  background-color: #f8f9fa;
  color: #6c757d;
  cursor: not-allowed;
}

.form-input::placeholder {
  color: #adb5bd;
}

.input-icon {
  position: absolute;
  right: 1rem;
  color: #adb5bd;
  pointer-events: none;
}

.form-row {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 1rem;
}

.ocr-info {
  display: flex;
  align-items: flex-start;
  gap: 0.5rem;
  padding: 0.75rem 1rem;
  background: linear-gradient(135deg, rgba(70, 130, 180, 0.08) 0%, rgba(70, 130, 180, 0.05) 100%);
  border-radius: 8px;
  border-left: 4px solid var(--main);
  margin-top: 1rem;
  border: 1px solid rgba(70, 130, 180, 0.1);
}

.info-icon {
  color: var(--main);
  flex-shrink: 0;
  margin-top: 0.1rem;
}

.ocr-info span {
  font-size: 0.85rem;
  color: #555;
  line-height: 1.4;
}

/* 에러 상태 */
.error-state {
  text-align: center;
  padding: 40px 20px;
}

.error-icon {
  color: #dc3545;
  margin-bottom: 16px;
}

.error-state h3 {
  font-size: 18px;
  font-weight: 600;
  color: #333;
  margin: 0 0 8px 0;
}

.error-message {
  font-size: 14px;
  color: #dc3545;
  margin: 0;
  padding: 12px 16px;
  background-color: #f8d7da;
  border-radius: 8px;
  margin-top: 16px;
}

/* 업로드 영역 */
.upload-area {
  text-align: center;
}

.upload-placeholder {
  padding: 40px 20px;
  border: 2px dashed #ddd;
  border-radius: 12px;
  margin-bottom: 20px;
}

.upload-placeholder svg {
  color: #ccc;
  margin-bottom: 16px;
}

.upload-placeholder h3 {
  font-size: 18px;
  font-weight: 600;
  color: #333;
  margin: 0 0 8px 0;
}

.upload-placeholder p {
  font-size: 14px;
  color: #666;
  margin: 0;
  line-height: 1.4;
}

.upload-tips {
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.tip-item {
  display: flex;
  align-items: flex-start;
  gap: 8px;
  font-size: 12px;
  color: #666;
}

.tip-item svg {
  color: #4682b4;
  flex-shrink: 0;
  margin-top: 2px;
}

/* 이미지 미리보기 */
.image-preview {
  text-align: center;
}

.image-preview img {
  width: 100%;
  max-width: 300px;
  height: auto;
  border-radius: 8px;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
  margin-bottom: 20px;
}

.image-actions {
  display: flex;
  gap: 12px;
  justify-content: center;
}

/* 버튼 스타일 */
.verification-actions {
  border-top: 1px solid #f8f9fa;
}

.primary-button {
  width: 100%;
  background-color: #4682b4;
  color: white;
  border: none;
  border-radius: 8px;
  padding: 14px 16px;
  font-size: 16px;
  font-weight: 600;
  cursor: pointer;
  transition: background-color 0.2s;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
}

.primary-button:hover:not(:disabled) {
  background-color: #3a6b9a;
}

.primary-button:disabled {
  background-color: #ccc;
  cursor: not-allowed;
}

.secondary-button {
  background-color: #f8f9fa;
  color: #666;
  border: 1px solid #e9ecef;
  border-radius: 8px;
  padding: 12px 16px;
  font-size: 14px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s;
}

.secondary-button:hover {
  background-color: #e9ecef;
}

/* 완료 카드 */
.completion-card {
  text-align: center;
  padding: 40px 20px;
}

.completion-icon {
  color: #28a745;
  margin-bottom: 24px;
}

.completion-card h2 {
  font-size: 24px;
  font-weight: 700;
  color: #333;
  margin: 0 0 8px 0;
}

.completion-card > p {
  font-size: 16px;
  color: #666;
  margin: 0 0 32px 0;
}

.completion-info {
  background-color: #f8f9fa;
  border-radius: 8px;
  padding: 20px;
  margin-bottom: 32px;
}

.info-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 8px 0;
}

.info-row:not(:last-child) {
  border-bottom: 1px solid #e9ecef;
}

.info-row .label {
  font-size: 14px;
  color: #666;
}

.info-row .value {
  font-size: 14px;
  font-weight: 500;
  color: #333;
}

/* 카메라 모달 */
.camera-modal {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: rgba(0, 0, 0, 0.9);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 1000;
}

.camera-container {
  width: 100%;
  max-width: 390px;
  height: 100vh;
  background-color: black;
  display: flex;
  flex-direction: column;
}

.camera-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 16px 20px;
  background-color: rgba(0, 0, 0, 0.8);
  color: white;
}

.camera-header h3 {
  font-size: 18px;
  font-weight: 600;
  margin: 0;
}

.close-button {
  width: 40px;
  height: 40px;
  border-radius: 50%;
  background-color: rgba(255, 255, 255, 0.2);
  color: white;
  border: none;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
}

.camera-content {
  flex: 1;
  position: relative;
  overflow: hidden;
}

.camera-content video {
  width: 100%;
  height: 100%;
  object-fit: cover;
}

.camera-overlay {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  background-color: rgba(0, 0, 0, 0.3);
}

.id-card-frame {
  width: 280px;
  height: 180px;
  position: relative;
  border: 2px solid rgba(255, 255, 255, 0.8);
  border-radius: 8px;
}

.frame-corner {
  position: absolute;
  width: 20px;
  height: 20px;
  border: 3px solid white;
}

.frame-corner.top-left {
  top: -3px;
  left: -3px;
  border-right: none;
  border-bottom: none;
}

.frame-corner.top-right {
  top: -3px;
  right: -3px;
  border-left: none;
  border-bottom: none;
}

.frame-corner.bottom-left {
  bottom: -3px;
  left: -3px;
  border-right: none;
  border-top: none;
}

.frame-corner.bottom-right {
  bottom: -3px;
  right: -3px;
  border-left: none;
  border-top: none;
}

.camera-guide {
  color: white;
  font-size: 14px;
  margin-top: 20px;
  text-align: center;
}

.camera-actions {
  padding: 20px;
  display: flex;
  justify-content: center;
  background-color: rgba(0, 0, 0, 0.8);
}

.capture-button {
  width: 80px;
  height: 80px;
  border-radius: 50%;
  background-color: white;
  border: 4px solid rgba(255, 255, 255, 0.3);
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: all 0.2s;
}

.capture-button:hover {
  transform: scale(1.05);
}

/* 로딩 스피너 애니메이션 */
.loading-spinner {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 16px;
}

.spinner-circle {
  width: 48px;
  height: 48px;
  border: 4px solid rgba(255, 255, 255, 0.3);
  border-top: 4px solid #ffffff;
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}

/* 성공 오버레이 스타일 */
.success-overlay {
  background-color: rgba(34, 197, 94, 0.95) !important;
  color: white;
}

.success-overlay .success-icon {
  margin-bottom: 16px;
  animation: checkmark-appear 0.6s ease-out;
}

@keyframes checkmark-appear {
  0% {
    transform: scale(0);
    opacity: 0;
  }
  50% {
    transform: scale(1.2);
  }
  100% {
    transform: scale(1);
    opacity: 1;
  }
}

.parsed-address-info {
  margin-top: 16px;
  padding: 16px;
  background-color: rgba(255, 255, 255, 0.1);
  border-radius: 8px;
  width: 100%;
  max-width: 280px;
}

.full-address {
  font-size: 16px;
  font-weight: 600;
  color: white;
  text-align: center;
  line-height: 1.4;
  word-break: keep-all;
}

/* 실패 오버레이 스타일 */
.failure-overlay {
  background-color: rgba(239, 68, 68, 0.95) !important;
  color: white;
}

.failure-overlay .failure-icon {
  margin-bottom: 16px;
  animation: error-shake 0.6s ease-out;
}

@keyframes error-shake {
  0%, 100% { transform: translateX(0); }
  10%, 30%, 50%, 70%, 90% { transform: translateX(-5px); }
  20%, 40%, 60%, 80% { transform: translateX(5px); }
}

.mismatch-info {
  margin-top: 16px;
  padding: 16px;
  background-color: rgba(255, 255, 255, 0.1);
  border-radius: 8px;
  width: 100%;
  max-width: 280px;
}

.mismatch-item {
  margin-bottom: 8px;
  font-size: 12px;
}

.mismatch-item .label {
  display: block;
  font-weight: 600;
  color: rgba(255, 255, 255, 0.8);
  margin-bottom: 2px;
}

.mismatch-item .value {
  display: block;
  font-weight: 400;
  color: white;
  word-break: break-all;
}

/* 오버레이 공통 스타일 개선 */
.image-overlay {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: rgba(0, 0, 0, 0.8);
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  color: white;
  text-align: center;
  padding: 20px;
  border-radius: 8px;
  animation: overlay-appear 0.3s ease-out;
}

@keyframes overlay-appear {
  0% {
    opacity: 0;
    transform: scale(0.9);
  }
  100% {
    opacity: 1;
    transform: scale(1);
  }
}

.image-overlay h3 {
  margin: 0 0 8px 0;
  font-size: 18px;
  font-weight: 600;
}

.image-overlay p {
  margin: 0;
  font-size: 14px;
  opacity: 0.9;
}

/* 스크롤바 숨기기 */
.verification-content::-webkit-scrollbar {
  display: none;
}

.verification-content {
  -ms-overflow-style: none;
  scrollbar-width: none;
}
</style>
