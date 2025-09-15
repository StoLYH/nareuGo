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
              <span class="step-label">위치 인증</span>
            </div>
            <div class="progress-line" :class="{ active: currentStep >= 2 }"></div>
            <div class="progress-step" :class="{ active: currentStep >= 2, completed: ocrVerified }">
              <div class="step-circle">
                <svg v-if="ocrVerified" width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <polyline points="20 6 9 17 4 12" stroke="white" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                </svg>
                <span v-else>2</span>
              </div>
              <span class="step-label">주소 인증</span>
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
              <div class="success-icon">
                <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M22 11.08V12a10 10 0 1 1-5.93-9.14" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                  <polyline points="22 4 12 14.01 9 11.01" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                </svg>
              </div>
              <h3>위치 인증 완료!</h3>
              <p class="location-info">{{ verifiedLocation }}</p>
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
            <button v-if="!gpsVerified" @click="startGpsVerification" :disabled="gpsLoading" class="primary-button">
              {{ gpsLoading ? '위치 확인 중...' : '위치 인증 시작' }}
            </button>
            <button v-if="gpsVerified" @click="nextStep" class="primary-button">다음 단계</button>
            <button v-if="gpsError" @click="retryGpsVerification" class="secondary-button">다시 시도</button>
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

            <div v-if="capturedImage && !ocrLoading && !ocrVerified" class="image-preview">
              <img :src="capturedImage" alt="촬영된 신분증" />
              <div class="image-actions">
                <button @click="retakePhoto" class="secondary-button">다시 촬영</button>
                <button @click="processOCR" class="primary-button">주소 확인</button>
              </div>
            </div>

            <div v-if="ocrLoading" class="loading-state">
              <div class="loading-spinner"></div>
              <p>신분증을 분석하고 있습니다...</p>
            </div>

            <div v-if="ocrVerified" class="success-state">
              <div class="success-icon">
                <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M22 11.08V12a10 10 0 1 1-5.93-9.14" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                  <polyline points="22 4 12 14.01 9 11.01" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                </svg>
              </div>
              <h3>주소 인증 완료!</h3>
              <p class="address-info">{{ verifiedAddress }}</p>
            </div>

            <div v-if="ocrError" class="error-state">
              <div class="error-icon">
                <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <circle cx="12" cy="12" r="10" stroke="currentColor" stroke-width="2" fill="none"/>
                  <line x1="15" y1="9" x2="9" y2="15" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
                  <line x1="9" y1="9" x2="15" y2="15" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
                </svg>
              </div>
              <h3>주소 인증 실패</h3>
              <p class="error-message">{{ ocrError }}</p>
            </div>
          </div>

          <div class="verification-actions">
            <button v-if="!capturedImage && !ocrVerified" @click="capturePhoto" class="primary-button">
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M23 19C23 20.1 22.1 21 21 21H3C1.9 21 1 20.1 1 19V8C1 6.9 1.9 6 3 6H7L9 4H15L17 6H21C22.1 6 23 6.9 23 8V19Z" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                <circle cx="12" cy="13" r="4" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
              </svg>
              신분증 촬영하기
            </button>
            <button v-if="ocrVerified" @click="completeVerification" class="primary-button">인증 완료</button>
            <button v-if="ocrError" @click="retryOCR" class="secondary-button">다시 시도</button>
          </div>
        </section>

        <!-- 완료 섹션 -->
        <section class="completion-card" v-if="currentStep === 3">
          <div class="completion-icon">
            <svg width="80" height="80" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M22 11.08V12a10 10 0 1 1-5.93-9.14" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
              <polyline points="22 4 12 14.01 9 11.01" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
            </svg>
          </div>
          <h2>동네 인증 완료!</h2>
          <p>이제 우리 동네에서 안전하게 거래할 수 있습니다.</p>
          
          <div class="completion-info">
            <div class="info-row">
              <span class="label">인증된 위치</span>
              <span class="value">{{ verifiedLocation }}</span>
            </div>
            <div class="info-row">
              <span class="label">인증된 주소</span>
              <span class="value">{{ verifiedAddress }}</span>
            </div>
          </div>

          <button @click="goToHome" class="primary-button">홈으로 가기</button>
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
export default {
  name: "NeighborhoodVerificationView",
  data() {
    return {
      currentStep: 1,
      
      // GPS 관련
      gpsLoading: false,
      gpsVerified: false,
      gpsError: null,
      verifiedLocation: '',
      
      // OCR 관련
      ocrLoading: false,
      ocrVerified: false,
      ocrError: null,
      verifiedAddress: '',
      capturedImage: null,
      
      // 카메라 관련
      showCamera: false,
      stream: null,
    };
  },
  methods: {
    goBack() {
      this.$router.go(-1);
    },
    
    nextStep() {
      this.currentStep = 2;
    },
    
    async startGpsVerification() {
      this.gpsLoading = true;
      this.gpsError = null;
      
      try {
        if (!navigator.geolocation) {
          throw new Error('이 브라우저는 위치 서비스를 지원하지 않습니다.');
        }
        
        const position = await new Promise((resolve, reject) => {
          navigator.geolocation.getCurrentPosition(
            resolve,
            reject,
            {
              enableHighAccuracy: true,
              timeout: 10000,
              maximumAge: 0
            }
          );
        });
        
        const { latitude, longitude } = position.coords;
        
        // 시뮬레이션을 위해 임시 지연
        await this.simulateDelay(2000);
        
        // Reverse Geocoding 시뮬레이션
        const address = await this.reverseGeocode(latitude, longitude);
        
        this.verifiedLocation = address;
        this.gpsVerified = true;
        
      } catch (error) {
        console.error('GPS 인증 오류:', error);
        this.gpsError = this.getGpsErrorMessage(error);
      } finally {
        this.gpsLoading = false;
      }
    },
    
    async reverseGeocode(lat, lng) {
      // 실제로는 카카오맵 API나 구글맵 API를 사용해야 합니다
      return '서울특별시 강남구 테헤란로 123';
    },
    
    getGpsErrorMessage(error) {
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
    },
    
    async processOCR() {
      this.ocrLoading = true;
      this.ocrError = null;
      
      try {
        // OCR 처리 시뮬레이션
        await this.simulateDelay(3000);
        
        const ocrResult = await this.performOCR(this.capturedImage);
        
        if (ocrResult.success) {
          this.verifiedAddress = ocrResult.address;
          this.ocrVerified = true;
        } else {
          throw new Error(ocrResult.error);
        }
        
      } catch (error) {
        console.error('OCR 처리 오류:', error);
        this.ocrError = error.message || 'OCR 처리 중 오류가 발생했습니다.';
      } finally {
        this.ocrLoading = false;
      }
    },
    
    async performOCR(imageData) {
      // 실제 OCR API 호출 로직이 들어갈 곳
      const random = Math.random();
      if (random > 0.2) { // 80% 성공률
        return {
          success: true,
          address: '서울특별시 강남구 테헤란로 123'
        };
      } else {
        return {
          success: false,
          error: '신분증을 인식할 수 없습니다. 더 선명하게 촬영해주세요.'
        };
      }
    },
    
    retryOCR() {
      this.ocrError = null;
      this.capturedImage = null;
    },
    
    completeVerification() {
      this.currentStep = 3;
    },
    
    goToHome() {
      this.$router.push('/');
    },
    
    simulateDelay(ms) {
      return new Promise(resolve => setTimeout(resolve, ms));
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
}

/* 헤더 스타일 */
.verification-header {
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
  border: none;
  cursor: pointer;
}

.back-button:hover {
  background-color: #f0f0f0;
}

.header-title {
  font-size: 18px;
  font-weight: 600;
  color: #333;
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
  padding: 40px 20px;
}

.success-icon {
  color: #28a745;
  margin-bottom: 16px;
}

.success-state h3 {
  font-size: 18px;
  font-weight: 600;
  color: #333;
  margin: 0 0 8px 0;
}

.location-info, .address-info {
  font-size: 14px;
  color: #666;
  margin: 0;
  padding: 12px 16px;
  background-color: #f8f9fa;
  border-radius: 8px;
  margin-top: 16px;
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
  padding: 20px;
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

/* 스크롤바 숨기기 */
.verification-content::-webkit-scrollbar {
  display: none;
}

.verification-content {
  -ms-overflow-style: none;
  scrollbar-width: none;
}
</style>
