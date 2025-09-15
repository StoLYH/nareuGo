<template>
  <div class="map-container">
    <div ref="mapContainer" class="map"></div>
    <div v-if="loading" class="map-loading">
      <div class="loading-spinner"></div>
      <p>지도를 불러오는 중...</p>
    </div>
    <div v-if="currentAddress" class="location-info">
      <div class="location-icon">📍</div>
      <div class="location-text">
        <p class="address">{{ currentAddress }}</p>
        <p class="accuracy" v-if="accuracy">정확도: {{ Math.round(accuracy) }}m</p>
      </div>
    </div>
  </div>
</template>

<script>
import { loadKakaoMapSDK } from '../config/kakaoMap.js';

export default {
  name: 'KakaoMapComponent',
  props: {
    latitude: {
      type: Number,
      default: null
    },
    longitude: {
      type: Number,
      default: null
    },
    address: {
      type: String,
      default: ''
    },
    accuracy: {
      type: Number,
      default: null
    },
    height: {
      type: String,
      default: '300px'
    }
  },
  data() {
    return {
      map: null,
      marker: null,
      loading: true,
      currentAddress: this.address
    };
  },
  watch: {
    latitude(newLat) {
      if (newLat && this.longitude) {
        this.updateMapLocation(newLat, this.longitude);
      }
    },
    longitude(newLng) {
      if (this.latitude && newLng) {
        this.updateMapLocation(this.latitude, newLng);
      }
    },
    address(newAddress) {
      this.currentAddress = newAddress;
    }
  },
  async mounted() {
    await this.initializeMap();
  },
  methods: {
    async initializeMap() {
      try {
        const kakao = await loadKakaoMapSDK();
        
        // 기본 위치 (서울시청)
        const defaultLat = this.latitude || 37.5665;
        const defaultLng = this.longitude || 126.9780;
        
        const mapContainer = this.$refs.mapContainer;
        const mapOption = {
          center: new kakao.maps.LatLng(defaultLat, defaultLng),
          level: 3 // 지도 확대 레벨
        };
        
        this.map = new kakao.maps.Map(mapContainer, mapOption);
        
        // 마커 생성
        this.createMarker(defaultLat, defaultLng);
        
        this.loading = false;
        
        // props로 위치가 전달된 경우 해당 위치로 이동
        if (this.latitude && this.longitude) {
          this.updateMapLocation(this.latitude, this.longitude);
        }
        
      } catch (error) {
        console.error('지도 초기화 실패:', error);
        this.loading = false;
      }
    },
    
    createMarker(lat, lng) {
      if (!this.map) return;
      
      const kakao = window.kakao;
      const markerPosition = new kakao.maps.LatLng(lat, lng);
      
      // 기존 마커 제거
      if (this.marker) {
        this.marker.setMap(null);
      }
      
      // 새 마커 생성
      this.marker = new kakao.maps.Marker({
        position: markerPosition,
        map: this.map
      });
      
      // 정확도 원 표시 (accuracy가 있는 경우)
      if (this.accuracy && this.accuracy > 0) {
        const circle = new kakao.maps.Circle({
          center: markerPosition,
          radius: this.accuracy,
          strokeWeight: 2,
          strokeColor: '#4682b4',
          strokeOpacity: 0.8,
          fillColor: '#4682b4',
          fillOpacity: 0.2
        });
        circle.setMap(this.map);
      }
    },
    
    updateMapLocation(lat, lng) {
      if (!this.map) return;
      
      const kakao = window.kakao;
      const newPosition = new kakao.maps.LatLng(lat, lng);
      
      // 지도 중심 이동
      this.map.setCenter(newPosition);
      
      // 마커 업데이트
      this.createMarker(lat, lng);
      
      // 부드러운 이동 애니메이션
      this.map.panTo(newPosition);
    },
    
    // 외부에서 호출할 수 있는 메서드
    setLocation(lat, lng, address = '') {
      this.updateMapLocation(lat, lng);
      if (address) {
        this.currentAddress = address;
      }
    }
  }
};
</script>

<style scoped>
.map-container {
  position: relative;
  width: 100%;
  border-radius: 12px;
  overflow: hidden;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
}

.map {
  width: 100%;
  height: v-bind(height);
  min-height: 250px;
}

.map-loading {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(255, 255, 255, 0.9);
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  z-index: 10;
}

.loading-spinner {
  width: 40px;
  height: 40px;
  border: 4px solid #f3f3f3;
  border-top: 4px solid #4682b4;
  border-radius: 50%;
  animation: spin 1s linear infinite;
  margin-bottom: 10px;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}

.location-info {
  position: absolute;
  bottom: 0;
  left: 0;
  right: 0;
  background: linear-gradient(to top, rgba(0, 0, 0, 0.8), transparent);
  padding: 20px 15px 15px;
  display: flex;
  align-items: center;
  color: white;
}

.location-icon {
  font-size: 24px;
  margin-right: 12px;
  filter: drop-shadow(0 2px 4px rgba(0, 0, 0, 0.3));
}

.location-text {
  flex: 1;
}

.address {
  margin: 0;
  font-size: 16px;
  font-weight: 600;
  text-shadow: 0 1px 2px rgba(0, 0, 0, 0.5);
  line-height: 1.3;
}

.accuracy {
  margin: 4px 0 0 0;
  font-size: 12px;
  opacity: 0.9;
  text-shadow: 0 1px 2px rgba(0, 0, 0, 0.5);
}
</style>
