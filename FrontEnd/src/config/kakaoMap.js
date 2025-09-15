// 카카오맵 API 설정
const KAKAO_API_KEY = import.meta.env.VITE_KAKAO_MAP_API_KEY;

// 카카오맵 API 설정
export const KAKAO_MAP_CONFIG = {
  apiKey: KAKAO_API_KEY,
  restApiUrl: "https://dapi.kakao.com/v2/local",
};

// 카카오맵 SDK 로드 함수
export const loadKakaoMapSDK = () => {
  return new Promise((resolve, reject) => {
    if (window.kakao && window.kakao.maps) {
      resolve(window.kakao);
      return;
    }

    const script = document.createElement("script");
    script.src = `//dapi.kakao.com/v2/maps/sdk.js?appkey=${KAKAO_API_KEY}&libraries=services&autoload=false`;
    script.onload = () => {
      window.kakao.maps.load(() => {
        resolve(window.kakao);
      });
    };
    script.onerror = () => reject(new Error("카카오맵 SDK 로드 실패"));
    document.head.appendChild(script);
  });
};

// 좌표를 주소로 변환하는 함수 (Reverse Geocoding) - JavaScript SDK 사용
export const coordToAddress = async (latitude, longitude) => {
  try {
    // 카카오맵 SDK 로드
    const kakao = await loadKakaoMapSDK();
    
    return new Promise((resolve, reject) => {
      const geocoder = new kakao.maps.services.Geocoder();
      
      geocoder.coord2Address(longitude, latitude, (result, status) => {
        if (status === kakao.maps.services.Status.OK) {
          const addressInfo = result[0];
          
          // 도로명 주소가 있으면 도로명 주소 사용, 없으면 지번 주소 사용
          if (addressInfo.road_address) {
            resolve({
              fullAddress: addressInfo.road_address.address_name,
              region1: addressInfo.road_address.region_1depth_name, // 시/도
              region2: addressInfo.road_address.region_2depth_name, // 시/군/구
              region3: addressInfo.road_address.region_3depth_name, // 동/면/읍
              roadName: addressInfo.road_address.road_name,
              buildingName: addressInfo.road_address.building_name,
              type: "road_address"
            });
          } else if (addressInfo.address) {
            resolve({
              fullAddress: addressInfo.address.address_name,
              region1: addressInfo.address.region_1depth_name,
              region2: addressInfo.address.region_2depth_name,
              region3: addressInfo.address.region_3depth_name,
              type: "address"
            });
          } else {
            reject(new Error("주소 정보를 찾을 수 없습니다."));
          }
        } else {
          reject(new Error(`카카오맵 Geocoder 오류: ${status}`));
        }
      });
    });
  } catch (error) {
    console.error("좌표를 주소로 변환 실패:", error);
    throw error;
  }
};

// 현재 위치 가져오기 (브라우저 Geolocation + 카카오맵 주소 변환)
export const getCurrentLocationWithAddress = async () => {
  try {
    // 1. 브라우저에서 현재 위치 가져오기
    const position = await new Promise((resolve, reject) => {
      if (!navigator.geolocation) {
        reject(new Error('이 브라우저는 위치 서비스를 지원하지 않습니다.'));
        return;
      }

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
    
    // 2. 카카오맵 API로 좌표를 주소로 변환
    const addressInfo = await coordToAddress(latitude, longitude);
    
    return {
      latitude,
      longitude,
      accuracy: position.coords.accuracy,
      ...addressInfo
    };
    
  } catch (error) {
    console.error('위치 정보 가져오기 실패:', error);
    throw error;
  }
};

// 주소로 좌표 검색하는 함수 (Geocoding) - JavaScript SDK 사용
export const addressToCoord = async (address) => {
  try {
    // 카카오맵 SDK 로드
    const kakao = await loadKakaoMapSDK();
    
    return new Promise((resolve, reject) => {
      const geocoder = new kakao.maps.services.Geocoder();
      
      geocoder.addressSearch(address, (result, status) => {
        if (status === kakao.maps.services.Status.OK) {
          const coords = result[0];
          resolve({
            latitude: parseFloat(coords.y),
            longitude: parseFloat(coords.x),
            address: coords.address_name
          });
        } else {
          reject(new Error(`카카오맵 Geocoder 오류: ${status}`));
        }
      });
    });
  } catch (error) {
    console.error("주소를 좌표로 변환 실패:", error);
    throw error;
  }
};

// 두 좌표 간의 거리 계산 (미터 단위)
export const calculateDistance = (lat1, lng1, lat2, lng2) => {
  const R = 6371e3; // 지구 반지름 (미터)
  const φ1 = lat1 * Math.PI / 180;
  const φ2 = lat2 * Math.PI / 180;
  const Δφ = (lat2 - lat1) * Math.PI / 180;
  const Δλ = (lng2 - lng1) * Math.PI / 180;

  const a = Math.sin(Δφ/2) * Math.sin(Δφ/2) +
          Math.cos(φ1) * Math.cos(φ2) *
          Math.sin(Δλ/2) * Math.sin(Δλ/2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

  return R * c; // 거리 (미터)
};
