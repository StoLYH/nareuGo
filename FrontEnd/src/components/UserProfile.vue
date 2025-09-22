<template>
  <div class="user-profile">
    <div class="profile-image-container">
      <img 
        :src="userInfo.profileImage || defaultProfileImage" 
        :alt="userInfo.name + ' 프로필 사진'"
        class="profile-image"
      />
    </div>
    <div class="user-info">
      <h2 class="user-name">{{ userInfo.name }}</h2>
      <div class="location-info">
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M12 2C8.13 2 5 5.13 5 9C5 14.25 12 22 12 22S19 14.25 19 9C19 5.13 15.87 2 12 2ZM12 11.5C10.62 11.5 9.5 10.38 9.5 9C9.5 7.62 10.62 6.5 12 6.5C13.38 6.5 14.5 7.62 14.5 9C14.5 10.38 13.38 11.5 12 11.5Z" fill="currentColor" stroke="currentColor" stroke-width="0.5"/>
        </svg>
        <span class="location-text">
          {{ userInfo.location.city }}, {{ userInfo.location.district }} {{ userInfo.location.apartmentName }}
        </span>
      </div>
      <div class="apartment-info">
        {{ userInfo.location.building }}동 {{ userInfo.location.unit }}호
      </div>
    </div>
  </div>
</template>

<script setup>
import { computed, onMounted, ref } from 'vue';
import { useAuthStore } from '@/stores/auth';
import apiClient from '@/api/client';

// 기본 프로필 이미지
const defaultProfileImage = 'https://images.unsplash.com/photo-1507003211169-0a1dd7228f2d?w=150&h=150&fit=crop&crop=face';

const authStore = useAuthStore();
const myPageData = ref(null);

onMounted(async () => {
  console.log('UserProfile onMounted - authStore.user:', authStore.user);
  console.log('UserProfile onMounted - authStore.accessToken:', authStore.accessToken);

  if (!authStore.user && authStore.accessToken) {
    try {
      await authStore.getUserInfo();
      console.log('UserProfile - getUserInfo 후 user:', authStore.user);
    } catch (e) {
      console.error('UserProfile - getUserInfo 실패:', e);
    }
  }

  // 마이페이지 API 호출
  if (authStore.user?.userId) {
    console.log('UserProfile - API 호출 시도, userId:', authStore.user.userId);
    try {
      const response = await apiClient.get(`/mypage/user/${authStore.user.userId}`);
      console.log('UserProfile - API 응답:', response.data);
      myPageData.value = response.data;
    } catch (error) {
      console.error('마이페이지 정보 조회 실패:', error);
    }
  } else {
    console.log('UserProfile - userId가 없어서 API 호출 안함');
  }
});

// 스토어 사용자 정보를 화면 표시용으로 매핑
const userInfo = computed(() => {
  console.log('UserProfile userInfo computed - myPageData:', myPageData.value);
  console.log('UserProfile userInfo computed - authStore.user:', authStore.user);

  // 로컬스토리지에서 name 가져오기
  let displayName = '사용자';
  try {
    const userData = JSON.parse(localStorage.getItem('user') || '{}');
    displayName = userData.name || userData.nickname || '사용자';
    console.log('UserProfile - 로컬스토리지 name:', userData.name);
  } catch (error) {
    console.error('UserProfile - 로컬스토리지 파싱 실패:', error);
  }

  // 마이페이지 API 데이터 우선 사용
  if (myPageData.value) {
    const result = {
      name: displayName, // 로컬스토리지의 name 사용
      profileImage: null, // MyPageResponse에 프로필 이미지 없음
      location: {
        city: myPageData.value.siGunGu || '',
        district: myPageData.value.eupMyeonDong || '',
        apartmentName: myPageData.value.apartName || '',
        building: myPageData.value.buildingDong || '',
        unit: myPageData.value.buildingHo || ''
      }
    };
    console.log('UserProfile userInfo - API 데이터 사용:', result);
    return result;
  }

  // 기존 스토어 데이터 fallback
  const u = authStore.user || {};
  const result = {
    name: displayName, // 로컬스토리지의 name 사용
    profileImage: u.profileImageUrl || u.profileImage || null,
    location: {
      city: u.siGunGu || '',
      district: u.eupMyeonDong || '',
      apartmentName: u.apartmentName || '',
      building: u.buildingDong || '',
      unit: u.buildingHo || ''
    }
  };
  console.log('UserProfile userInfo - 스토어 데이터 사용:', result);
  return result;
});
</script>

<style scoped>
.user-profile {
  display: flex;
  align-items: center;
  padding: 24px 20px;
  background-color: #fff;
  gap: 16px;
}

.profile-image-container {
  flex-shrink: 0;
}

.profile-image {
  width: 64px;
  height: 64px;
  border-radius: 50%;
  object-fit: cover;
  border: 2px solid #f0f0f0;
}

.user-info {
  flex: 1;
  min-width: 0;
}

.user-name {
  font-size: 20px;
  font-weight: 500;
  color: #333;
  margin: 0 0 8px 0;
  
}

.location-info {
  display: flex;
  align-items: center;
  gap: 6px;
  margin-bottom: 4px;
}

.location-info svg {
  color: #666;
  flex-shrink: 0;
}

.location-text {
  font-size: 14px;
  color: #666;
  line-height: 1.4;
}

.apartment-info {
  font-size: 14px;
  color: #666;
  font-weight: 500;
  margin-left: 30px;
}
</style>