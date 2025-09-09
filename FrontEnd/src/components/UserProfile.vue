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
import { ref } from 'vue';

// 기본 프로필 이미지
const defaultProfileImage = 'https://images.unsplash.com/photo-1507003211169-0a1dd7228f2d?w=150&h=150&fit=crop&crop=face';

// 사용자 정보 (실제로는 props나 store에서 가져올 데이터)
const userInfo = ref({
  name: '김승호',
  profileImage: null, // 실제 프로필 이미지 URL
  location: {
    city: '강남구',
    district: '역삼동',
    apartmentName: '역삼래미안',
    building: '105',
    unit: '1301'
  }
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