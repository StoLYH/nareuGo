<template>
  <div class="user-profile-card">
    <div class="profile-image-container">
      <img 
        :src="userInfo.profileImage || defaultProfileImage" 
        :alt="userInfo.name + ' 프로필 사진'"
        class="profile-image"
      />
    </div>

    <div class="profile-info">
      <div class="name-section">
        <h2 class="user-name">{{ userInfo.name }}</h2>
        <p v-if="localName" class="user-real-name">{{ localName }}</p>
      </div>

      <div class="location-section">
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
          <path d="M12 2C8.13 2 5 5.13 5 9C5 14.25 12 22 12 22S19 14.25 19 9C19 5.13 15.87 2 12 2ZM12 11.5C10.62 11.5 9.5 10.38 9.5 9C9.5 7.62 10.62 6.5 12 6.5C13.38 6.5 14.5 7.62 14.5 9C14.5 10.38 13.38 11.5 12 11.5Z" fill="currentColor" />
        </svg>
        <span class="location-text">
          {{ userInfo.location.city }} {{ userInfo.location.district }}
        </span>
      </div>

      <div class="apartment-section">
        {{ userInfo.location.apartmentName }} {{ userInfo.location.building }}동 {{ userInfo.location.unit }}호
      </div>
    </div>
    
    <!-- <div class="profile-actions-minimal">
        <a href="#" class="action-link">프로필 수정</a>
        <span class="link-divider">|</span>
        <a href="#" class="action-link" @click.prevent="handleLogout">로그아웃</a>
    </div> -->
  </div>
</template>

<script setup>
import { computed, ref, onMounted } from 'vue';
import { useAuthStore } from '@/stores/auth';
import { useRouter } from 'vue-router';
import apiClient from '@/api/client';

// 랜덤 프로필 이미지 배열
const randomProfileImages = [
  'https://images.unsplash.com/photo-1507003211169-0a1dd7228f2d?w=150&h=150&fit=crop&crop=face',
  'https://images.unsplash.com/photo-1472099645785-5658abf4ff4e?w=150&h=150&fit=crop&crop=face',
  'https://images.unsplash.com/photo-1500648767791-00dcc994a43e?w=150&h=150&fit=crop&crop=face',
  'https://images.unsplash.com/photo-1494790108755-2616b612b786?w=150&h=150&fit=crop&crop=face',
  'https://images.unsplash.com/photo-1506794778202-cad84cf45f1d?w=150&h=150&fit=crop&crop=face',
  'https://images.unsplash.com/photo-1517841905240-472988babdf9?w=150&h=150&fit=crop&crop=face',
  'https://images.unsplash.com/photo-1534528741775-53994a69daeb?w=150&h=150&fit=crop&crop=face',
  'https://images.unsplash.com/photo-1524504388940-b1c1722653e1?w=150&h=150&fit=crop&crop=face',
  'https://images.unsplash.com/photo-1507591064344-4c6ce005b128?w=150&h=150&fit=crop&crop=face',
  'https://images.unsplash.com/photo-1544005313-94ddf0286df2?w=150&h=150&fit=crop&crop=face'
];

// 랜덤 기본 프로필 이미지 선택
const getRandomProfileImage = () => {
  const randomIndex = Math.floor(Math.random() * randomProfileImages.length);
  return randomProfileImages[randomIndex];
};

const defaultProfileImage = getRandomProfileImage();

const authStore = useAuthStore();
const router = useRouter();
const myPageData = ref(null);
const localName = ref('');

onMounted(async () => {
  if (!authStore.user && authStore.accessToken) {
    try {
      await authStore.getUserInfo();
    } catch (e) {
      console.error('UserProfile - getUserInfo 실패:', e);
    }
  }

  if (authStore.user?.userId) {
    try {
      const response = await apiClient.get(`/mypage/user/${authStore.user.userId}`);
      myPageData.value = response.data;
    } catch (error) {
      console.error('마이페이지 정보 조회 실패:', error);
    }
  }

  // 로컬스토리지에서 이름 불러오기 (키 후보들을 유연하게 체크)
  try {
    const storedUser = localStorage.getItem('user') || localStorage.getItem('auth_user');
    if (storedUser) {
      const parsed = JSON.parse(storedUser);
      localName.value = parsed.name || parsed.realName || '';
    } else {
      // 혹시 개별로 저장된 경우도 대비
      localName.value = localStorage.getItem('name') || '';
    }
  } catch (e) {
    // 파싱 에러 시 무시하고 넘어감
    localName.value = localStorage.getItem('name') || '';
  }
});

const userInfo = computed(() => {
  const u = authStore.user || {};
  if (myPageData.value) {
    return {
      name: u.nickname || u.name || '사용자',
      profileImage: null,
      location: {
        city: myPageData.value.siGunGu || '',
        district: myPageData.value.eupMyeonDong || '',
        apartmentName: myPageData.value.apartName || '',
        building: myPageData.value.buildingDong || '',
        unit: myPageData.value.buildingHo || ''
      }
    };
  }
  return {
    name: u.nickname || u.name || '사용자',
    profileImage: u.profileImageUrl || u.profileImage || null,
    location: {
      city: u.siGunGu || '',
      district: u.eupMyeonDong || '',
      apartmentName: u.apartmentName || '',
      building: u.buildingDong || '',
      unit: u.buildingHo || ''
    }
  };
});

const handleLogout = async () => {
  try {
    await authStore.logout();
    router.push('/login');
  } catch (error) {
    console.error('로그아웃 실패:', error);
  }
};
</script>

<style scoped>
.user-profile-card {
  display: flex;
  flex-direction: column;
  align-items: center;
  background: var(--surface, #fff);
  padding: 28px 20px; /* 카드 패딩 조정 */
  border-radius: 16px;
  box-shadow: 0 6px 18px rgba(0,0,0,0.08);
  font-family: 'Pretendard', sans-serif;
  gap: 14px; /* 전체 요소 간격 */
  text-align: center;
}

.profile-image-container {
  width: 100px;
  height: 100px;
  border-radius: 50%;
  overflow: hidden;
  border: 3px solid #e6edf3;
  box-shadow: inset 0 0 0 2px rgba(70,130,180,0.06);
}

.profile-image {
  width: 100%;
  height: 100%;
  object-fit: cover;
}

.profile-info {
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.user-name {
  font-size: 24px; /* 폰트 크기 증가 */
  font-weight: 700;
  color: var(--secondary, #2c3e50);
  margin: 0;
}

.user-real-name {
  margin: 10px 0 0 0;
  font-size: 14px;
  font-weight: 600;
  color: var(--primary, #4682b4);
  background: rgba(70,130,180,0.08);
  padding: 6px 10px;
  border-radius: 999px;
  display: inline-block;
}

.location-section {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 4px;
  color: var(--muted, #6b7280);
  font-size: 14px;
  font-weight: 500;
}

.apartment-section {
  font-size: 14px;
  color: var(--muted, #6b7280);
  font-weight: 500;
}

/* subtle divider */
.name-section + .location-section {
  margin-top: 6px;
}

/* 간소화된 버튼 영역 */
.profile-actions-minimal {
    display: flex;
    align-items: center;
    justify-content: center;
    margin-top: 8px; /* 위쪽 여백 */
    gap: 8px;
}

.action-link {
    font-size: 14px;
    font-weight: 600;
    color: var(--primary, #4682b4);
    text-decoration: none;
    transition: color 0.2s ease;
}

.action-link:hover {
    color: var(--primary-600, #3a6b9a);
}

.link-divider {
    color: #a0a0a0;
}
</style>