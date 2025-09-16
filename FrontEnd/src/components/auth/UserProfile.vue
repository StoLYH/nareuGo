<template>                                                                                                                                                                        
    <div class="user-profile" v-if="authStore.isAuthenticated">                                                                                                                     
      <div class="profile-trigger" @click="toggleDropdown">
        <img
          :src="user?.profileImage || '/default-avatar.png'"
          :alt="user?.name"
          class="profile-avatar"
        />
        <span class="profile-name">{{ user?.name }}</span>
        <span class="dropdown-arrow">▼</span>
      </div>

      <div v-if="showDropdown" class="profile-dropdown">
        <div class="dropdown-item">
          <strong>{{ user?.name }}</strong>
          <small>{{ user?.email }}</small>
        </div>
        <hr />
        <button @click="handleLogout" class="dropdown-item logout-button">
          로그아웃
        </button>
      </div>
    </div>
</template>

<script setup>                                                                                                                                                                    
import { ref, computed, onMounted, onUnmounted } from 'vue'
import { useAuthStore } from '@/stores/auth'
import { useRouter } from 'vue-router'

const authStore = useAuthStore()
const router = useRouter()

const showDropdown = ref(false)
const user = computed(() => authStore.user)

const toggleDropdown = () => {
showDropdown.value = !showDropdown.value
}

const handleLogout = async () => {
try {
    await authStore.logout()
    router.push('/login')
} catch (error) {
    console.error('로그아웃 실패:', error)
}
}

// 드롭다운 외부 클릭 시 닫기
const handleClickOutside = (event) => {
if (!event.target.closest('.user-profile')) {
    showDropdown.value = false
}
}

onMounted(() => {
document.addEventListener('click', handleClickOutside)
})

onUnmounted(() => {
document.removeEventListener('click', handleClickOutside)
})
</script>                                                                                                                                                                         

<style scoped>                                                                                                                                                                    
.user-profile {
position: relative;
}

.profile-trigger {
display: flex;
align-items: center;
gap: 8px;
padding: 8px 12px;
border-radius: 8px;
cursor: pointer;
transition: background-color 0.2s;
}

.profile-trigger:hover {
background-color: #f7fafc;
}

.profile-avatar {
width: 32px;
height: 32px;
border-radius: 50%;
object-fit: cover;
}

.dropdown-arrow {
font-size: 12px;
color: #718096;
}

.profile-dropdown {
position: absolute;
top: 100%;
right: 0;
background: white;
border: 1px solid #e2e8f0;
border-radius: 8px;
box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
min-width: 200px;
z-index: 1000;
}

.dropdown-item {
padding: 12px;
display: block;
width: 100%;
text-align: left;
border: none;
background: none;
cursor: pointer;
}

.dropdown-item:hover {
background-color: #f7fafc;
}

.logout-button {
color: #e53e3e;
}
</style>