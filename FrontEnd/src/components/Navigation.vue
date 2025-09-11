<template>
  <nav class="navigation">
    <div class="nav-links">
      <!-- router-link를 사용한 네비게이션 -->
      <router-link to="/home" class="nav-link">홈</router-link>

      <!-- 로그인 상태에 따른 조건부 렌더링 -->
      <template v-if="!isLoggedIn">
        <router-link to="/login" class="nav-link">로그인</router-link>
      </template>

      <template v-else>
        <router-link to="/mypage" class="nav-link">마이페이지</router-link>
        <button @click="handleLogout" class="nav-link logout-btn">
          로그아웃
        </button>
      </template>

      <router-link to="/search" class="nav-link">검색</router-link>
    </div>
  </nav>
</template>

<script>
import { computed } from "vue";
import { useRouter } from "vue-router";
import { useAuthStore } from "@/stores/auth.js";

export default {
  name: "Navigation",
  setup() {
    const router = useRouter();
    const authStore = useAuthStore();

    const isLoggedIn = computed(() => authStore.isLoggedIn);

    const handleLogout = () => {
      authStore.logout();
      router.push("/login");
    };

    return {
      isLoggedIn,
      handleLogout,
    };
  },
};
</script>

<style scoped>
.navigation {
  background: white;
  padding: 1rem;
  border-bottom: 1px solid #eee;
}

.nav-links {
  display: flex;
  gap: 1rem;
  justify-content: center;
}

.nav-link {
  padding: 0.5rem 1rem;
  border-radius: 4px;
  transition: background-color 0.2s;
}

.nav-link:hover {
  background-color: #f5f5f5;
}

/* 현재 활성화된 링크 스타일 */
.nav-link.router-link-active {
  background-color: var(--main);
  color: white;
}

/* 로그아웃 버튼 스타일 */
.logout-btn {
  background: none;
  border: none;
  cursor: pointer;
  font-size: inherit;
  color: inherit;
  text-decoration: none;
}

.logout-btn:hover {
  background-color: #f5f5f5;
}
</style>
