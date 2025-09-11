<template>
  <div class="router-example">
    <h2>라우터 사용 예제</h2>

    <div class="button-group">
      <!-- router-link 사용 -->
      <router-link to="/login" class="btn"
        >router-link로 로그인 페이지 이동</router-link
      >

      <!-- 프로그래밍 방식 네비게이션 -->
      <button @click="goToLogin" class="btn">
        프로그래밍으로 로그인 페이지 이동
      </button>
      <button @click="goToMyPage" class="btn">마이페이지로 이동</button>
      <button @click="goBack" class="btn">뒤로가기</button>
    </div>

    <div class="info">
      <h3>현재 라우트 정보:</h3>
      <p><strong>경로:</strong> {{ $route.path }}</p>
      <p><strong>이름:</strong> {{ $route.name }}</p>
      <p><strong>쿼리:</strong> {{ $route.query }}</p>
      <p><strong>파라미터:</strong> {{ $route.params }}</p>
    </div>
  </div>
</template>

<script>
import { useRouter, useRoute } from "vue-router";

export default {
  name: "RouterExample",
  setup() {
    const router = useRouter();
    const route = useRoute();

    // 프로그래밍 방식 네비게이션
    const goToLogin = () => {
      router.push("/login");
    };

    const goToMyPage = () => {
      router.push({ name: "MyPage" });
    };

    const goBack = () => {
      router.go(-1); // 뒤로가기
    };

    return {
      goToLogin,
      goToMyPage,
      goBack,
    };
  },
};
</script>

<style scoped>
.router-example {
  padding: 2rem;
  max-width: 600px;
  margin: 0 auto;
}

.button-group {
  display: flex;
  flex-direction: column;
  gap: 1rem;
  margin-bottom: 2rem;
}

.btn {
  padding: 0.75rem 1.5rem;
  background: var(--main);
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  text-decoration: none;
  text-align: center;
  transition: background-color 0.2s;
}

.btn:hover {
  background: #7a1818;
}

.info {
  background: #f8f9fa;
  padding: 1rem;
  border-radius: 4px;
}

.info p {
  margin: 0.5rem 0;
}
</style>
