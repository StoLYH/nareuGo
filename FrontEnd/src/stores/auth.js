import { defineStore } from "pinia";
import { authAPI } from "@/api/auth.js";

export const useAuthStore = defineStore("auth", {
  state: () => ({
    user: null,
    token: null,
    isLoggedIn: false,
    isLoading: false,
  }),

  getters: {
    // 사용자 정보가 있는지 확인
    hasUser: (state) => !!state.user,

    // 로그인 상태 확인
    isAuthenticated: (state) => !!state.token,
  },

  actions: {
    // 로그인
    async login(credentials) {
      this.isLoading = true;

      try {
        const response = await authAPI.login(credentials);

        // 로그인 성공 시 상태 업데이트
        this.user = response.user;
        this.token = response.token;
        this.isLoggedIn = true;

        // 토큰을 localStorage에 저장
        localStorage.setItem("token", response.token);
        localStorage.setItem("isLoggedIn", "true");
        localStorage.setItem("user", JSON.stringify(response.user));

        return response;
      } catch (error) {
        // 로그인 실패 시 상태 초기화
        this.logout();
        throw error;
      } finally {
        this.isLoading = false;
      }
    },

    // 회원가입
    async signup(userData) {
      this.isLoading = true;

      try {
        const response = await authAPI.signup(userData);
        return response;
      } catch (error) {
        throw error;
      } finally {
        this.isLoading = false;
      }
    },

    // 로그아웃
    logout() {
      this.user = null;
      this.token = null;
      this.isLoggedIn = false;

      // localStorage에서 토큰 제거
      localStorage.removeItem("token");
      localStorage.removeItem("isLoggedIn");
      localStorage.removeItem("user");
    },

    // 페이지 새로고침 시 저장된 토큰으로 로그인 상태 복원
    restoreAuth() {
      const token = localStorage.getItem("token");
      const isLoggedIn = localStorage.getItem("isLoggedIn");
      const user = localStorage.getItem("user");

      if (token && isLoggedIn === "true" && user) {
        this.token = token;
        this.isLoggedIn = true;
        this.user = JSON.parse(user);
      }
    },
  },
});
