// 인증 관련 API 서비스
const API_BASE_URL = "http://localhost:8080/api";

export const authAPI = {
  // 로그인 API
  async login(credentials) {
    try {
      const response = await fetch(`${API_BASE_URL}/auth/login`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify(credentials),
      });

      if (!response.ok) {
        throw new Error("로그인에 실패했습니다.");
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error("로그인 API 오류:", error);
      throw error;
    }
  },

  // 회원가입 API
  async signup(userData) {
    try {
      const response = await fetch(`${API_BASE_URL}/auth/signup`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify(userData),
      });

      if (!response.ok) {
        throw new Error("회원가입에 실패했습니다.");
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error("회원가입 API 오류:", error);
      throw error;
    }
  },
};
