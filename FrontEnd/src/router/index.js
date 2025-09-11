import { createRouter, createWebHistory } from "vue-router";
import ItemList from "@/views/ItemList.vue";
import ItemDetailView from "@/views/ItemDetailView.vue";
import ItemRegistView from "@/views/ItemRegistView.vue";
import Login from "@/views/Login.vue";
import MyPage from "@/views/MyPage.vue";
import DeliveryStatus from "@/views/DeliveryStatus.vue";
import NotificationListView from "@/views/NotificationListView.vue";
import chatting from "@/views/chatting.vue";
import chatMessage from "@/views/chatMessage.vue";
import search from "@/views/search.vue";

const routes = [
  {
    path: "/",
    name: "Login",
    component: Login,
  },
  {
    path: "/home",
    name: "ItemList",
    component: ItemList,
  },
  {
    path: "/item/:id",
    name: "ItemDetail",
    component: ItemDetailView,
  },
  {
    path: "/regist",
    name: "ItemRegist",
    component: ItemRegistView,
  },
  {
    path: "/login",
    name: "LoginPage",
    component: Login,
  },
  {
    path: "/mypage",
    name: "MyPage",
    component: MyPage,
  },
  {
    path: "/delivery",
    name: "DeliveryStatus",
    component: DeliveryStatus,
  },
  {
    path: "/notifications",
    name: "NotificationList",
    component: NotificationListView,
  },
  {
    path: "/chatting",
    name: "chatting",
    component: chatting,
  },
  {
    path: "/chatmessage",
    name: "chatMessage",
    component: chatMessage,
  },
  {
    path: "/search",
    name: "search",
    component: search,
  },
];

const router = createRouter({
  history: createWebHistory(),
  routes,
});

// 라우터 가드 설정
router.beforeEach((to, from, next) => {
  console.log(`페이지 이동: ${from.path} → ${to.path}`);

  const isLoggedIn = localStorage.getItem("isLoggedIn") === "true";

  // 루트 경로(/)는 항상 로그인 페이지로 리다이렉트
  if (to.path === "/") {
    if (isLoggedIn) {
      next("/home"); // 로그인된 사용자는 홈으로
    } else {
      next(); // 로그인되지 않은 사용자는 로그인 페이지 유지
    }
    return;
  }

  // 로그인 페이지 접근 시
  if (to.path === "/login") {
    if (isLoggedIn) {
      next("/home"); // 이미 로그인된 사용자는 홈으로 리다이렉트
    } else {
      next(); // 로그인되지 않은 사용자는 로그인 페이지 유지
    }
    return;
  }

  // 인증이 필요한 페이지 (로그인한 사용자만 접근 가능)
  const authRequiredPages = [
    "/home",
    "/item",
    "/regist",
    "/mypage",
    "/delivery",
    "/notifications",
    "/chatting",
    "/chatmessage",
    "/search",
  ];
  const isAuthRequired = authRequiredPages.some((page) =>
    to.path.startsWith(page)
  );

  if (isAuthRequired) {
    if (!isLoggedIn) {
      alert("로그인이 필요합니다.");
      next("/"); // 로그인 페이지로 리다이렉트
      return;
    }
  }

  next();
});

export default router;
