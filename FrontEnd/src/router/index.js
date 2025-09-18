import { createRouter, createWebHistory } from 'vue-router'                                                                                                                       
import { useAuthStore } from '@/stores/auth'                                                                                                                                      

const routes = [
{
    path: '/',
    name: 'login',
    component: () => import('@/views/Login.vue'),
    meta: { requiresGuest: true }
},
{
    path: '/home',
    name: 'HomePage',
    component: () => import('@/views/ItemList.vue'),
    meta: { requiresAuth: true }
},
{
    path: '/items',
    name: 'ItemList',
    component: () => import('@/views/ItemList.vue'),
    meta: { requiresAuth: true }
},
{
    path: '/item/:id',
    name: 'ItemDetail',
    component: () => import('@/views/ItemDetailView.vue'),
    meta: { requiresAuth: true }
},
{
    path: '/item/register',
    name: 'ItemRegister',
    component: () => import('@/views/ItemRegistView.vue'),
    meta: { requiresAuth: true }
},
{
    path: '/search',
    name: 'Search',
    component: () => import('@/views/search.vue'),
    meta: { requiresAuth: true }
},
{
    path: '/chat',
    name: 'Chat',
    component: () => import('@/views/chatting.vue'),
    meta: { requiresAuth: true }
},
{
    path: '/chat/:id',
    name: 'ChatMessage',
    component: () => import('@/views/chatMessage.vue'),
    meta: { requiresAuth: true }
},
{
    path: '/profile',
    name: 'Profile',
    component: () => import('@/views/MyPage.vue'),
    meta: { requiresAuth: true }
},
{
    path: '/notifications',
    name: 'Notifications',
    component: () => import('@/views/NotificationListView.vue'),
    meta: { requiresAuth: true }
},
{
    path: '/neighborhood-verification',
    name: 'NeighborhoodVerification',
    component: () => import('@/views/NeighborhoodVerificationView.vue'),
    meta: { requiresAuth: true }
},
{
    path: '/payment/:orderId',
    name: 'PaymentDetail',
    component: () => import('@/views/PaymentDetailView.vue'),
    meta: { requiresAuth: true }
},
{
    path: '/payment/success',
    name: 'PaymentSuccess',
    component: () => import('@/views/PaymentSuccessView.vue'),
    meta: { requiresAuth: true }
},
{
    path: '/payment/fail',
    name: 'PaymentFail',
    component: () => import('@/views/PaymentFailView.vue'),
    meta: { requiresAuth: true }
},
{
    path: '/delivery/:orderId',
    name: 'DeliveryStatus',
    component: () => import('@/views/DeliveryStatus.vue'),
    meta: { requiresAuth: true }
},
{
    path: '/delivery-status',
    name: 'DeliveryStatusPage',
    component: () => import('@/views/DeliveryStatus.vue'),
    meta: { requiresAuth: true }
},
{
    path: '/login',
    name: 'Login',
    component: () => import('@/views/Login.vue'),
    meta: { requiresGuest: true }
},
{
    path: '/auth/success',
    name: 'OAuth2Callback',
    component: () => import('@/views/auth/OAuth2Callback.vue'),
    meta: { skipAuth: true }
},
{
    path: '/auth/error',
    name: 'OAuth2Error',
    component: () => import('@/views/auth/OAuth2Error.vue')
}
]

const router = createRouter({
history: createWebHistory(),
routes
})

// 전역 네비게이션 가드                                                                                                                                                           
router.beforeEach(async (to, from, next) => {
const authStore = useAuthStore()

// 인증 체크를 건너뛰는 페이지
if (to.meta.skipAuth) {
    return next()
}

// OAuth2 콜백에서 토큰이 포함된 경우 인증 체크 건너뛰기
if (to.query.token && (to.path === '/home' || to.path === '/')) {
    return next()
}

// 인증이 필요한 페이지                                                                                                                                                         
if (to.meta.requiresAuth) {
    // 토큰도 없고 사용자 정보도 없으면 바로 로그인으로
    if (!authStore.accessToken && !authStore.user) {
        authStore.clearAuth()
        return next('/login')
    }
    
    // 토큰은 있지만 사용자 정보가 없는 경우 조회 시도                                                                                                                          
    if (authStore.accessToken && !authStore.user) {
        try {
            await authStore.getUserInfo()
        } catch (error) {
            authStore.clearAuth()
            return next('/login')
        }
    }

    // 최종 인증 상태 확인
    if (!authStore.isAuthenticated) {
        authStore.clearAuth()
        return next('/login')
    }
}

// 게스트만 접근 가능한 페이지 (로그인 페이지 등)                                                                                                                               
if (to.meta.requiresGuest && authStore.isAuthenticated) {
    return next('/')
}

next()
})

export default router