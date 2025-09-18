// Firebase 설정 파일 (HTTP 개발 환경에서는 비활성화)
// HTTPS 환경에서만 Firebase Messaging이 지원됩니다.

// 빈 export로 import 오류 방지
export const messaging = null;
export const getFCMToken = async () => null;
export const onMessageListener = () => Promise.resolve(null);
export const requestNotificationPermission = async () => false;