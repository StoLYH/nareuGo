// Firebase 설정 파일
// HTTP 환경에서도 FCM 토큰 생성은 가능 (제한된 기능)

// 정적 import로 Firebase 모듈 로드
import { initializeApp } from 'firebase/app'
import { getMessaging, getToken, onMessage } from 'firebase/messaging'

// Firebase 설정
const firebaseConfig = {
  apiKey: "AIzaSyAjuwpKSdrcpA3zG3B1scbogbNnq9jBNy0",
  authDomain: "nareugo.firebaseapp.com",
  projectId: "nareugo", // 백엔드와 동일한 프로젝트 ID
  storageBucket: "nareugo.firebasestorage.app",
  messagingSenderId: "922768085258",
  appId: "1:922768085258:web:9a574b1cc6f5de57f92af3",
  measurementId: "G-DWZ3P4VKJS"
};

// VAPID Key - Firebase 콘솔에서 받은 웹 푸시 인증서 키
const vapidKey = "BBJPbLxufI94JLhNOgoNuqf_CzpzPB2aAW8EsVb00ZvXjpLyBmw0f_NoieGbqN0r27uUWUvZ7USNlLKh-RGk39s";

let app = null
let messaging = null
let isFirebaseInitialized = false

// Firebase 초기화 (정적 import 사용)
async function initializeFirebase() {
  if (isFirebaseInitialized) return messaging;

  try {
    // Firebase 앱 초기화
    app = initializeApp(firebaseConfig);
    messaging = getMessaging(app);
    isFirebaseInitialized = true;

    console.log('Firebase 초기화 성공');
    return messaging;
  } catch (error) {
    console.error('Firebase 초기화 실패:', error);
    return null;
  }
}

// 알림 권한 요청
export const requestNotificationPermission = async () => {
  try {
    const permission = await Notification.requestPermission();
    console.log('알림 권한 상태:', permission);
    return permission === 'granted';
  } catch (error) {
    console.error('알림 권한 요청 실패:', error);
    return false;
  }
};

// FCM 토큰 생성
export const getFCMToken = async () => {
  try {
    const messaging = await initializeFirebase();
    if (!messaging) {
      console.warn('Firebase Messaging 초기화 실패');
      return null;
    }

    // Service Worker 등록 확인
    if (!('serviceWorker' in navigator)) {
      console.warn('Service Worker를 지원하지 않는 브라우저입니다.');
      return null;
    }

    // 알림 권한 확인
    const hasPermission = await requestNotificationPermission();
    if (!hasPermission) {
      console.warn('알림 권한이 거부되었습니다.');
      return null;
    }

    // FCM 토큰 생성
    const token = await getToken(messaging, { vapidKey });

    console.log('FCM 토큰 생성 성공:', token);
    return token;
  } catch (error) {
    console.error('FCM 토큰 생성 실패:', error);
    return null;
  }
};

// 메시지 수신 리스너
export const onMessageListener = () => {
  return new Promise(async (resolve) => {
    try {
      const messaging = await initializeFirebase();
      if (!messaging) {
        resolve(null);
        return;
      }

      onMessage(messaging, (payload) => {
        console.log('포그라운드 메시지 수신:', payload);
        resolve(payload);
      });
    } catch (error) {
      console.error('메시지 리스너 설정 실패:', error);
      resolve(null);
    }
  });
};

export { messaging, firebaseConfig };