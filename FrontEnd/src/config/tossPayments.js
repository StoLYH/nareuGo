// 환경변수에서 클라이언트 키 읽기
const CLIENT_KEY = import.meta.env.VITE_TOSS_CLIENT_KEY;

// 토스페이먼츠 결제 설정
export const TOSS_PAYMENTS_CONFIG = {
  // 테스트 환경 설정
  clientKey: CLIENT_KEY,
  baseUrl: "https://api.tosspayments.com/v1",

  // 결제 성공/실패 URL
  successUrl: `${window.location.origin}/payment/success`,
  failUrl: `${window.location.origin}/payment/fail`,

  // 결제 수단 설정
  paymentMethods: {
    card: {
      name: "카드 결제",
      description: "신용카드, 체크카드",
    },
    account: {
      name: "계좌이체",
      description: "실시간 계좌이체",
    },
  },
};

// 토스페이먼츠 SDK 로드 함수
export const loadTossPaymentsSDK = () => {
  return new Promise((resolve, reject) => {
    if (window.TossPayments) {
      resolve(window.TossPayments);
      return;
    }

    const script = document.createElement("script");
    script.src = "https://js.tosspayments.com/v1/payment-widget";
    script.onload = () => resolve(window.TossPayments);
    script.onerror = () => reject(new Error("토스페이먼츠 SDK 로드 실패"));
    document.head.appendChild(script);
  });
};

// 결제 요청 함수
export const requestPayment = async (orderData) => {
  try {
    console.log("토스페이먼츠 SDK 로드 시작...");
    const TossPayments = await loadTossPaymentsSDK();
    console.log("토스페이먼츠 SDK 로드 완료:", TossPayments);

    const tossPayments = TossPayments(TOSS_PAYMENTS_CONFIG.clientKey);
    console.log("토스페이먼츠 인스턴스 생성 완료");

    const paymentData = {
      amount: orderData.amount,
      orderId: orderData.orderId.toString(),
      orderName: orderData.orderName,
      customerName: orderData.customerName,
      customerEmail: orderData.customerEmail,
      successUrl: TOSS_PAYMENTS_CONFIG.successUrl,
      failUrl: TOSS_PAYMENTS_CONFIG.failUrl,
    };

    console.log("결제 데이터:", paymentData);
    console.log("토스페이먼츠 결제창 호출 중...");

    await tossPayments.requestPayment("카드", paymentData);
    console.log("토스페이먼츠 결제창 호출 성공");
  } catch (error) {
    console.error("결제 요청 실패:", error);
    throw error;
  }
};

// 결제 승인 함수 (백엔드 API 호출)
export const confirmPayment = async (paymentData) => {
  try {
    const response = await fetch("/api/payments/confirm", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify(paymentData),
    });

    if (!response.ok) {
      throw new Error("결제 승인 실패");
    }

    return await response.json();
  } catch (error) {
    console.error("결제 승인 실패:", error);
    throw error;
  }
};
