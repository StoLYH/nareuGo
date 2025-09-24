#!/usr/bin/env python3
"""
임베디드 시스템 시나리오 테스트
로봇이 판매자 집에 도착하는 상황을 시뮬레이션
"""

import requests
import time
import sys

# 설정
BACKEND_URL = "http://localhost:8080"
DELIVERY_ID = 1  # 테스트할 배송 ID

def log(message):
    """로그 출력"""
    print(f"[{time.strftime('%H:%M:%S')}] {message}")

def check_robot_status(robot_id="NAREUGO_001"):
    """로봇 상태 확인"""
    try:
        url = f"{BACKEND_URL}/robot/status"
        params = {"robotId": robot_id}

        response = requests.get(url, params=params, timeout=5)

        if response.status_code == 200:
            status = response.json()
            log(f"로봇 상태: {status.get('status', 'Unknown')} - {status.get('message', '')}")
            return status.get('status') == 'VALID'
        else:
            log(f"로봇 상태 확인 실패: {response.status_code}")
            return False

    except Exception as e:
        log(f"로봇 상태 확인 오류: {e}")
        return False

def notify_seller_arrival(delivery_id):
    """판매자 집 도착 알림"""
    try:
        url = f"{BACKEND_URL}/robot/seller-arrival"
        params = {"deliveryId": delivery_id}

        log(f"판매자 집 도착 알림 발송 중... (배송 ID: {delivery_id})")
        response = requests.post(url, params=params, timeout=10)

        if response.status_code == 200:
            log(f"✓ 알림 발송 성공: {response.text}")
            return True
        else:
            log(f"✗ 알림 발송 실패: HTTP {response.status_code} - {response.text}")
            return False

    except requests.exceptions.Timeout:
        log("✗ 타임아웃: 백엔드 서버 응답 지연")
        return False
    except requests.exceptions.ConnectionError:
        log("✗ 연결 오류: 백엔드 서버에 연결할 수 없음")
        return False
    except Exception as e:
        log(f"✗ 예상치 못한 오류: {e}")
        return False

def simulate_robot_journey():
    """로봇 여행 시뮬레이션"""

    log("=" * 50)
    log("🤖 나르고 로봇 판매자 집 도착 시나리오 시작")
    log("=" * 50)

    # 1. 로봇 상태 확인
    log("\n1️⃣ 로봇 상태 확인 중...")
    if check_robot_status():
        log("✓ 로봇 준비 완료")
    else:
        log("⚠️ 로봇 상태 불안정하지만 계속 진행")

    # 2. 이동 시뮬레이션
    log("\n2️⃣ 판매자 집으로 이동 중...")
    locations = [
        "집하장 출발",
        "아파트 단지 입구",
        "105동 앞",
        "105동 1301호 도착!"
    ]

    for i, location in enumerate(locations):
        log(f"   📍 {location}")
        time.sleep(1)  # 이동 시뮬레이션

        # 마지막 위치(판매자 집)에 도착하면 알림 발송
        if i == len(locations) - 1:
            log("\n3️⃣ 판매자 집 도착! 알림 발송...")
            success = notify_seller_arrival(DELIVERY_ID)

            if success:
                log("🎉 시나리오 성공!")
                log("📱 판매자가 '물건을 넣어주세요' 알림을 받았습니다.")
                return True
            else:
                log("💥 시나리오 실패!")
                return False

    return False

def main():
    """메인 함수"""
    try:
        success = simulate_robot_journey()

        log("\n" + "=" * 50)
        if success:
            log("✅ 테스트 완료: 모든 기능이 정상 작동합니다!")
            log("🚀 임베디드 시스템에서 동일한 방식으로 사용하면 됩니다.")
        else:
            log("❌ 테스트 실패: 문제를 해결해주세요.")

        log("=" * 50)

        return 0 if success else 1

    except KeyboardInterrupt:
        log("\n⏹️ 사용자에 의해 중단됨")
        return 1
    except Exception as e:
        log(f"\n🚨 예상치 못한 오류: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())