#!/usr/bin/env python3
"""
간단한 로봇 API 인터페이스
임베디드 시스템에서 쉽게 사용할 수 있는 단순화된 함수들
"""

import requests
import logging

logger = logging.getLogger(__name__)

# 백엔드 서버 설정
BACKEND_URL = "http://localhost:8080"
REQUEST_TIMEOUT = 10

def send_seller_arrival_notification(delivery_id, backend_url=BACKEND_URL):
    """
    판매자 집 도착 알림 발송 (단순 버전)

    Args:
        delivery_id (int): 배송 ID
        backend_url (str): 백엔드 서버 URL

    Returns:
        bool: 성공 여부
    """
    try:
        url = f"{backend_url}/robot/seller-arrival"
        params = {"deliveryId": delivery_id}

        response = requests.post(url, params=params, timeout=REQUEST_TIMEOUT)

        if response.status_code == 200:
            print(f"✅ 알림 발송 성공 - 배송 ID: {delivery_id}")
            return True
        else:
            print(f"❌ 알림 발송 실패 - 상태: {response.status_code}, 내용: {response.text}")
            return False

    except requests.exceptions.Timeout:
        print(f"⏰ 타임아웃 오류 - 배송 ID: {delivery_id}")
        return False
    except requests.exceptions.ConnectionError:
        print(f"🔌 연결 오류 - 백엔드 서버에 연결할 수 없습니다")
        return False
    except Exception as e:
        print(f"🚨 오류 발생: {e}")
        return False


def check_robot_status(robot_id="NAREUGO_001", backend_url=BACKEND_URL):
    """
    로봇 상태 확인

    Args:
        robot_id (str): 로봇 ID
        backend_url (str): 백엔드 서버 URL

    Returns:
        dict or None: 로봇 상태 정보
    """
    try:
        url = f"{backend_url}/robot/status"
        params = {"robotId": robot_id}

        response = requests.get(url, params=params, timeout=REQUEST_TIMEOUT)

        if response.status_code == 200:
            status = response.json()
            print(f"🤖 로봇 상태: {status.get('status', 'Unknown')}")
            return status
        else:
            print(f"❌ 상태 조회 실패: {response.status_code}")
            return None

    except Exception as e:
        print(f"🚨 상태 조회 오류: {e}")
        return None


# 임베디드 시스템에서 사용할 메인 함수들
def robot_arrived_at_seller(delivery_id):
    """
    로봇이 판매자 집에 도착했을 때 호출하는 함수

    Args:
        delivery_id (int): 배송 ID

    Usage:
        # 로봇이 판매자 집에 도착했을 때
        robot_arrived_at_seller(1)
    """
    print(f"🏠 로봇이 판매자 집에 도착 - 배송 ID: {delivery_id}")

    # 알림 발송
    success = send_seller_arrival_notification(delivery_id)

    if success:
        print("📨 판매자에게 '물건을 넣어주세요' 알림 발송 완료")
    else:
        print("💥 알림 발송 실패")

    return success


def robot_pickup_completed(delivery_id, backend_url=BACKEND_URL):
    """
    로봇이 물건을 픽업했을 때 호출하는 함수 (참고용)

    Args:
        delivery_id (int): 배송 ID
        backend_url (str): 백엔드 서버 URL
    """
    try:
        url = f"{backend_url}/robot/delivery/{delivery_id}/pickup"
        response = requests.post(url, timeout=REQUEST_TIMEOUT)

        if response.status_code == 200:
            print(f"✅ 픽업 완료 처리 성공 - 배송 ID: {delivery_id}")
            return True
        else:
            print(f"❌ 픽업 완료 처리 실패: {response.status_code}")
            return False

    except Exception as e:
        print(f"🚨 픽업 완료 처리 오류: {e}")
        return False


def robot_delivery_completed(delivery_id, backend_url=BACKEND_URL):
    """
    로봇이 배송을 완료했을 때 호출하는 함수 (참고용)

    Args:
        delivery_id (int): 배송 ID
        backend_url (str): 백엔드 서버 URL
    """
    try:
        url = f"{backend_url}/robot/delivery/{delivery_id}/complete"
        response = requests.post(url, timeout=REQUEST_TIMEOUT)

        if response.status_code == 200:
            print(f"✅ 배송 완료 처리 성공 - 배송 ID: {delivery_id}")
            return True
        else:
            print(f"❌ 배송 완료 처리 실패: {response.status_code}")
            return False

    except Exception as e:
        print(f"🚨 배송 완료 처리 오류: {e}")
        return False


# 사용 예시
if __name__ == "__main__":
    print("🤖 나르고 로봇 API 테스트")
    print("=" * 40)

    # 1. 로봇 상태 확인
    print("\n1️⃣ 로봇 상태 확인")
    check_robot_status()

    # 2. 판매자 집 도착 시뮬레이션
    print("\n2️⃣ 판매자 집 도착 알림")
    robot_arrived_at_seller(1)

    print("\n🎉 테스트 완료!")