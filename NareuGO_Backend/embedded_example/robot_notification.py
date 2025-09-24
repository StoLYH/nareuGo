#!/usr/bin/env python3
"""
나르고 로봇 알림 시스템
로봇이 판매자 집에 도착했을 때 백엔드로 알림을 보내는 모듈
"""

import requests
import logging
import time
import json
from typing import Optional

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class RobotNotificationClient:
    """로봇 알림 클라이언트 클래스"""

    def __init__(self, backend_url: str = "http://localhost:8080"):
        """
        초기화

        Args:
            backend_url: 백엔드 서버 URL
        """
        self.backend_url = backend_url
        self.session = requests.Session()
        self.session.timeout = 10  # 10초 타임아웃

    def notify_seller_arrival(self, delivery_id: int, retry_count: int = 3) -> bool:
        """
        판매자 집 도착 알림 발송

        Args:
            delivery_id: 배송 ID
            retry_count: 재시도 횟수

        Returns:
            bool: 성공 여부
        """
        url = f"{self.backend_url}/robot/seller-arrival"
        params = {"deliveryId": delivery_id}

        for attempt in range(retry_count):
            try:
                logger.info(f"판매자 집 도착 알림 발송 시도 {attempt + 1}/{retry_count} - 배송 ID: {delivery_id}")

                response = self.session.post(url, params=params)

                if response.status_code == 200:
                    logger.info(f"✅ 판매자 집 도착 알림 발송 성공 - 배송 ID: {delivery_id}")
                    logger.info(f"응답: {response.text}")
                    return True
                else:
                    logger.error(f"❌ 알림 발송 실패 - 상태코드: {response.status_code}, 응답: {response.text}")

            except requests.exceptions.ConnectTimeout:
                logger.error(f"⏰ 연결 타임아웃 - 시도 {attempt + 1}/{retry_count}")
            except requests.exceptions.ConnectionError:
                logger.error(f"🔌 연결 오류 - 시도 {attempt + 1}/{retry_count}")
            except Exception as e:
                logger.error(f"🚨 예상치 못한 오류 - 시도 {attempt + 1}/{retry_count}: {e}")

            # 재시도 전 대기 (지수 백오프)
            if attempt < retry_count - 1:
                wait_time = 2 ** attempt  # 1초, 2초, 4초...
                logger.info(f"⏳ {wait_time}초 후 재시도...")
                time.sleep(wait_time)

        logger.error(f"💥 모든 시도 실패 - 배송 ID: {delivery_id}")
        return False

    def get_robot_status(self, robot_id: str) -> Optional[dict]:
        """
        로봇 상태 확인 (참고용)

        Args:
            robot_id: 로봇 ID

        Returns:
            dict: 로봇 상태 정보 또는 None
        """
        try:
            url = f"{self.backend_url}/robot/status"
            params = {"robotId": robot_id}

            response = self.session.get(url, params=params)

            if response.status_code == 200:
                return response.json()
            else:
                logger.error(f"로봇 상태 조회 실패: {response.status_code}")
                return None

        except Exception as e:
            logger.error(f"로봇 상태 조회 오류: {e}")
            return None


def simulate_robot_arrival_scenario():
    """로봇 도착 시나리오 시뮬레이션"""

    # 알림 클라이언트 초기화
    client = RobotNotificationClient("http://localhost:8080")

    # 시나리오: 배송 ID 1번 주문의 판매자 집에 도착
    delivery_id = 1
    robot_id = "NAREUGO_001"

    logger.info("🤖 === 나르고 로봇 판매자 집 도착 시나리오 시작 ===")

    # 1. 로봇 상태 확인 (선택사항)
    logger.info("1️⃣ 로봇 상태 확인...")
    robot_status = client.get_robot_status(robot_id)
    if robot_status:
        logger.info(f"로봇 상태: {robot_status}")

    # 2. 판매자 집 도착 알림
    logger.info("2️⃣ 판매자 집 도착 알림 발송...")
    success = client.notify_seller_arrival(delivery_id)

    if success:
        logger.info("✅ 시나리오 완료: 판매자에게 알림이 성공적으로 발송되었습니다!")
        logger.info("📱 판매자는 이제 '나르고가 도착했습니다! 물건을 넣어주세요' 알림을 받게 됩니다.")
    else:
        logger.error("❌ 시나리오 실패: 알림 발송에 실패했습니다.")

    logger.info("🤖 === 시나리오 종료 ===")


class RobotLocationTracker:
    """로봇 위치 추적 및 도착 감지 클래스"""

    def __init__(self, notification_client: RobotNotificationClient):
        self.notification_client = notification_client
        self.current_delivery_id = None
        self.seller_address = None
        self.target_reached = False

    def start_delivery(self, delivery_id: int, seller_address: str):
        """
        배송 시작

        Args:
            delivery_id: 배송 ID
            seller_address: 판매자 주소
        """
        self.current_delivery_id = delivery_id
        self.seller_address = seller_address
        self.target_reached = False

        logger.info(f"🚀 배송 시작 - ID: {delivery_id}, 목적지: {seller_address}")

    def check_arrival_at_seller(self, current_location: str) -> bool:
        """
        판매자 집 도착 여부 확인

        Args:
            current_location: 현재 위치

        Returns:
            bool: 도착 여부
        """
        # 실제 구현에서는 GPS 좌표나 주소 매칭 로직 사용
        # 여기서는 간단한 문자열 매칭으로 시뮬레이션

        if not self.seller_address or not self.current_delivery_id:
            return False

        # 판매자 주소와 현재 위치가 매칭되는지 확인
        if self.seller_address in current_location and not self.target_reached:
            logger.info(f"🎯 판매자 집 도착 감지: {current_location}")

            # 알림 발송
            success = self.notification_client.notify_seller_arrival(self.current_delivery_id)

            if success:
                self.target_reached = True
                logger.info("📨 판매자에게 '물건을 넣어주세요' 알림 발송 완료!")
                return True
            else:
                logger.error("📨 알림 발송 실패")

        return False


def advanced_robot_scenario():
    """고급 로봇 시나리오 - 위치 추적 포함"""

    logger.info("🚀 === 고급 로봇 시나리오 시작 ===")

    # 클라이언트 및 트래커 초기화
    client = RobotNotificationClient()
    tracker = RobotLocationTracker(client)

    # 배송 시작
    delivery_id = 1
    seller_address = "105동 1301호"
    tracker.start_delivery(delivery_id, seller_address)

    # 로봇 이동 시뮬레이션
    locations = [
        "출발지",
        "중간 지점 1",
        "중간 지점 2",
        "105동 근처",
        "105동 1301호"  # 판매자 집 도착
    ]

    for i, location in enumerate(locations):
        logger.info(f"📍 현재 위치: {location}")

        # 판매자 집 도착 체크
        if tracker.check_arrival_at_seller(location):
            logger.info("🎉 판매자 집 도착 처리 완료!")
            break

        # 이동 시뮬레이션 (실제로는 로봇의 이동 제어 코드)
        time.sleep(1)

    logger.info("🚀 === 고급 시나리오 종료 ===")


if __name__ == "__main__":
    """메인 실행부"""

    print("나르고 로봇 알림 시스템")
    print("1. 기본 시나리오 (즉시 알림 발송)")
    print("2. 고급 시나리오 (위치 추적 + 도착 감지)")

    choice = input("선택하세요 (1 또는 2): ").strip()

    if choice == "1":
        simulate_robot_arrival_scenario()
    elif choice == "2":
        advanced_robot_scenario()
    else:
        print("잘못된 선택입니다. 기본 시나리오를 실행합니다.")
        simulate_robot_arrival_scenario()