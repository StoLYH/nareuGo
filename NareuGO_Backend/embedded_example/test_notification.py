#!/usr/bin/env python3
"""
간단한 테스트 스크립트
판매자 집 도착 알림을 즉시 테스트하는 용도
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from robot_notification import RobotNotificationClient
import logging

# 로깅 설정
logging.basicConfig(level=logging.INFO)

def test_seller_arrival_notification():
    """판매자 집 도착 알림 테스트"""

    # 백엔드 URL (필요시 수정)
    backend_url = "http://localhost:8080"

    # 테스트할 배송 ID
    delivery_id = 1

    print(f"🧪 판매자 집 도착 알림 테스트")
    print(f"📍 백엔드 URL: {backend_url}")
    print(f"📦 배송 ID: {delivery_id}")
    print("-" * 50)

    # 알림 클라이언트 생성
    client = RobotNotificationClient(backend_url)

    # 알림 발송
    success = client.notify_seller_arrival(delivery_id)

    if success:
        print("✅ 테스트 성공!")
        print("📱 판매자에게 '나르고가 도착했습니다! 물건을 넣어주세요' 알림이 발송되었습니다.")
    else:
        print("❌ 테스트 실패!")
        print("🔧 백엔드 서버 상태와 배송 데이터를 확인해주세요.")

if __name__ == "__main__":
    test_seller_arrival_notification()