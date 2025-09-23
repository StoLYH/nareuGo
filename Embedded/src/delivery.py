#!/usr/bin/env python3
import sys
import requests
import json
import time

def start_delivery(delivery_id, seller_address, buyer_address):
    """배달을 시작하는 함수"""
    print(f"{seller_address}에서 {buyer_address}로 배달합니다.")
    time.sleep(3)
    # 배달 완료 후 API 호출
    complete_delivery(delivery_id)

def complete_delivery(delivery_id):
    """배달 완료 API 호출"""
    try:
        url = f"http://localhost:8080/robot/delivery/{delivery_id}/complete"
        response = requests.post(url)
        
        if response.status_code == 200:
            result = response.json()
            print(f"배송 완료 API 호출 성공: {result}")
        else:
            print(f"배송 완료 API 호출 실패: HTTP {response.status_code}")
    except Exception as e:
        print(f"배송 완료 API 호출 중 오류: {e}")

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("사용법: python delivery.py <배송ID> <판매자주소> <구매자주소>")
        sys.exit(1)
    
    delivery_id = sys.argv[1]
    seller_address = sys.argv[2]
    buyer_address = sys.argv[3]
    
    start_delivery(delivery_id, seller_address, buyer_address)