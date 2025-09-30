#!/usr/bin/env python3
"""
로봇 HTTP URL을 동적으로 생성하여 환경변수로 설정하는 스크립트
"""

import socket
import os
import subprocess
import sys

def get_local_ip():
    """현재 시스템의 로컬 IP 주소를 얻기"""
    try:
        # 인터넷에 연결할 수 있는 IP 주소 얻기
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        try:
            # 대안: hostname -I 명령어 사용
            result = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
            return result.stdout.strip().split()[0]
        except Exception:
            # 기본값 반환
            return "localhost"

def set_robot_url_env():
    """로봇 URL을 환경변수로 설정"""
    try:
        # 로컬 IP 주소 얻기
        local_ip = get_local_ip()
        robot_port = 8888  # 로봇 HTTP 서버 포트

        # 로봇 URL 생성
        robot_url = f"http://{local_ip}:{robot_port}"

        print(f"Generated Robot IP: {local_ip}")
        print(f"Generated Robot URL: {robot_url}")

        # 환경변수 설정
        os.environ['ROBOT_HTTP_URL'] = robot_url

        # 환경변수를 파일로도 저장 (.env 파일)
        env_file_path = "/home/hb/git/NareuGO_Backend/.env"
        with open(env_file_path, "w") as f:
            f.write(f"ROBOT_HTTP_URL={robot_url}\n")

        print(f"Environment variable ROBOT_HTTP_URL set to: {robot_url}")
        print(f"Environment file created at: {env_file_path}")

        return robot_url

    except Exception as e:
        print(f"Error setting robot URL: {e}")
        return None

def main():
    """메인 함수"""
    print("=== Robot URL Generator ===")

    # 로봇 URL 생성 및 환경변수 설정
    robot_url = set_robot_url_env()

    if robot_url:
        print("✅ Robot URL generation completed successfully!")
        return 0
    else:
        print("❌ Robot URL generation failed!")
        return 1

if __name__ == "__main__":
    sys.exit(main())