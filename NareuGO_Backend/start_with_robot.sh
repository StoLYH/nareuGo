#!/bin/bash
"""
Spring Boot를 로봇 환경변수와 함께 시작하는 스크립트
"""

echo "=== NareuGO Backend with Robot URL Setup ==="

# 1. 임베디드에서 로봇 URL 생성
echo "Generating robot URL from embedded system..."
cd /home/hb/Git/Embedded
python3 generate_robot_url.py

# 결과 확인
if [ $? -eq 0 ]; then
    echo "✅ Robot URL generation completed successfully!"
else
    echo "❌ Robot URL generation failed! Using default localhost:8888"
fi

# 2. 로컬 IP 가져와서 환경변수 설정
echo "Setting up environment variables..."
LOCAL_IP=$(hostname -I | awk '{print $1}')
export ROBOT_HTTP_URL="http://${LOCAL_IP}:8888"

echo "Environment Variables:"
echo "  ROBOT_HTTP_URL = ${ROBOT_HTTP_URL}"

# 3. Spring Boot 백엔드 실행
echo "Starting Spring Boot application..."
cd /home/hb/git/NareuGO_Backend

# Gradle로 실행 (환경변수가 자동으로 전달됨)
./gradlew bootRun

# 또는 JAR 파일로 실행하려면:
# java -jar -DROBOT_HTTP_URL="${ROBOT_HTTP_URL}" build/libs/your-app.jar