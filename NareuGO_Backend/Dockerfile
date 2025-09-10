FROM amazoncorretto:17-alpine-jdk

# Gradle 빌드 결과물이 들어오는 경로로 수정
ARG JAR_FILE=build/libs/*.jar
ARG PROFILES
ARG ENV

COPY ${JAR_FILE} app.jar

ENTRYPOINT ["java", "-Dspring.profiles.active=${PROFILES}", "-Dserver.env=${ENV}", "-jar", "app.jar"]
