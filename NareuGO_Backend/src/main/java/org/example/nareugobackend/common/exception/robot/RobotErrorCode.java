package org.example.nareugobackend.common.exception.robot;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.common.exception.ErrorCode;
import org.springframework.http.HttpStatus;

@Getter
@RequiredArgsConstructor
public enum RobotErrorCode implements ErrorCode {

    ROBOT_CONNECTION_FAILED(HttpStatus.SERVICE_UNAVAILABLE, "ROBOT_CONNECTION_FAILED", "로봇과의 연결에 실패했습니다"),
    ROBOT_STATUS_INVALID(HttpStatus.BAD_REQUEST, "ROBOT_STATUS_INVALID", "로봇 상태가 작업 불가능합니다"),
    ROBOT_COMMUNICATION_TIMEOUT(HttpStatus.REQUEST_TIMEOUT, "ROBOT_COMMUNICATION_TIMEOUT", "로봇과의 통신 시간이 초과되었습니다"),
    DELIVERY_NOT_FOUND(HttpStatus.NOT_FOUND, "DELIVERY_NOT_FOUND", "배송 정보를 찾을 수 없습니다"),
    DELIVERY_ADDRESS_INVALID(HttpStatus.BAD_REQUEST, "DELIVERY_ADDRESS_INVALID", "배송 주소 정보가 유효하지 않습니다"),
    ROS_BRIDGE_CONNECTION_FAILED(HttpStatus.SERVICE_UNAVAILABLE, "ROS_BRIDGE_CONNECTION_FAILED", "ROS Bridge 연결에 실패했습니다"),
    ROBOT_COMMAND_FAILED(HttpStatus.INTERNAL_SERVER_ERROR, "ROBOT_COMMAND_FAILED", "로봇 명령 전송에 실패했습니다");

    private final HttpStatus httpStatus;
    private final String code;
    private final String message;
}