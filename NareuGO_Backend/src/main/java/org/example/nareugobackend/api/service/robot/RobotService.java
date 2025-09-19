package org.example.nareugobackend.api.service.robot;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.controller.robot.response.RobotStatusResponse;
import org.example.nareugobackend.domain.robot.RobotStatus;
import org.example.nareugobackend.mapper.RobotMapper;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class RobotService {

    private final RobotMapper robotMapper;

    public RobotStatusResponse checkRobotStatus(String robotId) {
        String dbStatus = robotMapper.findRobotStatusByName(robotId);

        if (dbStatus == null) {
            return RobotStatusResponse.builder()
                    .status(RobotStatus.INVALID.name().toLowerCase())
                    .message("로봇을 찾을 수 없습니다")
                    .build();
        }

        RobotStatus status = RobotStatus.valueOf(dbStatus);
        String message = getStatusMessage(status);

        return RobotStatusResponse.builder()
                .status(status.name().toLowerCase())
                .message(message)
                .build();
    }

    private String getStatusMessage(RobotStatus status) {
        switch (status) {
            case VALID:
                return "작업 가능";
            case INVALID:
                return "작업 불가능";
            default:
                return "상태 불명";
        }
    }
}