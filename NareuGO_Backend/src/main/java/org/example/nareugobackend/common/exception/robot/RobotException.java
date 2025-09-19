package org.example.nareugobackend.common.exception.robot;

import lombok.Getter;
import org.example.nareugobackend.common.exception.NAREUGOException;

@Getter
public class RobotException extends NAREUGOException {

    private final RobotErrorCode robotErrorCode;

    public RobotException(RobotErrorCode robotErrorCode) {
        super(robotErrorCode);
        this.robotErrorCode = robotErrorCode;
    }

    public RobotException(RobotErrorCode robotErrorCode, String message) {
        super(robotErrorCode);
        this.robotErrorCode = robotErrorCode;
    }

    public RobotException(RobotErrorCode robotErrorCode, String message, Throwable cause) {
        super(robotErrorCode);
        this.robotErrorCode = robotErrorCode;
        this.initCause(cause);
    }
}