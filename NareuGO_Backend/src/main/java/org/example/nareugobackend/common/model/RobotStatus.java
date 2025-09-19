package org.example.nareugobackend.common.model;

public enum RobotStatus {
    VALID("valid"),
    INVALID("invalid");

    private final String value;

    RobotStatus(String value) {
        this.value = value;
    }

    public String getValue() {
        return value;
    }
}