package org.example.nareugobackend.common.model;

public enum RobotStatus {
    VALID("VALID"),
    INVALID("INVALID");

    private final String value;

    RobotStatus(String value) {
        this.value = value;
    }

    public String getValue() {
        return value;
    }
}