package org.example.nareugobackend.api.controller.robot.request;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@AllArgsConstructor
public class PickupConfirmationRequest {

    private String action;
    private String timestamp;
}