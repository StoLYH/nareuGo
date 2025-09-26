package org.example.nareugobackend.api.controller.robot.response;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class BuyerArrivedResponse {
    private String timestamp;
}