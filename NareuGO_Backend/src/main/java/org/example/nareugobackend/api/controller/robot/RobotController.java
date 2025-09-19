package org.example.nareugobackend.api.controller.robot;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.controller.robot.request.RobotStatusRequest;
import org.example.nareugobackend.api.controller.robot.response.RobotStatusResponse;
import org.example.nareugobackend.api.service.robot.RobotService;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

@RestController
@RequiredArgsConstructor
@RequestMapping("/robot")
public class RobotController {

    private final RobotService robotService;

    @PostMapping("/status")
    public ResponseEntity<RobotStatusResponse> checkRobotStatus(@RequestBody RobotStatusRequest request) {
        RobotStatusResponse response = robotService.checkRobotStatus(request.getRobotId());
        return ResponseEntity.ok(response);
    }
}