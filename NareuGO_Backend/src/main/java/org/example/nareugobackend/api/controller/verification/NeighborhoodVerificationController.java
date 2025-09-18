package org.example.nareugobackend.api.controller.verification;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.service.verification.NeighborhoodVerificationService;
import org.example.nareugobackend.api.service.verification.request.VerificationSubmitRequest;
import org.example.nareugobackend.api.service.verification.response.VerificationResponse;
import org.example.nareugobackend.api.service.verification.response.VerificationStatusResponse;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/verification")
public class NeighborhoodVerificationController {

    private final NeighborhoodVerificationService verificationService;

    @PostMapping("/submit/{userId}")
    public ResponseEntity<VerificationResponse> submitVerification(
        @PathVariable Long userId,
        @RequestBody VerificationSubmitRequest request) {
        VerificationResponse response = verificationService.submitVerification(userId, request);
        return ResponseEntity.ok(response);
    }

    @GetMapping("/status/{userId}")
    public ResponseEntity<VerificationStatusResponse> getVerificationStatus(@PathVariable Long userId) {
        VerificationStatusResponse response = verificationService.getVerificationStatus(userId);
        return ResponseEntity.ok(response);
    }

    @GetMapping("/history/{userId}")
    public ResponseEntity<List<VerificationResponse>> getUserVerificationHistory(@PathVariable Long userId) {
        List<VerificationResponse> history = verificationService.getUserVerificationHistory(userId);
        return ResponseEntity.ok(history);
    }
}