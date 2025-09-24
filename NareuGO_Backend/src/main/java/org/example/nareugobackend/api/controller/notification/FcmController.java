package org.example.nareugobackend.api.controller.notification;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.service.notification.FcmService;
import org.example.nareugobackend.api.controller.notification.request.FcmTokenRequest;
import org.example.nareugobackend.domain.notification.FcmToken;
import org.example.nareugobackend.domain.notification.FcmTokenRepository;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequiredArgsConstructor
@RequestMapping("/fcm")
public class FcmController {

    private final FcmService fcmService;
    private final FcmTokenRepository fcmTokenRepository;

    @GetMapping("/tokens")
    public ResponseEntity<List<FcmToken>> getAllTokens() {
        List<FcmToken> tokens = fcmTokenRepository.findAll();
        return ResponseEntity.ok(tokens);
    }

    @PostMapping("/token")
    public ResponseEntity<Void> saveToken(@RequestBody FcmTokenRequest request) {
        fcmService.saveToken(request.getUserId(), request.getToken(), request.getDeviceType());
        return ResponseEntity.ok().build();
    }

    @DeleteMapping("/token")
    public ResponseEntity<Void> removeToken(@RequestBody FcmTokenRequest request) {
        fcmService.removeToken(request.getUserId(), request.getToken());
        return ResponseEntity.ok().build();
    }
}