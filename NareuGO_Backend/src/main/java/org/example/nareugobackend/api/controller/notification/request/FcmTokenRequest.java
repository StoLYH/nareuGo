package org.example.nareugobackend.api.controller.notification.request;

import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
public class FcmTokenRequest {
    private Long userId;
    private String token;
    private String deviceType;
}