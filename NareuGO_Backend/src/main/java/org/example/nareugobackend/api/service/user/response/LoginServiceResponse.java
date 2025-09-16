package org.example.nareugobackend.api.service.user.response;

import lombok.Builder;
import lombok.Getter;

@Getter
@Builder
public class LoginServiceResponse {
    
    private String message;
    private boolean success;
    private Long userId;
    private String email;
    private String name;
    
    public static LoginServiceResponse success(Long userId, String email, String name) {
        return LoginServiceResponse.builder()
                .message("로그인 성공")
                .success(true)
                .userId(userId)
                .email(email)
                .name(name)
                .build();
    }
    
    public static LoginServiceResponse failure(String message) {
        return LoginServiceResponse.builder()
                .message(message)
                .success(false)
                .build();
    }
}
