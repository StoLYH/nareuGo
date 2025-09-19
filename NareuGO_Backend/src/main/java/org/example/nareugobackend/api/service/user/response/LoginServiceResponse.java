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
    // 닉네임 추가 (기본 로그인 응답에 포함)
    private String nickname;
    
    public static LoginServiceResponse success(Long userId, String email, String name) {
        return LoginServiceResponse.builder()
                .message("로그인 성공")
                .success(true)
                .userId(userId)
                .email(email)
                .name(name)
                .build();
    }
    
    // 닉네임을 함께 응답하기 위한 팩토리 메서드 (기존 코드와의 호환성을 위해 별도 추가)
    public static LoginServiceResponse successWithNickname(Long userId, String email, String name, String nickname) {
        return LoginServiceResponse.builder()
                .message("로그인 성공")
                .success(true)
                .userId(userId)
                .email(email)
                .name(name)
                .nickname(nickname)
                .build();
    }
    
    public static LoginServiceResponse failure(String message) {
        return LoginServiceResponse.builder()
                .message(message)
                .success(false)
                .build();
    }
}
