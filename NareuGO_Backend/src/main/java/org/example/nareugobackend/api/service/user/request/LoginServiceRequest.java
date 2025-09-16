package org.example.nareugobackend.api.service.user.request;

import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
public class LoginServiceRequest {
    
    private String email;
    
    @Builder
    public LoginServiceRequest(String email) {
        this.email = email;
    }
}
