package org.example.nareugobackend.api.service.user.response;

import lombok.Builder;
import lombok.Getter;

@Getter
@Builder
public class UserProfileResponse {
    private Long userId;
    private String nickname;
    private String email;
    private String apartmentName;
    private String buildingDong;
    private String buildingHo;
    private String siDo;
    private String siGunGu;
    private String eupMyeonDong;
    private boolean addressVerified;
}