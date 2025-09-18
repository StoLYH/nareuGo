package org.example.nareugobackend.api.controller.user.response;

import lombok.Builder;
import lombok.Getter;

@Getter
@Builder
public class MyPageResponse {
    private String name;
    private String siGunGu;
    private String eupMyeonDong;
    private String apartName;
    private String buildingDong;
    private String buildingHo;
}