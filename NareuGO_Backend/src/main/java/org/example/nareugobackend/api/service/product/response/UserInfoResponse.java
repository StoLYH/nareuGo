package org.example.nareugobackend.api.service.product.response;


import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class UserInfoResponse {

    @Override
    public String toString() {
        return "UserInfoRequest{" +
            "siDo='" + siDo + '\'' +
            ", siGunGu='" + siGunGu + '\'' +
            ", eupMyeonDong='" + eupMyeonDong + '\'' +
            ", apartmentName='" + apartmentName + '\'' +
            '}';
    }

    private String siDo;           // si_do
    private String siGunGu;        // si_gun_gu
    private String eupMyeonDong;   // eup_myeon_dong
    private String apartmentName;  // apartment_name
}
