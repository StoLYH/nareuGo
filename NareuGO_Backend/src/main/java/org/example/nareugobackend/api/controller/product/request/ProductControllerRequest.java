package org.example.nareugobackend.api.controller.product.request;

import java.math.BigDecimal;
import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class ProductControllerRequest {

    private Long sellerId;         // seller_id
    private String title;          // title
    private String description;    // description
    private BigDecimal price;      // price

    private String apartmentName;  // apartment_name
    private String siDo;           // si_do
    private String siGunGu;        // si_gun_gu
    private String eupMyeonDong;   // eup_myeon_dong
}