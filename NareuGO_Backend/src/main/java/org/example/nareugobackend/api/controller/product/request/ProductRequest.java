package org.example.nareugobackend.api.controller.product.request;

import java.math.BigDecimal;
import java.time.LocalDateTime;
import lombok.Getter;
import lombok.Setter;
import org.example.nareugobackend.common.model.ProductStatus;

@Getter
@Setter
public class ProductRequest {

    private Long sellerId;         // seller_id
    private String title;          // title
    private String description;    // description
    private BigDecimal price;      // price

    private String apartmentName;  // apartment_name
    private String siDo;           // si_do
    private String siGunGu;        // si_gun_gu
    private String eupMyeonDong;   // eup_myeon_dong

}