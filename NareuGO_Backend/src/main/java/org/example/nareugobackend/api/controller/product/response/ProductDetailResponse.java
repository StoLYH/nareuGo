package org.example.nareugobackend.api.controller.product.response;

import lombok.Builder;
import lombok.Getter;
import lombok.Setter;

import java.math.BigDecimal;
import java.time.LocalDateTime;
import java.util.List;

@Getter
@Setter
@Builder
public class ProductDetailResponse {
    
    // Product 테이블블
    private Long productId;
    private Long sellerId;
    private String title;
    private String description;
    private BigDecimal price;
    private String status;
    private LocalDateTime createdAt;
    private LocalDateTime updatedAt;
    private String apartmentName;
    private String siDo;
    private String siGunGu;
    private String eupMyeonDong;
    
    // productImage 테이블 (productId이용)
    private List<String> imageUrls;

    // 사용자 동,호 (users)
    private String buildingDong;
    private String buildingHo;

}
