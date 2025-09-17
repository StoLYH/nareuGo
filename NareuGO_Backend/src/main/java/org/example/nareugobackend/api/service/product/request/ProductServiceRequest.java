package org.example.nareugobackend.api.service.product.request;


import java.math.BigDecimal;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.Setter;


@AllArgsConstructor
@Setter
@Getter
public class ProductServiceRequest {

    private Long productId;        // 등록 후 s3 위함

    private Long sellerId;         // seller_id (FRONT 로컬호스트에서 가져오기)
    private String title;          // title
    private String description;    // description
    private BigDecimal price;      // price

    // user DB 에서 조회
    private String siDo;           // 서울특별시
    private String siGunGu;        // 강남구
    private String eupMyeonDong;   // 역삼동
    private String apartmentName;  // 래미안 아파트

}
