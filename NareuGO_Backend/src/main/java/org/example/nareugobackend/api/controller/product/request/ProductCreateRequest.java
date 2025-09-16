package org.example.nareugobackend.api.controller.product.request;

import java.math.BigDecimal;
import java.util.Arrays;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@AllArgsConstructor
@NoArgsConstructor
@Getter
@Setter
public class ProductCreateRequest {

    private Long productId;        // s3 연동을 위해


    private Long sellerId;         // seller_id (FRONT 로컬호스트에서 가져오기)
    private String title;          // title
    private String description;    // description
    private BigDecimal price;      // price

    private String [] files;       // s3전용 파일
}


