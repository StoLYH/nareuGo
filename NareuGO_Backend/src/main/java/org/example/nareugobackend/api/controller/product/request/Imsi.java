package org.example.nareugobackend.api.controller.product.request;

import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class Imsi {
    private Long sellerId;          // 판매자 ID
    private String title;           // 상품명
    private String description;     // 상품 설명
    private Long price;             // 가격
    private String apartmentName;   // 아파트 이름
    private String siDo;            // 시/도
    private String siGunGu;         // 시/군/구
    private String eupMyeonDong;    // 읍/면/동
}