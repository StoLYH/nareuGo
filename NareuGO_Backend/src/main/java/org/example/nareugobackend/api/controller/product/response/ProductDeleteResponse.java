package org.example.nareugobackend.api.controller.product.response;

import lombok.AllArgsConstructor;
import lombok.Data;
@Data
@AllArgsConstructor
public class ProductDeleteResponse {

    private boolean success;  // 성공 여부
    private String message;   // 결과 메시지
}