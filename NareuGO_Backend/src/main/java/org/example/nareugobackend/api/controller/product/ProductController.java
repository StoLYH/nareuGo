package org.example.nareugobackend.api.controller.product;

import java.util.Map;
import java.util.TreeMap;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.DeleteMapping;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequiredArgsConstructor
@RequestMapping("/products")
public class ProductController {


    /**
     * 상품 등록
     *
     * @return
     */
    @PostMapping("/")
    public ResponseEntity<?> createProduct() {

        // TODO 사용자 사는 곳

        // TODO 상품 이미지 S3 -> service

        // 상품


        return null;
    }


    /**
     * 상품 삭제
     *
     * @return
     */
    @DeleteMapping("/{id}")
    public ResponseEntity<?> deleteProduct(@PathVariable Long id) {
        // TODO: 상품 삭제 로직
        return ResponseEntity.noContent().build();
    }




    /**
     * 상품 조회 (엘라스틱 서치)
     *
     * @return
     */




}
