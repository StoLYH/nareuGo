package org.example.nareugobackend.api.controller.product;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.controller.product.request.ProductControllerRequest;
import org.example.nareugobackend.api.service.product.ProductService;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.DeleteMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequiredArgsConstructor
@RequestMapping("/products")
public class ProductController {

    private final ProductService productService;


    /**
     * 상품 등록
     *
     * @return
     */
    @PostMapping()
    public ResponseEntity<ProductControllerRequest> createProduct(@RequestBody ProductControllerRequest productRequest) {



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
