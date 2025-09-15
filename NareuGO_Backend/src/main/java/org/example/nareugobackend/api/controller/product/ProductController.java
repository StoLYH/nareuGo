package org.example.nareugobackend.api.controller.product;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.controller.product.request.ProductCreateRequest;
import org.example.nareugobackend.api.controller.product.response.ProductDeleteResponse;
import org.example.nareugobackend.api.service.product.ProductService;
import org.example.nareugobackend.api.controller.product.response.ProductCreateResponse;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.DeleteMapping;
import org.springframework.web.bind.annotation.GetMapping;
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
     * @param productRequest
     * @return ProductCreateResponse
     */
    @PostMapping()
    public ResponseEntity<ProductCreateResponse> createProduct(@RequestBody ProductCreateRequest productRequest) {
        return ResponseEntity.ok(productService.createProduct(productRequest));
    }


    /**
     * 상품 삭제
     *
     * @parm id
     * @return ProductDeleteResponse
     */
    @DeleteMapping("/{id}")
    public ResponseEntity<ProductDeleteResponse> deleteProduct(@PathVariable Long id) {
        return ResponseEntity.ok(new ProductDeleteResponse(true, "상품삭제 성공"));
    }


    /**
     * 아파트 별 상품 전체 조회 (서울특별시 강남구 역삼동 래미안아파트)
     * 하드코딩
     *
     * @return
     */
    @GetMapping()
    public ResponseEntity<?> selectProduct() {


        return null;
    }


    /**
     * 상품 개별 조회 (서울특별시 강남구 역삼동 래미안아파트)
     *
     *
     * @return
     */
    @GetMapping()
    public ResponseEntity<?> selectOneProduct() {

        return null;
    }


    /**
     * 상품 검색 (엘라스틱 서치용)
     *
     * @return
     */


}
