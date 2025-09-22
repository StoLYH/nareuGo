package org.example.nareugobackend.api.controller.product;

import java.util.List;
import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.controller.product.request.ProductCreateRequest;
import org.example.nareugobackend.api.controller.product.response.ProductDetailResponse;
import org.example.nareugobackend.api.service.product.ProductService;
import org.example.nareugobackend.api.controller.product.response.ProductCreateResponse;
import org.springframework.http.ResponseEntity;
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
     * 아파트 별 상품 전체 조회
     *
     * @return List<ProductDetailResponse>
     */
    @GetMapping("/{userId}")
    public ResponseEntity<List<ProductDetailResponse>> selectProduct(@PathVariable Long userId) {
        return ResponseEntity.ok(productService.selectProduct(userId));
    }

    /**
     * 상품 개별 조회
     *
     * @Param productId
     * @return ProductDetailResponse
     */
    @GetMapping("/item/{productId}")
    public ResponseEntity<ProductDetailResponse> selectOneProduct(@PathVariable Long productId) {
        ProductDetailResponse product = productService.selectOneProduct(productId);

        if (product == null) {
            return ResponseEntity.notFound().build();
        }
        return ResponseEntity.ok(product);
    }

    // ===== 결제용 API (기존 상품 코드와 분리) =====
    /**
     * 결제용 상품 단일 조회
     *
     * @param productId 상품 ID
     * @return ProductDetailResponse
     */
    @GetMapping("/payment/{productId}")
    public ResponseEntity<ProductDetailResponse> getProductForPayment(@PathVariable Long productId) {
        ProductDetailResponse product = productService.getProductForPayment(productId);

        if (product == null) {
            return ResponseEntity.notFound().build();
        }
        return ResponseEntity.ok(product);
    }
}
