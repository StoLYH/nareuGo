package org.example.nareugobackend.api.controller.product;

import java.util.List;
import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.controller.product.request.ProductCreateRequest;
import org.example.nareugobackend.api.controller.product.response.ProductDetailResponse;
import org.example.nareugobackend.api.service.product.ProductService;
import org.example.nareugobackend.api.controller.product.response.ProductCreateResponse;
import org.springframework.http.ResponseEntity;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.PageRequest;
import org.springframework.data.domain.Pageable;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.DeleteMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;
import org.example.nareugobackend.search.ProductDocument;
import org.example.nareugobackend.search.ProductSearchService;
import org.example.nareugobackend.api.controller.product.response.ProductDeleteResponse;

@RestController
@RequiredArgsConstructor
@RequestMapping("/products")
public class ProductController {

    private final ProductService productService;
    private final ProductSearchService productSearchService;


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
     * 상품 삭제 (DB + ES 동기화)
     */
    @DeleteMapping("/{productId}")
    public ResponseEntity<ProductDeleteResponse> delete(@PathVariable Long productId) {
        return ResponseEntity.ok(productService.deleteProduct(productId));
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

    /**
     * 상품명 자동완성 제안 API (문자열 리스트)
     */
    @GetMapping("/suggest")
    public ResponseEntity<List<String>> suggest(
        @RequestParam String q,
        @RequestParam(defaultValue = "10") int size
    ) {
        return ResponseEntity.ok(productSearchService.suggest(q, size));
    }

    /**
     * Nori 기반 상품 검색 (제목 + 설명)
     */
    @GetMapping("/search")
    public ResponseEntity<Page<ProductDocument>> search(
        @RequestParam String q,
        @RequestParam(defaultValue = "0") int page,
        @RequestParam(defaultValue = "20") int size
    ) {
        Page<ProductDocument> result = productSearchService.searchNori(q, page, size);
        return ResponseEntity.ok(result);
    }


}
