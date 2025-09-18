package org.example.nareugobackend.api.service.product;

import java.util.List;
import org.example.nareugobackend.api.controller.product.request.ProductCreateRequest;
import org.example.nareugobackend.api.controller.product.response.ProductCreateResponse;
import org.example.nareugobackend.api.controller.product.response.ProductDetailResponse;
import org.example.nareugobackend.api.service.product.request.UserInfoRequest;

public interface ProductService {

    ProductCreateResponse createProduct(ProductCreateRequest productRequest);

    //void deleteProduct(long productId);

    List<ProductDetailResponse> selectProduct (long userId);

    ProductDetailResponse selectOneProduct (long productId);

    // ===== 결제용 메서드 (기존 상품 코드와 분리) =====
    /**
     * 결제용 상품 단일 조회
     * @param productId 상품 ID
     * @return ProductDetailResponse
     */
    ProductDetailResponse getProductForPayment(long productId);

}
