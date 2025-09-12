package org.example.nareugobackend.api.service.product;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.controller.product.request.ProductControllerRequest;
import org.example.nareugobackend.api.controller.product.response.ProductControllerResponse;
import org.example.nareugobackend.mapper.ProductMapper;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

@Service
@RequiredArgsConstructor
public class ProductServiceImpl implements ProductService {

    private final ProductMapper productMapper;

    @Transactional
    @Override
    public ProductControllerResponse createProduct(ProductControllerRequest productRequest) {
        // TODO 사용자 정보 검증

        // 상품 등록
        productMapper.createProduct(productRequest);

        return null;
    }
}
