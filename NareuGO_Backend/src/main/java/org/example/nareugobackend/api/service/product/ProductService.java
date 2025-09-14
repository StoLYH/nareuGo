package org.example.nareugobackend.api.service.product;

import org.example.nareugobackend.api.controller.product.request.ProductCreateRequest;
import org.example.nareugobackend.api.controller.product.response.ProductCreateResponse;

public interface ProductService {

    ProductCreateResponse createProduct(ProductCreateRequest productRequest);

    void deleteProduct(long productId);
}
