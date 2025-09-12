package org.example.nareugobackend.api.service.product;

import org.example.nareugobackend.api.controller.product.request.ProductControllerRequest;
import org.example.nareugobackend.api.controller.product.response.ProductControllerResponse;

public interface ProductService {

    ProductControllerResponse createProduct(ProductControllerRequest productRequest);
}
