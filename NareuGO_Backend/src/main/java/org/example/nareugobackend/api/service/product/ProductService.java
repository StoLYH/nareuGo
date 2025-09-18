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

}
