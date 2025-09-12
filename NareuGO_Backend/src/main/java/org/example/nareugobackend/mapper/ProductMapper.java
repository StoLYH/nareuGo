package org.example.nareugobackend.mapper;
import org.apache.ibatis.annotations.Mapper;
import org.example.nareugobackend.api.controller.product.request.ProductControllerRequest;

@Mapper
public interface ProductMapper {

    long createProduct(ProductControllerRequest productServiceRequest);

}
