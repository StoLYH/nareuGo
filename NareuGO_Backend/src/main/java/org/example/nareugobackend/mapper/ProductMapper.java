package org.example.nareugobackend.mapper;
import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Param;
import org.example.nareugobackend.api.controller.product.request.ProductCreateRequest;

@Mapper
public interface ProductMapper {

    long createProduct(ProductCreateRequest productServiceRequest);

    void insertImageKeys(@Param("productId") Long productId, 
                         @Param("imageUrls") String[] imageUrls);

}
