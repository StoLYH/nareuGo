package org.example.nareugobackend.mapper;
import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Param;
import org.example.nareugobackend.api.controller.product.request.ProductCreateRequest;

@Mapper
public interface ProductMapper {

    int createProduct(ProductCreateRequest productServiceRequest);

    int insertImageKeys(@Param("productId") Long productId,
                         @Param("imageUrls") String[] imageUrls);

    int deleteProduct(@Param("productId") Long productId);



}
