package org.example.nareugobackend.mapper;
import java.util.List;
import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Param;
import org.example.nareugobackend.api.controller.product.response.ProductDetailResponse;
import org.example.nareugobackend.api.service.product.request.ProductServiceRequest;
import org.example.nareugobackend.api.service.product.request.UserInfoRequest;

@Mapper
public interface ProductMapper {

    int createProduct(ProductServiceRequest productServiceRequest);

    int insertImageKeys(@Param("productId") Long productId,
                         @Param("imageUrls") String[] imageUrls);

//    int deleteProduct(@Param("productId") Long productId);


    // 전체 (읍,면,동 기준)
    List<ProductDetailResponse> selectProduct(UserInfoRequest userInfoRequest);


//    // 개별 단위
//    ProductDetailResponse selectOneProduct(@Param("productId") Long productId);



    // 이미지 조회 (단일/전체 모두 사용)
    List<String> selectProductImages(@Param("productId") Long productId);

    // 사용자 Id 이용하기
    UserInfoRequest selectUserInfo(@Param("userId") Long userId);

}
