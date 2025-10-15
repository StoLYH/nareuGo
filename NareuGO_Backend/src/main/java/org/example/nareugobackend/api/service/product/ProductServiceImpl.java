package org.example.nareugobackend.api.service.product;

import java.util.ArrayList;
import java.util.List;
import org.example.nareugobackend.api.controller.product.response.ProductDetailResponse;
import org.example.nareugobackend.api.service.product.request.ProductServiceRequest;
import org.example.nareugobackend.api.service.product.response.UserInfoResponse;
import software.amazon.awssdk.services.s3.S3Client;
import software.amazon.awssdk.services.s3.model.GetObjectRequest;
import software.amazon.awssdk.services.s3.presigner.S3Presigner;
import software.amazon.awssdk.services.s3.presigner.model.GetObjectPresignRequest;
import software.amazon.awssdk.services.s3.presigner.model.PutObjectPresignRequest;
import software.amazon.awssdk.services.s3.model.PutObjectRequest;
import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.controller.product.request.ProductCreateRequest;
import org.example.nareugobackend.api.controller.product.response.ProductCreateResponse;
import org.example.nareugobackend.api.controller.product.response.ProductDeleteResponse;
import org.example.nareugobackend.mapper.ProductMapper;
import org.example.nareugobackend.search.ProductSearchService;
import org.example.nareugobackend.util.S3UrlGenerator;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.Duration;

@Service
@RequiredArgsConstructor
public class ProductServiceImpl implements ProductService {

    private final ProductMapper productMapper;
    private final ProductSearchService productSearchService;
    private final S3UrlGenerator s3UrlGenerator;


    /**
     * 상품 등록
     *
     * @param productRequest
     * @return ProductCreateResponse
     */
    @Transactional
    @Override
    public ProductCreateResponse createProduct(ProductCreateRequest productRequest) {

        // 사용자 정보 조회
        Long userId = productRequest.getSellerId();
        UserInfoResponse userInfoRequest = productMapper.selectUserInfo(userId);

        ProductServiceRequest productServiceRequest = new ProductServiceRequest(
            productRequest.getProductId(), productRequest.getSellerId(), productRequest.getTitle(),
            productRequest.getDescription(), productRequest.getPrice(),
            userInfoRequest.getSiDo(), userInfoRequest.getSiGunGu(),
            userInfoRequest.getEupMyeonDong(), userInfoRequest.getApartmentName());


        // 상품 등록
        productMapper.createProduct(productServiceRequest);

        // 상품 Id
        long productId = productServiceRequest.getProductId();

        // S3 이미지 처리
        if (productRequest.getFiles() != null && productRequest.getFiles().length > 0) {
            // FRONT 반환
            String[] uploadUrls = new String[productRequest.getFiles().length];
            // DB 저장
            String[] s3Keys = new String[productRequest.getFiles().length];

            // 키 발급
            for (int i = 0; i < productRequest.getFiles().length; i++) {
                String fileName = productRequest.getFiles()[i];

                // S3 키 생성: product/{productId}/1, product/{productId}/2, ...
                String s3Key = "product/" + productId + "/" + (i + 1);
                s3Keys[i] = s3Key;

                // Presigned URL 생성 (업로드용)
                String uploadUrl = s3UrlGenerator.generatePresignedUploadUrl(s3Key);
                uploadUrls[i] = uploadUrl;
            }

            // DB에 S3 KEY들 저장
            productMapper.insertImageKeys(productId, s3Keys);

            // Elasticsearch 색인 (이미지 키 저장 이후 실행)
            productSearchService.indexProduct(productId);

            // Presigned URL 목록 반환 (프론트엔드 업로드용)
            ProductCreateResponse productServiceResponse = new ProductCreateResponse();
            productServiceResponse.setUrls(uploadUrls);
            return productServiceResponse;
        }

        // 이미지가 없는 경우에도 Elasticsearch 색인 실행
        productSearchService.indexProduct(productId);

        return new ProductCreateResponse();
    }


    /**
     * 아파트 별 상품 목록 조회
     *
     * @param userId
     * @return List<ProductDetailResponse>
     */
    @Transactional
    @Override
    public List<ProductDetailResponse> selectProduct(long userId) {

        UserInfoResponse userInfoRequest = productMapper.selectUserInfo(userId);

        List<ProductDetailResponse> result =  productMapper.selectProduct(userInfoRequest);

        for (ProductDetailResponse productDetailResponse : result) {
            // 해당 상품에 있는 이미지들
            List<String> fileImageKEYS = productMapper.selectProductImages(productDetailResponse.getProductId());

            // KEYS 이용해서 S3 직접 URL 생성
            List<String> downloadUrls = new ArrayList<>();
            for (String s3Key : fileImageKEYS) {
                String directUrl = s3UrlGenerator.generateDirectUrl(s3Key);
                downloadUrls.add(directUrl);
            }

            productDetailResponse.setImageUrls(downloadUrls);
        }

        return result;
    }




    /**
     * 단일 상품 조회
     *
     * @param productId
     * @return ProductDetailResponse
     */
    @Override
    public ProductDetailResponse selectOneProduct(long productId) {
        // 상품 기본 정보 조회
        ProductDetailResponse product = productMapper.selectOneProduct(productId);

        if (product == null) {
            return null; // 상품이 존재하지 않음
        }

        // 해당 상품의 이미지들 조회 (공통 메서드 사용)
        List<String> imageKeys = productMapper.selectProductImages(productId);

        // S3 KEY들을 Presigned Download URL로 변환
        List<String> downloadUrls = new ArrayList<>();
        for (String s3Key : imageKeys) {
            String downloadUrl = s3UrlGenerator.generatePresignedDownloadUrl(s3Key);
            downloadUrls.add(downloadUrl);
        }

        // 이미지 URL 설정
        product.setImageUrls(downloadUrls);

        return product;
    }

    // ===== 결제용 메서드 (기존 상품 코드와 분리) =====
    /**
     * 결제용 상품 단일 조회
     * @param productId 상품 ID
     * @return ProductDetailResponseㄴ
     */
    @Override
    public ProductDetailResponse getProductForPayment(long productId) {
        // 결제용 상품 기본 정보 조회 (새로 추가한 매퍼 메서드 사용)
        ProductDetailResponse product = productMapper.findProductDetailById(productId);
        
        if (product == null) {
            return null; // 상품이 존재하지 않거나 판매중이 아님
        }
        
        // 해당 상품의 이미지들 조회
        List<String> imageKeys = productMapper.selectProductImages(productId);
        
        // S3 KEY들을 직접 URL로 변환 (기존 로직과 동일)
        List<String> downloadUrls = new ArrayList<>();
        for (String s3Key : imageKeys) {
            String directUrl = s3UrlGenerator.generateDirectUrl(s3Key);
            downloadUrls.add(directUrl);
        }
        
        // 이미지 URL 설정
        product.setImageUrls(downloadUrls);
        
        return product;
    }

    @Transactional
    @Override
    public ProductDeleteResponse deleteProduct(long productId) {
        int row = productMapper.deleteProduct(productId);
        if (row > 0) {
            productSearchService.deleteFromIndex(productId);
            return new ProductDeleteResponse(true, "상품이 삭제되었습니다.");
        }
        return new ProductDeleteResponse(false, "상품을 찾을 수 없습니다.");
    }

}
