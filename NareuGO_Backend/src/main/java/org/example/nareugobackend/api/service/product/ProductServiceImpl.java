package org.example.nareugobackend.api.service.product;

import java.util.ArrayList;
import java.util.List;
import org.example.nareugobackend.api.controller.product.response.ProductDetailResponse;
import org.example.nareugobackend.api.service.product.request.ProductServiceRequest;
import org.example.nareugobackend.api.service.product.request.UserInfoRequest;
import software.amazon.awssdk.services.s3.S3Client;
import software.amazon.awssdk.services.s3.model.GetObjectRequest;
import software.amazon.awssdk.services.s3.presigner.S3Presigner;
import software.amazon.awssdk.services.s3.presigner.model.GetObjectPresignRequest;
import software.amazon.awssdk.services.s3.presigner.model.PutObjectPresignRequest;
import software.amazon.awssdk.services.s3.model.PutObjectRequest;
import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.controller.product.request.ProductCreateRequest;
import org.example.nareugobackend.api.controller.product.response.ProductCreateResponse;
import org.example.nareugobackend.mapper.ProductMapper;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.Duration;

@Service
@RequiredArgsConstructor
public class ProductServiceImpl implements ProductService {

    private final ProductMapper productMapper;

    @Autowired
    private S3Client s3Client;
    
    @Value("${cloud.aws.s3.bucket}")
    private String bucketName;
    
    @Value("${cloud.aws.credentials.access-key}")
    private String accessKey;
    
    @Value("${cloud.aws.credentials.secret-key}")
    private String secretKey;
    
    @Value("${cloud.aws.s3.region}")
    private String region;


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
        UserInfoRequest userInfoRequest = productMapper.selectUserInfo(userId);

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
                String uploadUrl = generatePresignedUploadUrl(s3Key);
                uploadUrls[i] = uploadUrl;
            }

            // DB에 S3 KEY들 저장
            productMapper.insertImageKeys(productId, s3Keys);

            // Presigned URL 목록 반환 (프론트엔드 업로드용)
            ProductCreateResponse productServiceResponse = new ProductCreateResponse();
            productServiceResponse.setUrls(uploadUrls);
            return productServiceResponse;
        }

        return new ProductCreateResponse();
    }

    private String generatePresignedUploadUrl(String s3Key) {
        try {
            // S3Presigner를 리전과 함께 생성
            S3Presigner presigner = S3Presigner.builder()
                .region(software.amazon.awssdk.regions.Region.of(region))
                .credentialsProvider(software.amazon.awssdk.auth.credentials.StaticCredentialsProvider.create(
                    software.amazon.awssdk.auth.credentials.AwsBasicCredentials.create(accessKey, secretKey)
                ))
                .build();

            PutObjectRequest putObjectRequest = PutObjectRequest.builder()
                .bucket(bucketName)
                .key(s3Key)
                .build();

            PutObjectPresignRequest presignRequest = PutObjectPresignRequest.builder()
                .signatureDuration(Duration.ofHours(24))
                .putObjectRequest(putObjectRequest)
                .build();

            String presignedUrl = presigner.presignPutObject(presignRequest).url().toString();
            presigner.close();

            return presignedUrl;

        } catch (Exception e) {
            throw new RuntimeException("Presigned URL 생성 실패: " + e.getMessage(), e);
        }
    }

//
//    /**
//     * 상품 삭제
//     *
//     * @parm id
//     * @return ProductDeleteResponse
//     */
//    @Transactional
//    @Override
//    public void deleteProduct(long productId) {
//        // TODO 사용자 검증
//        productMapper.deleteProduct(productId);
//    }


    /**
     * 메인페이지 상품 목록 조회
     *
     */
    @Transactional
    @Override
    public List<ProductDetailResponse> selectProduct(long userId) {

        UserInfoRequest userInfoRequest = productMapper.selectUserInfo(userId);

        List<ProductDetailResponse> result =  productMapper.selectProduct(userInfoRequest);

        for (ProductDetailResponse productDetailResponse : result) {
            // 해당 상품에 있는 이미지들
            List<String> fileImageKEYS = productMapper.selectProductImages(productDetailResponse.getProductId());

            // KEYS 이용해서 S3 PRESIGNED DOWNLOAD URLS 발급
            List<String> downloadUrls = new ArrayList<>();
            for (String s3Key : fileImageKEYS) {
                // 직접 S3 URL 사용
                String directUrl = "https://" + bucketName + ".s3." + region + ".amazonaws.com/" + s3Key;
                downloadUrls.add(directUrl);
            }

            productDetailResponse.setImageUrls(downloadUrls);
        }

        return result;
    }


    /**
     * Presigend download urls
     *
     */
    private String generatePresignedDownloadUrl(String s3Key) {
        try {
            // S3Presigner를 리전과 함께 생성
            S3Presigner presigner = S3Presigner.builder()
                .region(software.amazon.awssdk.regions.Region.of(region))
                .credentialsProvider(software.amazon.awssdk.auth.credentials.StaticCredentialsProvider.create(
                    software.amazon.awssdk.auth.credentials.AwsBasicCredentials.create(accessKey, secretKey)
                ))
                .build();

            GetObjectRequest getObjectRequest = GetObjectRequest.builder()
                .bucket(bucketName)
                .key(s3Key)
                .build();

            GetObjectPresignRequest presignRequest = GetObjectPresignRequest.builder()
                .signatureDuration(Duration.ofHours(24)) // 24시간 후 만료
                .getObjectRequest(getObjectRequest)
                .build();

            String presignedUrl = presigner.presignGetObject(presignRequest).url().toString();
            presigner.close();

            return presignedUrl;

        } catch (Exception e) {
            throw new RuntimeException("Presigned Download URL 생성 실패: " + e.getMessage(), e);
        }
    }


//    /**
//     * 단일 상품 조회
//     * ProductDetailResponse selectOneProduct (long productId);
//     *
//     */
//    @Override
//    public ProductDetailResponse selectOneProduct(long productId) {
//        // 상품 기본 정보 조회
//        ProductDetailResponse product = productMapper.selectOneProduct(productId);
//
//        if (product == null) {
//            return null; // 상품이 존재하지 않음
//        }
//
//        // 해당 상품의 이미지들 조회 (공통 메서드 사용)
//        List<String> imageKeys = productMapper.selectProductImages(productId);
//
//        // S3 KEY들을 Presigned Download URL로 변환
//        List<String> downloadUrls = new ArrayList<>();
//        for (String s3Key : imageKeys) {
//            String downloadUrl = generatePresignedDownloadUrl(s3Key);
//            downloadUrls.add(downloadUrl);
//        }
//
//        // 이미지 URL 설정
//        product.setImageUrls(downloadUrls);
//
//        return product;
//    }

}
