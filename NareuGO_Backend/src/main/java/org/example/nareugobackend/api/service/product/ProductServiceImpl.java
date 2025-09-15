package org.example.nareugobackend.api.service.product;

import software.amazon.awssdk.services.s3.S3Client;
import software.amazon.awssdk.services.s3.model.GetObjectRequest;
import software.amazon.awssdk.services.s3.presigner.S3Presigner;
import software.amazon.awssdk.services.s3.presigner.model.GetObjectPresignRequest;
import software.amazon.awssdk.services.s3.presigner.model.PutObjectPresignRequest;
import software.amazon.awssdk.services.s3.model.PutObjectRequest;
import software.amazon.awssdk.auth.credentials.AwsBasicCredentials;
import software.amazon.awssdk.auth.credentials.StaticCredentialsProvider;
import software.amazon.awssdk.regions.Region;
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

    @Transactional
    @Override
    public ProductCreateResponse createProduct(ProductCreateRequest productRequest) {
        // TODO 사용자 정보 검증

        // 상품 등록
        productMapper.createProduct(productRequest);

        // 상품 Id
        long productId = productRequest.getProductId();
        
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

    @Transactional
    @Override
    public void deleteProduct(long productId) {
        // TODO 사용자 검증
        
        productMapper.deleteProduct(productId);
    }











}
