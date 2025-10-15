package org.example.nareugobackend.util;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Component;
import software.amazon.awssdk.auth.credentials.AwsBasicCredentials;
import software.amazon.awssdk.auth.credentials.StaticCredentialsProvider;
import software.amazon.awssdk.services.s3.presigner.S3Presigner;
import software.amazon.awssdk.services.s3.presigner.model.GetObjectPresignRequest;
import software.amazon.awssdk.services.s3.presigner.model.PutObjectPresignRequest;
import software.amazon.awssdk.services.s3.model.GetObjectRequest;
import software.amazon.awssdk.services.s3.model.PutObjectRequest;

import java.time.Duration;

/**
 * S3 URL 생성 유틸리티 클래스
 * - Presigned Upload URL 생성
 * - Presigned Download URL 생성
 * - 직접 S3 URL 생성
 */
@Component
public class S3UrlGenerator {

    @Value("${cloud.aws.s3.bucket}")
    private String bucketName;
    
    @Value("${cloud.aws.s3.region}")
    private String region;
    
    @Value("${cloud.aws.credentials.access-key}")
    private String accessKey;
    
    @Value("${cloud.aws.credentials.secret-key}")
    private String secretKey;

    /**
     * AWS S3 Presigned Upload URL 생성
     */
    public String generatePresignedUploadUrl(String s3Key) {
        try {
            S3Presigner presigner = S3Presigner.builder()
                .region(software.amazon.awssdk.regions.Region.of(region))
                .credentialsProvider(StaticCredentialsProvider.create(
                    AwsBasicCredentials.create(accessKey, secretKey)
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
            throw new RuntimeException("Presigned Upload URL 생성 실패: " + e.getMessage(), e);
        }
    }

    /**
     * AWS S3 Presigned Download URL 생성
     */
    public String generatePresignedDownloadUrl(String s3Key) {
        try {
            S3Presigner presigner = S3Presigner.builder()
                .region(software.amazon.awssdk.regions.Region.of(region))
                .credentialsProvider(StaticCredentialsProvider.create(
                    AwsBasicCredentials.create(accessKey, secretKey)
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

    /**
     * 직접 S3 URL 생성 (공개 접근용)
     */
    public String generateDirectUrl(String s3Key) {
        return "https://" + bucketName + ".s3." + region + ".amazonaws.com/" + s3Key;
    }
}
