package org.example.nareugobackend.api.service.ocr;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.example.nareugobackend.api.service.ocr.dto.OcrRequestDto;
import org.example.nareugobackend.api.service.ocr.dto.OcrResponseDto;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.*;
import org.springframework.stereotype.Service;
import org.springframework.web.client.RestTemplate;

import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

@Slf4j
@Service
@RequiredArgsConstructor
public class OcrServiceImpl implements OcrService {

    private final RestTemplate restTemplate;
    private final ObjectMapper objectMapper;

    @Value("${clova.ocr.api-url}")
    private String clovaOcrApiUrl;

    @Value("${clova.ocr.secret-key}")
    private String clovaOcrSecretKey;

    @Override
    public OcrResponseDto extractAddressFromIdCard(OcrRequestDto requestDto) {
        OcrResponseDto response = new OcrResponseDto();
        
        try {
            // 1. Clova OCR API 호출
            String ocrResult = callClovaOcrApi(requestDto.getImageData(), requestDto.getImageFormat());
            
            // 2. OCR 결과에서 텍스트 추출
            List<String> extractedTexts = parseOcrResult(ocrResult);
            response.setAllExtractedTexts(extractedTexts);
            
            // 3. 주소 정보 추출
            String extractedAddress = extractAddressFromTexts(extractedTexts);
            response.setExtractedAddress(extractedAddress);
            
            // 4. 주소 구성 요소 파싱
            OcrResponseDto.AddressComponents components = parseAddressComponents(extractedAddress);
            response.setAddressComponents(components);
            
            // 5. GPS 주소와 비교
            if (requestDto.getGpsAddress() != null && !requestDto.getGpsAddress().isEmpty()) {
                double matchScore = calculateAddressMatchScore(extractedAddress, requestDto.getGpsAddress());
                response.setMatchScore(matchScore);
                response.setAddressMatched(matchScore > 0.7); // 70% 이상 일치 시 성공
            }
            
            response.setSuccess(true);
            log.info("OCR 처리 성공: {}", extractedAddress);
            
        } catch (Exception e) {
            log.error("OCR 처리 실패", e);
            response.setSuccess(false);
            response.setErrorMessage("신분증 인식에 실패했습니다: " + e.getMessage());
        }
        
        return response;
    }

    /**
     * Clova OCR API 호출
     */
    private String callClovaOcrApi(String imageData, String imageFormat) {
        try {
            // HTTP 헤더 설정
            HttpHeaders headers = new HttpHeaders();
            headers.setContentType(MediaType.APPLICATION_JSON);
            headers.set("X-OCR-SECRET", clovaOcrSecretKey);

            // 요청 바디 생성
            Map<String, Object> requestBody = new HashMap<>();
            requestBody.put("version", "V2");
            requestBody.put("requestId", UUID.randomUUID().toString());
            requestBody.put("timestamp", System.currentTimeMillis());
            
            // 이미지 정보
            Map<String, Object> image = new HashMap<>();
            image.put("format", imageFormat.toLowerCase());
            image.put("data", imageData.replaceFirst("^data:image/[^;]*;base64,", "")); // Base64 prefix 제거
            image.put("name", "id_card");
            
            List<Map<String, Object>> images = new ArrayList<>();
            images.add(image);
            requestBody.put("images", images);

            HttpEntity<Map<String, Object>> entity = new HttpEntity<>(requestBody, headers);

            // API 호출
            ResponseEntity<String> response = restTemplate.postForEntity(clovaOcrApiUrl, entity, String.class);
            
            if (response.getStatusCode() == HttpStatus.OK) {
                return response.getBody();
            } else {
                throw new RuntimeException("Clova OCR API 호출 실패: " + response.getStatusCode());
            }
            
        } catch (Exception e) {
            log.error("Clova OCR API 호출 중 오류 발생", e);
            throw new RuntimeException("OCR API 호출 실패", e);
        }
    }

    /**
     * OCR 결과에서 텍스트 추출
     */
    private List<String> parseOcrResult(String ocrResult) {
        List<String> texts = new ArrayList<>();
        
        try {
            JsonNode rootNode = objectMapper.readTree(ocrResult);
            JsonNode imagesNode = rootNode.get("images");
            
            if (imagesNode != null && imagesNode.isArray()) {
                for (JsonNode imageNode : imagesNode) {
                    JsonNode fieldsNode = imageNode.get("fields");
                    if (fieldsNode != null && fieldsNode.isArray()) {
                        for (JsonNode fieldNode : fieldsNode) {
                            JsonNode inferTextNode = fieldNode.get("inferText");
                            if (inferTextNode != null) {
                                texts.add(inferTextNode.asText());
                            }
                        }
                    }
                }
            }
        } catch (Exception e) {
            log.error("OCR 결과 파싱 실패", e);
            throw new RuntimeException("OCR 결과 파싱 실패", e);
        }
        
        return texts;
    }

    /**
     * 텍스트에서 주소 정보 추출
     */
    private String extractAddressFromTexts(List<String> texts) {
        // 주소 패턴 정규식들
        List<Pattern> addressPatterns = Arrays.asList(
            Pattern.compile(".*[시도]\\s+.*[시군구]\\s+.*[동면읍].*"),
            Pattern.compile(".*특별시.*구.*동.*"),
            Pattern.compile(".*광역시.*구.*동.*"),
            Pattern.compile(".*도\\s+.*시\\s+.*동.*"),
            Pattern.compile(".*도\\s+.*군\\s+.*면.*")
        );
        
        for (String text : texts) {
            for (Pattern pattern : addressPatterns) {
                Matcher matcher = pattern.matcher(text);
                if (matcher.matches()) {
                    return text.trim();
                }
            }
        }
        
        // 패턴 매칭 실패 시, 가장 긴 텍스트를 주소로 간주
        return texts.stream()
                .max(Comparator.comparing(String::length))
                .orElse("");
    }

    /**
     * 주소 구성 요소 파싱
     */
    private OcrResponseDto.AddressComponents parseAddressComponents(String address) {
        OcrResponseDto.AddressComponents components = new OcrResponseDto.AddressComponents();
        
        if (address == null || address.isEmpty()) {
            return components;
        }
        
        // 시/도 추출
        Pattern sidoPattern = Pattern.compile("(.*?[시도]|.*?특별시|.*?광역시)");
        Matcher sidoMatcher = sidoPattern.matcher(address);
        if (sidoMatcher.find()) {
            components.setSido(sidoMatcher.group(1).trim());
        }
        
        // 시/군/구 추출
        Pattern sigunguPattern = Pattern.compile("(.*?[시군구])");
        Matcher sigunguMatcher = sigunguPattern.matcher(address);
        if (sigunguMatcher.find()) {
            components.setSigungu(sigunguMatcher.group(1).trim());
        }
        
        // 동/면/읍 추출
        Pattern dongPattern = Pattern.compile("(.*?[동면읍])");
        Matcher dongMatcher = dongPattern.matcher(address);
        if (dongMatcher.find()) {
            components.setDong(dongMatcher.group(1).trim());
        }
        
        // 우편번호 추출
        Pattern postalPattern = Pattern.compile("(\\d{5})");
        Matcher postalMatcher = postalPattern.matcher(address);
        if (postalMatcher.find()) {
            components.setPostalCode(postalMatcher.group(1));
        }
        
        return components;
    }

    /**
     * GPS 주소와 OCR 주소의 일치도 계산
     */
    private double calculateAddressMatchScore(String ocrAddress, String gpsAddress) {
        if (ocrAddress == null || gpsAddress == null) {
            return 0.0;
        }
        
        // 간단한 문자열 유사도 계산 (Levenshtein distance 기반)
        String[] ocrParts = ocrAddress.split("\\s+");
        String[] gpsParts = gpsAddress.split("\\s+");
        
        int matches = 0;
        int total = Math.max(ocrParts.length, gpsParts.length);
        
        for (String ocrPart : ocrParts) {
            for (String gpsPart : gpsParts) {
                if (ocrPart.contains(gpsPart) || gpsPart.contains(ocrPart)) {
                    matches++;
                    break;
                }
            }
        }
        
        return total > 0 ? (double) matches / total : 0.0;
    }
}
