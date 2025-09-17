package org.example.nareugobackend.api.service.ocr;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.example.nareugobackend.api.service.ocr.dto.OcrRequestDto;
import org.example.nareugobackend.api.service.ocr.dto.OcrResponseDto;
import org.example.nareugobackend.common.model.UserEntity;
import org.example.nareugobackend.mapper.UserMapper;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.*;
import org.springframework.http.client.SimpleClientHttpRequestFactory;
import org.springframework.stereotype.Service;
import org.springframework.web.client.RestTemplate;

import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

@Slf4j
@Service
@RequiredArgsConstructor
public class OcrServiceImpl implements OcrService {

    private final ObjectMapper objectMapper;
    private final UserMapper userMapper;

    @Value("${clova.ocr.api-url}")
    private String clovaOcrApiUrl;

    @Value("${clova.ocr.secret-key}")
    private String clovaOcrSecretKey;

    @Override
    public OcrResponseDto extractAddressFromIdCard(OcrRequestDto requestDto) {
        OcrResponseDto response = new OcrResponseDto();
        
        try {
            // 환경변수 확인
            if (clovaOcrApiUrl == null || clovaOcrApiUrl.isEmpty()) {
                throw new RuntimeException("CLOVA_OCR_API_URL 환경변수가 설정되지 않았습니다.");
            }
            if (clovaOcrSecretKey == null || clovaOcrSecretKey.isEmpty()) {
                throw new RuntimeException("CLOVA_OCR_SECRET_KEY 환경변수가 설정되지 않았습니다.");
            }
            
            log.info("OCR API URL: {}", clovaOcrApiUrl);
            log.info("OCR Secret Key 설정됨: {}", clovaOcrSecretKey != null && !clovaOcrSecretKey.isEmpty());
            
            // 1. Clova OCR API 호출
            String ocrResult = callClovaOcrApi(requestDto.getImageData(), requestDto.getImageFormat());
            log.info("OCR API 응답: {}", ocrResult);
            
            // 2. OCR 결과에서 텍스트 추출
            List<String> extractedTexts = parseOcrResult(ocrResult);
            log.info("추출된 텍스트들: {}", extractedTexts);
            response.setAllExtractedTexts(extractedTexts);
            
            // 3. 주소 정보 추출
            String extractedAddress = extractAddressFromTexts(extractedTexts);
            log.info("추출된 주소: {}", extractedAddress);
            response.setExtractedAddress(extractedAddress);
            
            // 4. 주소 구성 요소 파싱
            OcrResponseDto.AddressComponents components = parseAddressComponents(extractedAddress);
            response.setAddressComponents(components);
            
            // 5. GPS 주소와 비교 (70% 이상 일치 기준)
            if (requestDto.getGpsAddress() != null && !requestDto.getGpsAddress().isEmpty()) {
                double matchScore = calculateAddressMatchScore(extractedAddress, requestDto.getGpsAddress());
                response.setMatchScore(matchScore);
                response.setAddressMatched(matchScore >= 0.7); // 70% 이상 일치 시 성공
                
                log.info("주소 매칭 결과 - 일치도: {:.2f}%, 매칭 성공: {}", matchScore * 100, matchScore >= 0.7);
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
            // 타임아웃 설정이 있는 RestTemplate 생성
            RestTemplate restTemplate = createRestTemplateWithTimeout();
            
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

            log.info("Clova OCR API 호출 시작: {}", clovaOcrApiUrl);
            
            // 재시도 로직 추가
            int maxRetries = 3;
            for (int attempt = 1; attempt <= maxRetries; attempt++) {
                try {
                    // API 호출
                    ResponseEntity<String> response = restTemplate.postForEntity(clovaOcrApiUrl, entity, String.class);
                    
                    if (response.getStatusCode() == HttpStatus.OK) {
                        log.info("Clova OCR API 호출 성공 (시도 {}회)", attempt);
                        return response.getBody();
                    } else {
                        throw new RuntimeException("Clova OCR API 호출 실패: " + response.getStatusCode());
                    }
                } catch (Exception e) {
                    log.warn("Clova OCR API 호출 실패 (시도 {}/{}): {}", attempt, maxRetries, e.getMessage());
                    
                    if (attempt == maxRetries) {
                        throw e; // 마지막 시도에서 실패하면 예외 던지기
                    }
                    
                    // 재시도 전 대기 (1초씩 증가)
                    try {
                        Thread.sleep(attempt * 1000);
                    } catch (InterruptedException ie) {
                        Thread.currentThread().interrupt();
                        throw new RuntimeException("재시도 중 인터럽트 발생", ie);
                    }
                }
            }
            
            throw new RuntimeException("모든 재시도 실패");
            
        } catch (Exception e) {
            log.error("Clova OCR API 호출 중 오류 발생", e);
            throw new RuntimeException("OCR API 호출 실패", e);
        }
    }
    
    /**
     * 사용자 인증 상태 업데이트
     */
    private void updateUserVerificationStatus(Long userId, String extractedAddress, 
                                            String gpsAddress, Double latitude, 
                                            Double longitude, boolean addressMatched, 
                                            double matchScore) {
        try {
            log.info("사용자 인증 정보 업데이트 시작 - 사용자 ID: {}", userId);
            
            // 사용자 조회
            UserEntity user = userMapper.findById(userId);
            if (user == null) {
                log.warn("사용자를 찾을 수 없음 - ID: {}", userId);
                return;
            }
            
            // 인증 정보 업데이트 (단순화된 로직)
            user.updateAddressVerification(extractedAddress, gpsAddress, addressMatched);
            
            // DB 업데이트
            userMapper.updateNeighborhoodVerification(user);
            
            log.info("사용자 인증 정보 업데이트 완료 - 인증 상태: {}", user.isAddressVerified());
            
        } catch (Exception e) {
            log.error("사용자 인증 정보 업데이트 실패", e);
            // 인증 정보 업데이트 실패해도 OCR 결과는 반환
        }
    }

    /**
     * 타임아웃 설정이 있는 RestTemplate 생성
     */
    private RestTemplate createRestTemplateWithTimeout() {
        SimpleClientHttpRequestFactory factory = new SimpleClientHttpRequestFactory();
        
        // 연결 타임아웃: 30초
        factory.setConnectTimeout(30000);
        // 읽기 타임아웃: 60초
        factory.setReadTimeout(60000);
        
        RestTemplate restTemplate = new RestTemplate(factory);
        
        return restTemplate;
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
        
        // 패턴 매칭 실패 시, 모든 텍스트를 합쳐서 반환
        log.warn("주소 패턴 매칭 실패. 모든 텍스트: {}", texts);
        return String.join(" ", texts);
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
     * GPS 주소와 OCR 주소의 일치도 계산 (시도/시군구/읍면동 기반)
     */
    private double calculateAddressMatchScore(String ocrAddress, String gpsAddress) {
        if (ocrAddress == null || gpsAddress == null) {
            return 0.0;
        }
        
        log.info("주소 비교 시작 - OCR: {}, GPS: {}", ocrAddress, gpsAddress);
        
        try {
            // OCR 주소에서 주요 구성 요소 추출
            AddressComponents ocrComponents = extractAddressComponents(ocrAddress);
            AddressComponents gpsComponents = extractAddressComponents(gpsAddress);
            
            log.info("OCR 주소 구성요소 - 시도: {}, 시군구: {}, 읍면동: {}", 
                    ocrComponents.getSiDo(), ocrComponents.getSiGunGu(), ocrComponents.getEupMyeonDong());
            log.info("GPS 주소 구성요소 - 시도: {}, 시군구: {}, 읍면동: {}", 
                    gpsComponents.getSiDo(), gpsComponents.getSiGunGu(), gpsComponents.getEupMyeonDong());
            
            // 각 구성 요소별 일치도 계산
            double sidoMatch = compareAddressPart(ocrComponents.getSiDo(), gpsComponents.getSiDo());
            double sigunguMatch = compareAddressPart(ocrComponents.getSiGunGu(), gpsComponents.getSiGunGu());
            double eupMyeonDongMatch = compareAddressPart(ocrComponents.getEupMyeonDong(), gpsComponents.getEupMyeonDong());
            
            // 가중 평균 계산 (시도: 30%, 시군구: 40%, 읍면동: 30%)
            double totalScore = (sidoMatch * 0.3) + (sigunguMatch * 0.4) + (eupMyeonDongMatch * 0.3);
            
            log.info("주소 일치도 계산 결과 - 시도: {:.2f}, 시군구: {:.2f}, 읍면동: {:.2f}, 총점: {:.2f}", 
                    sidoMatch, sigunguMatch, eupMyeonDongMatch, totalScore);
            
            return totalScore;
            
        } catch (Exception e) {
            log.warn("주소 비교 중 오류 발생: {}", e.getMessage());
            return 0.0;
        }
    }
    
    /**
     * 주소에서 시도/시군구/읍면동 구성 요소 추출
     */
    private AddressComponents extractAddressComponents(String address) {
        AddressComponents components = new AddressComponents();
        
        if (address == null || address.trim().isEmpty()) {
            return components;
        }
        
        String cleanAddress = address.trim();
        
        // 1. 시/도 추출
        java.util.regex.Pattern sidoPattern = java.util.regex.Pattern.compile("(서울특별시|부산광역시|대구광역시|인천광역시|광주광역시|대전광역시|울산광역시|세종특별자치시|경기도|강원도|충청북도|충청남도|전라북도|전라남도|경상북도|경상남도|제주특별자치도)");
        java.util.regex.Matcher sidoMatcher = sidoPattern.matcher(cleanAddress);
        if (sidoMatcher.find()) {
            components.setSiDo(sidoMatcher.group(1));
        }
        
        // 2. 시/군/구 추출
        java.util.regex.Pattern sigunguPattern = java.util.regex.Pattern.compile("([가-힣]+[시군구])");
        java.util.regex.Matcher sigunguMatcher = sigunguPattern.matcher(cleanAddress);
        while (sigunguMatcher.find()) {
            String match = sigunguMatcher.group(1);
            if (!match.contains("특별시") && !match.contains("광역시")) {
                if (components.getSiGunGu() == null) {
                    components.setSiGunGu(match);
                } else if (match.endsWith("구")) {
                    // 구가 나오면 읍면동으로 설정 (예: 분당구)
                    components.setEupMyeonDong(match);
                }
            }
        }
        
        // 3. 동/면/읍 추출 (괄호 안의 동 정보 우선)
        java.util.regex.Pattern dongPattern = java.util.regex.Pattern.compile("\\(([가-힣]+동)\\)");
        java.util.regex.Matcher dongMatcher = dongPattern.matcher(cleanAddress);
        if (dongMatcher.find()) {
            components.setEupMyeonDong(dongMatcher.group(1));
        } else {
            // 괄호가 없으면 일반적인 동/면/읍 패턴 찾기
            java.util.regex.Pattern generalDongPattern = java.util.regex.Pattern.compile("([가-힣]+[동면읍])");
            java.util.regex.Matcher generalDongMatcher = generalDongPattern.matcher(cleanAddress);
            if (generalDongMatcher.find() && components.getEupMyeonDong() == null) {
                components.setEupMyeonDong(generalDongMatcher.group(1));
            }
        }
        
        return components;
    }
    
    /**
     * 주소 구성 요소 간 유사도 비교
     */
    private double compareAddressPart(String part1, String part2) {
        if (part1 == null && part2 == null) {
            return 1.0; // 둘 다 null이면 일치
        }
        if (part1 == null || part2 == null) {
            return 0.0; // 하나만 null이면 불일치
        }
        
        String clean1 = part1.trim();
        String clean2 = part2.trim();
        
        // 완전 일치
        if (clean1.equals(clean2)) {
            return 1.0;
        }
        
        // 포함 관계 확인
        if (clean1.contains(clean2) || clean2.contains(clean1)) {
            return 0.8;
        }
        
        // 유사도 계산 (Levenshtein distance 기반)
        int maxLength = Math.max(clean1.length(), clean2.length());
        if (maxLength == 0) {
            return 1.0;
        }
        
        int distance = levenshteinDistance(clean1, clean2);
        return 1.0 - ((double) distance / maxLength);
    }
    
    /**
     * Levenshtein Distance 계산
     */
    private int levenshteinDistance(String s1, String s2) {
        int[][] dp = new int[s1.length() + 1][s2.length() + 1];
        
        for (int i = 0; i <= s1.length(); i++) {
            dp[i][0] = i;
        }
        for (int j = 0; j <= s2.length(); j++) {
            dp[0][j] = j;
        }
        
        for (int i = 1; i <= s1.length(); i++) {
            for (int j = 1; j <= s2.length(); j++) {
                if (s1.charAt(i - 1) == s2.charAt(j - 1)) {
                    dp[i][j] = dp[i - 1][j - 1];
                } else {
                    dp[i][j] = Math.min(Math.min(dp[i - 1][j], dp[i][j - 1]), dp[i - 1][j - 1]) + 1;
                }
            }
        }
        
        return dp[s1.length()][s2.length()];
    }
    
    /**
     * 주소 구성 요소를 담는 내부 클래스
     */
    private static class AddressComponents {
        private String siDo;
        private String siGunGu;
        private String eupMyeonDong;
        
        public String getSiDo() { return siDo; }
        public void setSiDo(String siDo) { this.siDo = siDo; }
        
        public String getSiGunGu() { return siGunGu; }
        public void setSiGunGu(String siGunGu) { this.siGunGu = siGunGu; }
        
        public String getEupMyeonDong() { return eupMyeonDong; }
        public void setEupMyeonDong(String eupMyeonDong) { this.eupMyeonDong = eupMyeonDong; }
    }
}
