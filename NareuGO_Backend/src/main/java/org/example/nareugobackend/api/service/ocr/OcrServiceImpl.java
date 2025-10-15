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
            if (clovaOcrApiUrl == null || clovaOcrApiUrl.isEmpty()) {
                throw new RuntimeException("CLOVA_OCR_API_URL 환경변수가 설정되지 않았습니다.");
            }
            if (clovaOcrSecretKey == null || clovaOcrSecretKey.isEmpty()) {
                throw new RuntimeException("CLOVA_OCR_SECRET_KEY 환경변수가 설정되지 않았습니다.");
            }
            
            String ocrResult = callClovaOcrApi(requestDto.getImageData(), requestDto.getImageFormat());
            List<String> extractedTexts = parseOcrResult(ocrResult);
            response.setAllExtractedTexts(extractedTexts);
            
            String extractedAddress = extractAddressFromTexts(extractedTexts);
            response.setExtractedAddress(extractedAddress);
            
            OcrResponseDto.AddressComponents components = parseAddressComponents(extractedAddress);
            response.setAddressComponents(components);
            
            if (requestDto.getGpsAddress() != null && !requestDto.getGpsAddress().isEmpty()) {
                double matchScore = calculateAddressMatchScore(extractedAddress, requestDto.getGpsAddress());
                response.setMatchScore(matchScore);
                response.setAddressMatched(matchScore >= 0.7);
            }
            
            response.setSuccess(true);
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
            RestTemplate restTemplate = createRestTemplateWithTimeout();
            
            HttpHeaders headers = new HttpHeaders();
            headers.setContentType(MediaType.APPLICATION_JSON);
            headers.set("X-OCR-SECRET", clovaOcrSecretKey);

            Map<String, Object> requestBody = new HashMap<>();
            requestBody.put("version", "V2");
            requestBody.put("requestId", UUID.randomUUID().toString());
            requestBody.put("timestamp", System.currentTimeMillis());
            
            Map<String, Object> image = new HashMap<>();
            image.put("format", imageFormat.toLowerCase());
            image.put("data", imageData.replaceFirst("^data:image/[^;]*;base64,", ""));
            image.put("name", "id_card");
            
            List<Map<String, Object>> images = new ArrayList<>();
            images.add(image);
            requestBody.put("images", images);

            HttpEntity<Map<String, Object>> entity = new HttpEntity<>(requestBody, headers);
            
            int maxRetries = 3;
            for (int attempt = 1; attempt <= maxRetries; attempt++) {
                try {
                    ResponseEntity<String> response = restTemplate.postForEntity(clovaOcrApiUrl, entity, String.class);
                    
                    if (response.getStatusCode() == HttpStatus.OK) {
                        return response.getBody();
                    } else {
                        throw new RuntimeException("Clova OCR API 호출 실패: " + response.getStatusCode());
                    }
                } catch (Exception e) {
                    if (attempt == maxRetries) {
                        throw e;
                    }
                    
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
            log.error("Clova OCR API 호출 실패", e);
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
            UserEntity user = userMapper.findById(userId);
            if (user == null) {
                return;
            }
            
            user.updateAddressVerification(extractedAddress, addressMatched);
            userMapper.updateNeighborhoodVerification(user);
        } catch (Exception e) {
            log.error("사용자 인증 정보 업데이트 실패", e);
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
        
        // 문서 타입 확인
        boolean isDriverLicense = address.contains("운전면허증") || address.contains("자동차운전");
        boolean isIdCard = address.contains("주민등록증");
        
        // 불필요한 텍스트 제거 및 주소 부분만 추출
        String cleanAddress = extractAddressPart(address, isDriverLicense, isIdCard);
        
        // 시/도 추출
        Pattern sidoPattern = Pattern.compile("([가-힣]+[시도]|[가-힣]+특별시|[가-힣]+광역시)");
        Matcher sidoMatcher = sidoPattern.matcher(cleanAddress);
        if (sidoMatcher.find()) {
            components.setSido(sidoMatcher.group(1).trim());
        }
        
        // 시/군/구 추출
        Pattern sigunguPattern = Pattern.compile("([가-힣]+[시군구])");
        Matcher sigunguMatcher = sigunguPattern.matcher(cleanAddress);
        if (sigunguMatcher.find()) {
            components.setSigungu(sigunguMatcher.group(1).trim());
        }
        
        // 읍/면/동 추출 - 실제 행정구역만 추출
        Pattern dongPattern = Pattern.compile("([가-힣]+[읍면동구])(?![가-힣]*(?:면허증|등록증|경찰청))");
        Matcher dongMatcher = dongPattern.matcher(cleanAddress);
        if (dongMatcher.find()) {
            String dongName = dongMatcher.group(1).trim();
            // "자동차운전", "대구지방경찰청" 등 제외
            if (!dongName.contains("자동차") && !dongName.contains("경찰") && !dongName.contains("지방")) {
                components.setDong(dongName);
            }
        }
        
        // 우편번호 추출
        Pattern postalPattern = Pattern.compile("(\\d{5})");
        Matcher postalMatcher = postalPattern.matcher(address);
        if (postalMatcher.find()) {
            components.setPostalCode(postalMatcher.group(1));
        }
        
        // 아파트명 추출 (괄호 안의 내용)
        Pattern apartmentPattern = Pattern.compile("\\(([^,)]+)\\)");
        Matcher apartmentMatcher = apartmentPattern.matcher(address);
        if (apartmentMatcher.find()) {
            String apartmentInfo = apartmentMatcher.group(1).trim();
            // 동명이 아닌 경우 아파트명으로 설정
            if (!apartmentInfo.matches(".*[동면읍]$")) {
                components.setApartmentName(apartmentInfo);
            }
        }
        
        // 동 번호 추출 (숫자+동 패턴)
        Pattern buildingDongPattern = Pattern.compile("(\\d+)동");
        Matcher buildingDongMatcher = buildingDongPattern.matcher(address);
        if (buildingDongMatcher.find()) {
            try {
                components.setBuildingDong(Integer.parseInt(buildingDongMatcher.group(1)));
            } catch (NumberFormatException e) {
                log.warn("동 번호 파싱 실패: {}", buildingDongMatcher.group(1));
            }
        }
        
        // 호 번호 추출 (숫자+호 패턴)
        Pattern buildingHoPattern = Pattern.compile("(\\d+)호");
        Matcher buildingHoMatcher = buildingHoPattern.matcher(address);
        if (buildingHoMatcher.find()) {
            try {
                components.setBuildingHo(Integer.parseInt(buildingHoMatcher.group(1)));
            } catch (NumberFormatException e) {
                log.warn("호 번호 파싱 실패: {}", buildingHoMatcher.group(1));
            }
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
        
        try {
            AddressComponents ocrComponents = extractAddressComponents(ocrAddress);
            AddressComponents gpsComponents = extractAddressComponents(gpsAddress);
            
            double sidoMatch = compareAddressPart(ocrComponents.getSiDo(), gpsComponents.getSiDo());
            double sigunguMatch = compareAddressPart(ocrComponents.getSiGunGu(), gpsComponents.getSiGunGu());
            double eupMyeonDongMatch = compareAddressPart(ocrComponents.getEupMyeonDong(), gpsComponents.getEupMyeonDong());
            
            double totalScore = (sidoMatch * 0.3) + (sigunguMatch * 0.4) + (eupMyeonDongMatch * 0.3);
            
            return totalScore;
        } catch (Exception e) {
            log.warn("주소 비교 중 오류 발생", e);
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
     * 문서 타입에 따라 주소 부분만 추출
     */
    private String extractAddressPart(String fullText, boolean isDriverLicense, boolean isIdCard) {
        if (fullText == null || fullText.isEmpty()) {
            return "";
        }
        
        String cleanText = fullText;
        
        if (isDriverLicense) {
            // 운전면허증: 주민번호 이후부터 동호수까지 추출
            Pattern driverPattern = Pattern.compile("\\d{6}-\\d{7}\\s+(.+?)\\s+\\d{4}\\.\\d{2}\\.\\d{2}");
            Matcher driverMatcher = driverPattern.matcher(fullText);
            if (driverMatcher.find()) {
                cleanText = driverMatcher.group(1).trim();
            } else {
                // 대안: 시도명부터 동호수까지
                Pattern altPattern = Pattern.compile("([가-힣]+[시도][^가-힣]*[가-힣]+[시군구][^가-힣]*[가-힣]+[읍면동구][^가-힣]*\\d+동\\s*\\d+호)");
                Matcher altMatcher = altPattern.matcher(fullText);
                if (altMatcher.find()) {
                    cleanText = altMatcher.group(1).trim();
                }
            }
        } else if (isIdCard) {
            // 주민등록증: 주민번호 이후부터 괄호 전까지 추출
            Pattern idPattern = Pattern.compile("\\d{6}-\\d{7}\\s+(.+?)\\s+\\(");
            Matcher idMatcher = idPattern.matcher(fullText);
            if (idMatcher.find()) {
                cleanText = idMatcher.group(1).trim();
            } else {
                // 대안: 시도명부터 동호수까지
                Pattern altPattern = Pattern.compile("([가-힣]+[시도][^가-힣]*[가-힣]+[시군구][^가-힣]*[가-힣]+[읍면동구][^가-힣]*\\d+동\\s*\\d+호)");
                Matcher altMatcher = altPattern.matcher(fullText);
                if (altMatcher.find()) {
                    cleanText = altMatcher.group(1).trim();
                }
            }
        }
        
        // 불필요한 텍스트 제거
        cleanText = cleanText.replaceAll("(운전면허증|자동차운전|주민등록증|지방경찰청장)", "");
        cleanText = cleanText.replaceAll("\\s+", " ").trim();
        
        return cleanText;
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
