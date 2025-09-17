package org.example.nareugobackend.api.service.neighborhood;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.example.nareugobackend.api.service.neighborhood.dto.NeighborhoodVerificationRequest;
import org.example.nareugobackend.api.service.neighborhood.dto.NeighborhoodVerificationResponse;
import org.example.nareugobackend.common.model.UserEntity;
import org.example.nareugobackend.mapper.UserMapper;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;

@Slf4j
@Service
@RequiredArgsConstructor
public class NeighborhoodServiceImpl implements NeighborhoodService {

    private final UserMapper userMapper;

    @Override
    @Transactional
    public NeighborhoodVerificationResponse saveVerificationInfo(String userEmail, NeighborhoodVerificationRequest request) {
        NeighborhoodVerificationResponse response = new NeighborhoodVerificationResponse();
        
        try {
            // 사용자 조회
            UserEntity user = userMapper.findByEmail(userEmail);
            if (user == null) {
                throw new RuntimeException("사용자를 찾을 수 없습니다: " + userEmail);
            }
            
            // OCR 주소에서 주소 구성 요소 파싱
            AddressComponents addressComponents = parseOcrAddress(request.getOcrAddress());
            
            // 인증 정보 업데이트
            LocalDateTime verificationDate = LocalDateTime.now();
            user.setAddressVerified(true);
            user.setApartmentName(addressComponents.getApartmentName());
            user.setSiDo(addressComponents.getSiDo());
            user.setSiGunGu(addressComponents.getSiGunGu());
            user.setEupMyeonDong(addressComponents.getEupMyeonDong());
            user.setBuildingDong(addressComponents.getBuildingDong());
            user.setBuildingHo(addressComponents.getBuildingHo());
            user.setVerificationDate(verificationDate);
            
            int updateResult = userMapper.updateNeighborhoodVerification(user);
            
            if (updateResult == 0) {
                throw new RuntimeException("동네 인증 정보 업데이트에 실패했습니다.");
            }
            
            response.setSuccess(true);
            response.setMessage("동네 인증이 완료되었습니다.");
            response.setVerified(true);
            response.setVerifiedLocation(request.getGpsAddress());
            response.setVerifiedAddress(request.getOcrAddress());
            response.setVerificationDate(LocalDateTime.now());
            
            log.info("동네 인증 정보 저장 완료 - 사용자: {}, 위치: {}", userEmail, request.getGpsAddress());
            
        } catch (Exception e) {
            log.error("동네 인증 정보 저장 실패", e);
            response.setSuccess(false);
            response.setMessage("동네 인증 정보 저장에 실패했습니다: " + e.getMessage());
        }
        
        return response;
    }
    
    /**
     * OCR로 추출된 주소를 파싱하여 주소 구성 요소를 추출합니다.
     */
    private AddressComponents parseOcrAddress(String ocrAddress) {
        AddressComponents components = new AddressComponents();
        
        if (ocrAddress == null || ocrAddress.trim().isEmpty()) {
            return components;
        }
        
        String address = ocrAddress.trim();
        log.info("OCR 주소 파싱 시작: {}", address);
        
        try {
            // 정규식을 사용한 상세 주소 파싱
            
            // 1. 시/도 추출 (경기도, 서울특별시, 부산광역시 등)
            java.util.regex.Pattern sidoPattern = java.util.regex.Pattern.compile("(서울특별시|부산광역시|대구광역시|인천광역시|광주광역시|대전광역시|울산광역시|세종특별자치시|경기도|강원도|충청북도|충청남도|전라북도|전라남도|경상북도|경상남도|제주특별자치도)");
            java.util.regex.Matcher sidoMatcher = sidoPattern.matcher(address);
            if (sidoMatcher.find()) {
                components.setSiDo(sidoMatcher.group(1));
            }
            
            // 2. 시/군/구 추출 (성남시, 분당구 등)
            java.util.regex.Pattern sigunguPattern = java.util.regex.Pattern.compile("([가-힣]+[시군구])");
            java.util.regex.Matcher sigunguMatcher = sigunguPattern.matcher(address);
            while (sigunguMatcher.find()) {
                String match = sigunguMatcher.group(1);
                // 특별시, 광역시는 제외
                if (!match.contains("특별시") && !match.contains("광역시")) {
                    if (components.getSiGunGu() == null) {
                        components.setSiGunGu(match);
                    } else {
                        // 두 번째로 찾은 것이 구인 경우 (예: 성남시 다음의 분당구)
                        if (match.endsWith("구")) {
                            components.setEupMyeonDong(match);
                        }
                    }
                }
            }
            
            // 3. 동/면/읍 추출 (정자일로, 금곡동 등)
            java.util.regex.Pattern dongPattern = java.util.regex.Pattern.compile("([가-힣]+[동면읍로])");
            java.util.regex.Matcher dongMatcher = dongPattern.matcher(address);
            while (dongMatcher.find()) {
                String match = dongMatcher.group(1);
                if (match.endsWith("동") && components.getEupMyeonDong() == null) {
                    // 괄호 안의 동 정보 우선 (예: 금곡동)
                    if (address.contains("(" + match + ")") || address.contains("（" + match + "）")) {
                        components.setEupMyeonDong(match);
                    }
                }
            }
            
            // 4. 동/호수 추출 (111동 901호)
            java.util.regex.Pattern buildingPattern = java.util.regex.Pattern.compile("(\\d+)동\\s*(\\d+)호");
            java.util.regex.Matcher buildingMatcher = buildingPattern.matcher(address);
            if (buildingMatcher.find()) {
                components.setBuildingDong(Integer.parseInt(buildingMatcher.group(1)));
                components.setBuildingHo(Integer.parseInt(buildingMatcher.group(2)));
            }
            
            // 5. 아파트명 추출 (청솔마을)
            java.util.regex.Pattern apartmentPattern = java.util.regex.Pattern.compile("\\(([가-힣\\s]+)\\)");
            java.util.regex.Matcher apartmentMatcher = apartmentPattern.matcher(address);
            while (apartmentMatcher.find()) {
                String match = apartmentMatcher.group(1).trim();
                // 동 이름이 아닌 아파트명 추출
                if (!match.endsWith("동") && match.length() > 2) {
                    String[] parts = match.split(",\\s*");
                    for (String part : parts) {
                        if (!part.endsWith("동") && part.length() > 2) {
                            components.setApartmentName(part.trim());
                            break;
                        }
                    }
                }
            }
            
            log.info("파싱 결과 - 시도: {}, 시군구: {}, 읍면동: {}, 동: {}, 호: {}, 아파트: {}", 
                    components.getSiDo(), components.getSiGunGu(), components.getEupMyeonDong(),
                    components.getBuildingDong(), components.getBuildingHo(), components.getApartmentName());
            
        } catch (Exception e) {
            log.warn("주소 파싱 중 오류 발생: {}", e.getMessage());
        }
        
        return components;
    }
    
    /**
     * 주소 구성 요소를 담는 내부 클래스
     */
    private static class AddressComponents {
        private String siDo;
        private String siGunGu;
        private String eupMyeonDong;
        private String apartmentName;
        private Integer buildingDong;
        private Integer buildingHo;
        
        public String getSiDo() { return siDo; }
        public void setSiDo(String siDo) { this.siDo = siDo; }
        
        public String getSiGunGu() { return siGunGu; }
        public void setSiGunGu(String siGunGu) { this.siGunGu = siGunGu; }
        
        public String getEupMyeonDong() { return eupMyeonDong; }
        public void setEupMyeonDong(String eupMyeonDong) { this.eupMyeonDong = eupMyeonDong; }
        
        public String getApartmentName() { return apartmentName; }
        public void setApartmentName(String apartmentName) { this.apartmentName = apartmentName; }
        
        public Integer getBuildingDong() { return buildingDong; }
        public void setBuildingDong(Integer buildingDong) { this.buildingDong = buildingDong; }
        
        public Integer getBuildingHo() { return buildingHo; }
        public void setBuildingHo(Integer buildingHo) { this.buildingHo = buildingHo; }
    }

    @Override
    @Transactional(readOnly = true)
    public NeighborhoodVerificationResponse getVerificationStatus(String userEmail) {
        NeighborhoodVerificationResponse response = new NeighborhoodVerificationResponse();
        
        try {
            // 사용자 조회
            UserEntity user = userMapper.findByEmail(userEmail);
            if (user == null) {
                throw new RuntimeException("사용자를 찾을 수 없습니다: " + userEmail);
            }
            
            response.setSuccess(true);
            response.setVerified(user.isAddressVerified());
            response.setVerifiedLocation(user.getVerifiedLocation());
            response.setVerifiedAddress(user.getVerifiedAddress());
            response.setVerificationDate(user.getVerificationDate());
            response.setMessage("인증 상태 조회 완료");
            
        } catch (Exception e) {
            log.error("동네 인증 상태 조회 실패", e);
            response.setSuccess(false);
            response.setMessage("인증 상태 조회에 실패했습니다: " + e.getMessage());
        }
        
        return response;
    }
}
