package org.example.nareugobackend.common.model;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@Getter
@Setter
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class UserEntity {
    
    private Long id;
    private String email;
    private String name;
    private String sex;
    private String birth;
    private String phoneNumber;
    private String providerType;
    private String providerId;
    private Boolean isActive;
    private String nickname;
    private Integer buildingDong;
    private Integer buildingHo;
    private String apartmentName;
    private String siDo;
    private String siGunGu;
    private String eupMyeonDong;
    private String role;
    
    // 주소 인증 관련 필드들 (새로 추가될 컬럼)
    private Boolean addressVerified = false;
    private java.time.LocalDateTime verificationDate;
    
    /**
     * 주소 인증 정보를 업데이트합니다.
     */
    public void updateAddressVerification(String extractedAddress, String gpsAddress, 
                                        boolean addressMatched) {
        this.addressVerified = addressMatched;
        if (addressMatched) {
            this.verificationDate = java.time.LocalDateTime.now();
            // 추출된 주소에서 주소 구성요소 파싱하여 기존 필드에 저장
            parseAndUpdateAddressComponents(extractedAddress);
        }
    }
    
    /**
     * 추출된 주소에서 시도, 시군구, 읍면동 정보를 파싱하여 업데이트
     */
    private void parseAndUpdateAddressComponents(String address) {
        if (address == null || address.isEmpty()) return;
        
        // 시/도 추출
        String[] cityKeywords = {"서울특별시", "부산광역시", "대구광역시", "인천광역시", 
                               "광주광역시", "대전광역시", "울산광역시", "세종특별자치시",
                               "경기도", "강원도", "충청북도", "충청남도", "전라북도", 
                               "전라남도", "경상북도", "경상남도", "제주특별자치도"};
        
        for (String city : cityKeywords) {
            if (address.contains(city)) {
                this.siDo = city;
                break;
            }
        }
        
        // 구/시/군 추출
        java.util.regex.Pattern districtPattern = java.util.regex.Pattern.compile("([가-힣]+[시군구])");
        java.util.regex.Matcher districtMatcher = districtPattern.matcher(address);
        if (districtMatcher.find()) {
            this.siGunGu = districtMatcher.group(1);
        }
        
        // 동/면/읍 추출
        java.util.regex.Pattern neighborhoodPattern = java.util.regex.Pattern.compile("([가-힣]+[동면읍])");
        java.util.regex.Matcher neighborhoodMatcher = neighborhoodPattern.matcher(address);
        if (neighborhoodMatcher.find()) {
            this.eupMyeonDong = neighborhoodMatcher.group(1);
        }
    }
    
    /**
     * 주소 인증 여부를 확인합니다.
     */
    public boolean isAddressVerified() {
        return this.addressVerified != null && this.addressVerified;
    }
    
    /**
     * 인증된 위치 정보를 반환합니다.
     */
    public String getVerifiedLocation() {
        if (!isAddressVerified()) {
            return null;
        }
        StringBuilder location = new StringBuilder();
        if (siDo != null) location.append(siDo).append(" ");
        if (siGunGu != null) location.append(siGunGu).append(" ");
        if (eupMyeonDong != null) location.append(eupMyeonDong);
        return location.toString().trim();
    }
    
    /**
     * 인증된 주소 정보를 반환합니다.
     */
    public String getVerifiedAddress() {
        if (!isAddressVerified()) {
            return null;
        }
        StringBuilder address = new StringBuilder();
        if (siDo != null) address.append(siDo).append(" ");
        if (siGunGu != null) address.append(siGunGu).append(" ");
        if (eupMyeonDong != null) address.append(eupMyeonDong).append(" ");
        if (apartmentName != null) address.append(apartmentName);
        return address.toString().trim();
    }
}
