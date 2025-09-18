package org.example.nareugobackend.common.model;

import java.time.LocalDateTime;
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
    
    private Boolean addressVerified; // DB tinyint(1)과 매칭
    private LocalDateTime verificationDate;
    private LocalDateTime deletedAt;
    
    /**
     * 주소 구성 요소를 업데이트
     */
    public void updateAddressVerification(String extractedAddress, boolean addressMatched) {
        this.addressVerified = addressMatched;
        if (addressMatched) {
            this.verificationDate = LocalDateTime.now();
            parseAndUpdateAddressComponents(extractedAddress);
        }
    }
    
    /**
     * 추출된 주소에서 시도, 시군구, 읍면동 정보를 파싱하여 업데이트
     */
     private void parseAndUpdateAddressComponents(String address) {
        if (address == null || address.isEmpty()) return;

        // 시/도
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

        // 구/시/군
        java.util.regex.Matcher districtMatcher = java.util.regex.Pattern.compile("([가-힣]+[시군구])")
                .matcher(address);
        if (districtMatcher.find()) this.siGunGu = districtMatcher.group(1);

        // 동/면/읍
        java.util.regex.Matcher neighborhoodMatcher = java.util.regex.Pattern.compile("([가-힣]+[동면읍])")
                .matcher(address);
        if (neighborhoodMatcher.find()) this.eupMyeonDong = neighborhoodMatcher.group(1);
    }
    
    /**
     * 주소 인증 여부를 확인할 때 MyBatis와 충돌 방지용 커스텀 메서드
     */
    public boolean isAddressVerifiedFlag() {
        return this.addressVerified != null && this.addressVerified;
    }
    
    /**
     * 인증된 주소 정보를 반환합니다.
     */
    public String getVerifiedAddress() {
        if (!isAddressVerifiedFlag()) {
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
