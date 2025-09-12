package org.example.nareugobackend.api.service.auth.info;

import static org.example.nareugobackend.domain.user.ProviderType.NAVER;
import static org.example.nareugobackend.domain.user.Sex.MAN;
import static org.example.nareugobackend.domain.user.Sex.WOMAN;

import org.apache.commons.lang3.StringUtils;
import org.example.nareugobackend.domain.user.ProviderType;
import org.example.nareugobackend.domain.user.Sex;

public record NaverUserInfo(
    String id,
    String name,
    String email,
    String gender,
    String mobile,
    String birthday,
    String birthyear
) implements SocialUserInfo {

  @Override
  public String getName() {
    return name;
  }

  @Override
  public String getEmail() {
    return email;
  }

  @Override
  public Sex getGender() {
    return StringUtils.equals(gender, "M") ? MAN : WOMAN;
  }

  @Override
  public String getBirth() {
    return String.format("%s-%s", birthyear, birthday);
  }

  @Override
  public String getProviderId() {
    return id;
  }

  @Override
  public String getPhoneNumber() {
    return mobile;
  }

  @Override
  public ProviderType getProviderType() {
    return NAVER;
  }
}
