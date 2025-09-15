package org.example.nareugobackend.api.service.auth.info;

import org.example.nareugobackend.domain.user.ProviderType;
import org.example.nareugobackend.domain.user.Sex;

public interface SocialUserInfo {

  String getName();

  Sex getGender();

  String getEmail();

  String getBirth();

  String getProviderId();

  ProviderType getProviderType();

  String getPhoneNumber();

}