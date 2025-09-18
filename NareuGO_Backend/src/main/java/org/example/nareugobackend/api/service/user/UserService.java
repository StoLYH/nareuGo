package org.example.nareugobackend.api.service.user;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.service.auth.info.SocialUserInfo;
import org.example.nareugobackend.api.service.user.request.LoginServiceRequest;
import org.example.nareugobackend.api.service.user.response.LoginServiceResponse;
import org.example.nareugobackend.api.service.user.response.UserProfileResponse;
import org.example.nareugobackend.common.model.UserEntity;
import org.example.nareugobackend.domain.user.User;
import org.example.nareugobackend.domain.user.UserRepository;
import org.example.nareugobackend.common.exception.user.UserException;
import org.example.nareugobackend.common.exception.user.UserErrorCode;
import org.example.nareugobackend.mapper.UserMapper;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

@Service
@RequiredArgsConstructor
@Transactional(readOnly = true)
public class UserService {

  private final UserRepository userRepository;
  private final UserMapper userMapper;

  @Transactional
  public User registerUser(SocialUserInfo userInfo) {
    if (userRepository.existsMemberByEmail(userInfo.getEmail())) {
      return userRepository.findByEmail(userInfo.getEmail());
    }
    User savedUser = userRepository.save(User.from(userInfo));
    return savedUser;
  }

  public User findById(Long userId) {
    return userRepository.findById(userId)
        .orElseThrow(() -> new UserException(UserErrorCode.USER_NOT_FOUND));
  }

  // 기본 로그인 -> 건들지 X
  @Transactional
  public LoginServiceResponse loginByEmail(LoginServiceRequest request) {
    try {
      // 이메일로 사용자 조회
      UserEntity user = userMapper.findByEmail(request.getEmail());
      
      if (user == null) {
        return LoginServiceResponse.failure("등록되지 않은 이메일입니다.");
      }

      return LoginServiceResponse.success(
          user.getId(),
          user.getEmail(),
          user.getName()
      );

    } catch (Exception e) {
      return LoginServiceResponse.failure("로그인 처리 중 오류가 발생했습니다.");
    }
  }

  public UserProfileResponse getUserProfile(Long userId) {
    User user = findById(userId);

    return UserProfileResponse.builder()
        .userId(user.getId())
        .nickname(user.getNickname())
        .email(user.getEmail())
        .apartmentName(user.getApartmentName())
        .buildingDong(String.valueOf(user.getBuildingDong()))
        .buildingHo(String.valueOf(user.getBuildingHo()))
        .siDo(user.getSiDo())
        .siGunGu(user.getSiGunGu())
        .eupMyeonDong(user.getEupMyeonDong())
        .addressVerified(user.getNickname() != null) // nickname이 있으면 인증된 것으로 가정
        .build();
  }

}