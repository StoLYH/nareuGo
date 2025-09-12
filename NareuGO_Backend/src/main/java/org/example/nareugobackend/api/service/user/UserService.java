package org.example.nareugobackend.api.service.user;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.service.auth.info.SocialUserInfo;
import org.example.nareugobackend.domain.user.User;
import org.example.nareugobackend.domain.user.UserRepository;
import org.example.nareugobackend.common.exception.user.UserException;
import org.example.nareugobackend.common.exception.user.UserErrorCode;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

@Service
@RequiredArgsConstructor
@Transactional(readOnly = true)
public class UserService {

  private final UserRepository userRepository;

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
}