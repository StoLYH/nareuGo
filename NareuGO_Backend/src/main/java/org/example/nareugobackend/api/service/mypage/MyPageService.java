package org.example.nareugobackend.api.service.mypage;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.controller.user.response.MyPageResponse;
import org.example.nareugobackend.domain.user.User;
import org.example.nareugobackend.domain.user.UserRepository;
import org.example.nareugobackend.common.exception.user.UserException;
import org.example.nareugobackend.common.exception.user.UserErrorCode;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

@Service
@RequiredArgsConstructor
@Transactional(readOnly = true)
public class MyPageService {

    private final UserRepository userRepository;

    public User findById(Long userId) {
        return userRepository.findById(userId)
            .orElseThrow(() -> new UserException(UserErrorCode.USER_NOT_FOUND));
    }

    public MyPageResponse getMyPage(Long userId) {
        User user = findById(userId);

        return MyPageResponse.builder()
            .name(user.getName())
            .siGunGu(user.getSiGunGu())
            .eupMyeonDong(user.getEupMyeonDong())
            .apartName(user.getApartmentName())
            .buildingDong(String.valueOf(user.getBuildingDong()))
            .buildingHo(String.valueOf(user.getBuildingHo()))
            .build();
    }
}