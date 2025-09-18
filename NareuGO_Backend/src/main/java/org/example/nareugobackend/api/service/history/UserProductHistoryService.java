package org.example.nareugobackend.api.service.history;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.domain.history.UserProductHistory;
import org.example.nareugobackend.domain.history.UserProductHistoryRepository;
import org.example.nareugobackend.domain.user.User;
import org.example.nareugobackend.domain.user.UserRepository;
import org.example.nareugobackend.common.exception.user.UserException;
import org.example.nareugobackend.common.exception.user.UserErrorCode;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.util.List;

@Service
@RequiredArgsConstructor
@Transactional(readOnly = true)
public class UserProductHistoryService {

    private final UserProductHistoryRepository userProductHistoryRepository;
    private final UserRepository userRepository;

    @Transactional
    public void addProductView(Long userId, Long productId) {
        User user = userRepository.findById(userId)
            .orElseThrow(() -> new UserException(UserErrorCode.USER_NOT_FOUND));

        UserProductHistory history = UserProductHistory.create(user, productId);
        userProductHistoryRepository.save(history);
    }

    public List<Long> getUserRecentViewedProductIds(Long userId) {
        return userProductHistoryRepository.findDistinctProductIdsByUserIdOrderByViewedAtDesc(userId);
    }

    @Transactional
    public void clearUserHistory(Long userId) {
        List<UserProductHistory> histories = userProductHistoryRepository.findByUserIdOrderByViewedAtDesc(userId);
        userProductHistoryRepository.deleteAll(histories);
    }
}