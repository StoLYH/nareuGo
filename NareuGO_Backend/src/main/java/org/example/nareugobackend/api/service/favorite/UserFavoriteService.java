package org.example.nareugobackend.api.service.favorite;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.service.favorite.response.FavoriteResponse;
import org.example.nareugobackend.api.service.notification.FcmService;
import org.example.nareugobackend.domain.favorite.UserFavorite;
import org.example.nareugobackend.domain.favorite.UserFavoriteRepository;
import org.example.nareugobackend.domain.product.Product;
import org.example.nareugobackend.domain.product.ProductRepository;
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
public class UserFavoriteService {

    private final UserFavoriteRepository userFavoriteRepository;
    private final UserRepository userRepository;
    private final ProductRepository productRepository;
    private final FcmService fcmService;

    @Transactional
    public FavoriteResponse toggleFavorite(Long userId, Long productId) {
        User user = userRepository.findById(userId)
            .orElseThrow(() -> new UserException(UserErrorCode.USER_NOT_FOUND));

        if (userFavoriteRepository.existsByUserIdAndProductId(userId, productId)) {
            userFavoriteRepository.deleteByUserIdAndProductId(userId, productId);
            return FavoriteResponse.builder()
                .isFavorite(false)
                .message("관심목록에서 제거되었습니다.")
                .build();
        } else {
            UserFavorite favorite = UserFavorite.create(user, productId);
            userFavoriteRepository.save(favorite);

            // 상품 정보 조회 및 판매자에게 알림 발송
            Product product = productRepository.findById(productId).orElse(null);
            if (product != null && !product.getSeller().getId().equals(userId)) {
                fcmService.sendLikeNotification(
                    product.getSeller().getId(),
                    product.getTitle(),
                    user.getNickname() != null ? user.getNickname() : user.getName()
                );
            }

            return FavoriteResponse.builder()
                .isFavorite(true)
                .message("관심목록에 추가되었습니다.")
                .build();
        }
    }

    public boolean isFavorite(Long userId, Long productId) {
        return userFavoriteRepository.existsByUserIdAndProductId(userId, productId);
    }

    public List<Long> getUserFavoriteProductIds(Long userId) {
        return userFavoriteRepository.findProductIdsByUserIdOrderByCreatedAtDesc(userId);
    }
}