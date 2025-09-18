package org.example.nareugobackend.domain.notification;

import java.util.List;
import java.util.Optional;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.repository.query.Param;

public interface FcmTokenRepository extends JpaRepository<FcmToken, Long> {

    List<FcmToken> findByUserIdAndIsActiveTrue(@Param("userId") Long userId);

    Optional<FcmToken> findByUserIdAndTokenAndIsActiveTrue(
        @Param("userId") Long userId,
        @Param("token") String token
    );

    List<FcmToken> findByTokenAndIsActiveTrue(@Param("token") String token);

    void deleteByUserIdAndToken(@Param("userId") Long userId, @Param("token") String token);
}