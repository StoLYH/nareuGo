package org.example.nareugobackend.domain.history;

import java.util.List;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;

public interface UserProductHistoryRepository extends JpaRepository<UserProductHistory, Long> {

    @Query("SELECT DISTINCT uph.productId FROM UserProductHistory uph WHERE uph.user.id = :userId ORDER BY uph.viewedAt DESC")
    List<Long> findDistinctProductIdsByUserIdOrderByViewedAtDesc(@Param("userId") Long userId);

    List<UserProductHistory> findByUserIdOrderByViewedAtDesc(@Param("userId") Long userId);

    void deleteByUserIdAndProductId(@Param("userId") Long userId, @Param("productId") Long productId);
}