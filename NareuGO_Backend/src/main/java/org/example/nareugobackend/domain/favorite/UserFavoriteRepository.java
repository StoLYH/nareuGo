package org.example.nareugobackend.domain.favorite;

import java.util.List;
import java.util.Optional;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;

public interface UserFavoriteRepository extends JpaRepository<UserFavorite, Long> {

    Optional<UserFavorite> findByUserIdAndProductId(@Param("userId") Long userId, @Param("productId") Long productId);

    boolean existsByUserIdAndProductId(@Param("userId") Long userId, @Param("productId") Long productId);

    void deleteByUserIdAndProductId(@Param("userId") Long userId, @Param("productId") Long productId);

    @Query("SELECT uf.productId FROM UserFavorite uf WHERE uf.user.id = :userId ORDER BY uf.createdAt DESC")
    List<Long> findProductIdsByUserIdOrderByCreatedAtDesc(@Param("userId") Long userId);

    List<UserFavorite> findByUserIdOrderByCreatedAtDesc(@Param("userId") Long userId);
}