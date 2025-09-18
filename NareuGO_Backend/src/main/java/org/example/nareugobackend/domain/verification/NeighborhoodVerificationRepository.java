package org.example.nareugobackend.domain.verification;

import java.util.List;
import java.util.Optional;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.repository.query.Param;

public interface NeighborhoodVerificationRepository extends JpaRepository<NeighborhoodVerification, Long> {

    List<NeighborhoodVerification> findByUserIdOrderBySubmittedAtDesc(@Param("userId") Long userId);

    Optional<NeighborhoodVerification> findTopByUserIdOrderBySubmittedAtDesc(@Param("userId") Long userId);

    List<NeighborhoodVerification> findByStatus(@Param("status") NeighborhoodVerification.VerificationStatus status);

    boolean existsByUserIdAndStatus(@Param("userId") Long userId, @Param("status") NeighborhoodVerification.VerificationStatus status);
}