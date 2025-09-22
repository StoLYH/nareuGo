package org.example.nareugobackend.domain.delivery;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;
import org.springframework.stereotype.Repository;

import java.util.List;
import java.util.Optional;

@Repository
public interface DeliveryRepository extends JpaRepository<Delivery, Long> {

    Optional<Delivery> findByOrderId(Long orderId);

    Optional<Delivery> findByTrackingNumber(String trackingNumber);

    @Query("SELECT d FROM Delivery d JOIN d.order o WHERE o.buyer.id = :userId")
    List<Delivery> findDeliveriesByUserId(@Param("userId") Long userId);

    @Query("SELECT d FROM Delivery d " +
           "JOIN FETCH d.order o " +
           "JOIN FETCH o.buyer " +
           "WHERE d.id = :deliveryId")
    Optional<Delivery> findByIdWithOrderAndBuyer(@Param("deliveryId") Long deliveryId);
}