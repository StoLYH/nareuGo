package org.example.nareugobackend.domain.order;

import java.util.List;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;

public interface OrderRepository extends JpaRepository<Order, Long> {

    // 구매내역 조회 (특정 사용자가 구매한 주문들)
    List<Order> findByBuyerIdOrderByCreatedAtDesc(@Param("buyerId") Long buyerId);

    // 판매내역 조회를 위한 쿼리 (특정 사용자가 판매한 상품들의 주문)
    @Query("""
        SELECT o FROM Order o
        JOIN Product p ON o.productId = p.id
        WHERE p.seller.id = :sellerId
        ORDER BY o.createdAt DESC
        """)
    List<Order> findBySellerId(@Param("sellerId") Long sellerId);

    // 특정 상품의 주문 조회
    List<Order> findByProductIdOrderByCreatedAtDesc(@Param("productId") Long productId);

    // 특정 상태의 주문 조회
    List<Order> findByStatusOrderByCreatedAtDesc(@Param("status") Order.OrderStatus status);
}