package org.example.nareugobackend.mapper;

import java.util.List;
import java.util.Optional;
import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Param;
import org.example.nareugobackend.common.model.Order;
import org.example.nareugobackend.common.model.OrderStatus;

@Mapper
public interface OrderMapper {

    Order findById(@Param("orderId") Long orderId);
    Optional<Order> findByProductId(@Param("productId") Long productId);
    Order findByTossOrderId(@Param("tossOrderId") String tossOrderId);

    void updateStatus(@Param("orderId") Long orderId, @Param("status") OrderStatus status);

    void insert(@Param("order") Order order);

    void repend(@Param("orderId") Long orderId,
                @Param("buyerId") Long buyerId,
                @Param("amount") java.math.BigDecimal amount,
                @Param("tossOrderId") String tossOrderId);

    java.util.List<Order> findExpiredPendingOrders();

    void expireOrders(@Param("orderIds") java.util.List<Long> orderIds);
    
    // 채팅방 ID로 상품 ID 조회 (채팅 중인 상품 정보 가져오기용)
    Long getProductIdByRoomId(@Param("roomId") Long roomId);

    // 판매자별 주문 목록 조회
    List<Order> findOrdersBySeller(@Param("sellerId") Long sellerId, @Param("status") String status);
}
