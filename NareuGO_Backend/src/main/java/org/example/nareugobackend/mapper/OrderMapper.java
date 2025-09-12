package org.example.nareugobackend.mapper;

import java.util.Optional;
import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Param;
import org.example.nareugobackend.common.model.Order;
import org.example.nareugobackend.common.model.OrderStatus;

@Mapper
public interface OrderMapper {

    Optional<Order> findById(String orderId);
    Optional<Order> findByProductId(@Param("productId") Long productId);

    void updateStatus(@Param("orderId") Long orderId, @Param("status") OrderStatus status);

    void insert(@Param("order") Order order);

    void repend(@Param("orderId") Long orderId,
                @Param("buyerId") Long buyerId,
                @Param("amount") java.math.BigDecimal amount);

    java.util.List<Order> findExpiredPendingOrders();

    void expireOrders(@Param("orderIds") java.util.List<Long> orderIds);
}
