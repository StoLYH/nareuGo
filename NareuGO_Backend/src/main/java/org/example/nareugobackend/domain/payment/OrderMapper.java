package org.example.nareugobackend.domain.payment;

import java.util.Optional;
import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Param;

@Mapper
public interface OrderMapper {

    Optional<Order> findById(String orderId);
    Optional<Order> findByProductId(@Param("productId") Long productId);

    void updateStatus(@Param("orderId") Long orderId, @Param("status") OrderStatus status);

    void insert(@Param("order") Order order);

    void repend(@Param("orderId") Long orderId,
                @Param("buyerId") Long buyerId,
                @Param("amount") java.math.BigDecimal amount);
}
