package org.example.nareugobackend.domain.payment;

import java.util.Optional;
import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Param;

@Mapper
public interface OrderMapper {

    Optional<Order> findById(String orderId);

    void updateStatus(@Param("orderId") Long orderId, @Param("status") OrderStatus status);
}
