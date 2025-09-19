package org.example.nareugobackend.mapper;

import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Param;

@Mapper
public interface DeliveryMapper {
    int insertInitialDelivery(
            @Param("orderId") Long orderId,
            @Param("deliveryAddress") String deliveryAddress,
            @Param("status") String status,
            @Param("trackingNumber") String trackingNumber
    );
}
