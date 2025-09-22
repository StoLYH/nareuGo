package org.example.nareugobackend.mapper;

import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Param;
import org.example.nareugobackend.api.controller.robot.response.DeliveryAddressResponse;

@Mapper
public interface RobotMapper {
    String findRobotStatusByName(@Param("robotName") String robotName);

    DeliveryAddressResponse getDeliveryAddresses(@Param("deliveryId") Long deliveryId);
}