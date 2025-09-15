package org.example.nareugobackend.mapper;

import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Param;
import org.example.nareugobackend.common.model.Payment;

@Mapper
public interface PaymentMapper {

    void save(@Param("payment") Payment payment);
}
