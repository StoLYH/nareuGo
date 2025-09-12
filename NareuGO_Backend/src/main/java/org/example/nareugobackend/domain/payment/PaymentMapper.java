package org.example.nareugobackend.domain.payment;

import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Param;

@Mapper
public interface PaymentMapper {

    void save(@Param("payment") Payment payment);
}
