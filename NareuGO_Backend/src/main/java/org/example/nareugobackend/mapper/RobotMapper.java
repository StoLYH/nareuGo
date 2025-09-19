package org.example.nareugobackend.mapper;

import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Param;

@Mapper
public interface RobotMapper {
    String findRobotStatusByName(@Param("robotName") String robotName);
}