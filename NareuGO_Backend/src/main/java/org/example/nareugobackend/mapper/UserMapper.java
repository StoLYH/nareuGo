package org.example.nareugobackend.mapper;

import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Param;
import org.example.nareugobackend.domain.user.User;

@Mapper
public interface UserMapper {
    
    User findByEmail(@Param("email") String email);
    
    boolean existsByEmail(@Param("email") String email);
}
