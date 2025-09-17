package org.example.nareugobackend.mapper;

import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Param;
import org.example.nareugobackend.common.model.UserEntity;

@Mapper
public interface UserMapper {
    
    UserEntity findByEmail(@Param("email") String email);
    
    UserEntity findById(@Param("id") Long id);
    
    boolean existsByEmail(@Param("email") String email);
    
    /**
     * 사용자의 동네 인증 정보를 업데이트합니다.
     */
    int updateNeighborhoodVerification(UserEntity user);
}
