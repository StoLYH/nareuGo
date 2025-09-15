package org.example.nareugobackend.domain.user;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface UserRepository extends JpaRepository<User, Long> {

  boolean existsMemberByEmail(String email);

  User findByEmail(String email);
}