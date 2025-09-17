package org.example.nareugobackend;
import org.mybatis.spring.annotation.MapperScan;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.scheduling.annotation.EnableScheduling;

@SpringBootApplication
@EnableScheduling
@MapperScan("org.example.nareugobackend.mapper")
public class NareuGoBackendApplication {

  public static void main(String[] args) {
    SpringApplication.run(NareuGoBackendApplication.class, args);
  }
}