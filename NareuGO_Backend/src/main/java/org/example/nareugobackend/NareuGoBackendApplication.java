package org.example.nareugobackend;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.scheduling.annotation.EnableScheduling;

@SpringBootApplication
@EnableScheduling
public class NareuGoBackendApplication {

    public static void main(String[] args) {
        SpringApplication.run(NareuGoBackendApplication.class, args);
    }

}


// 빌드 테스트11