package org.example.nareugobackend.api.controller.Health;

import java.util.Map;
import java.util.TreeMap;
import java.util.concurrent.TimeUnit;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.data.redis.core.StringRedisTemplate;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
public class RedisHealthCheckController {

    @Autowired
    private StringRedisTemplate stringRedisTemplate;

    @GetMapping("/redis/health")
    public ResponseEntity<?> redisHealthCheck() {
        Map<String, Object> responseData = new TreeMap<>();

        try {
            stringRedisTemplate.opsForValue().set("health_check", "ok", 300, TimeUnit.SECONDS);

            String value = stringRedisTemplate.opsForValue().get("health_check");

            responseData.put("status", "OK");
            responseData.put("message", "Redis 연결 정상");
            responseData.put("test_value", value);
            responseData.put("expire_time", "5분");

        } catch (Exception e) {
            responseData.put("status", "FAILED");
            responseData.put("message", "Redis 연결 실패");
            responseData.put("error", e.getMessage());
        }

        return ResponseEntity.ok(responseData);
    }
}
