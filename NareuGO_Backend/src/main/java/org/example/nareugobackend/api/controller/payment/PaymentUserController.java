package org.example.nareugobackend.api.controller.payment;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.service.user.UserService;
import org.example.nareugobackend.common.model.UserEntity;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

// ===== 결제용 사용자 정보 컨트롤러 (기존 코드와 분리) =====
@RestController
@RequiredArgsConstructor
@RequestMapping("/payment/users")
public class PaymentUserController {

    private final UserService userService;

    /**
     * 결제용 사용자 정보 조회 (판매자/구매자 이름 등)
     * @param userId 사용자 ID
     * @return UserEntity
     */
    @GetMapping("/{userId}")
    public ResponseEntity<UserEntity> getUserForPayment(@PathVariable Long userId) {
        UserEntity user = userService.getUserById(userId);
        if (user == null) {
            return ResponseEntity.notFound().build();
        }
        return ResponseEntity.ok(user);
    }
}
