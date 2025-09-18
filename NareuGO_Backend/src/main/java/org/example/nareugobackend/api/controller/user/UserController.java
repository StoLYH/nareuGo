
package org.example.nareugobackend.api.controller.user;
import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.service.user.UserService;
import org.example.nareugobackend.api.service.user.request.LoginServiceRequest;
import org.example.nareugobackend.api.service.user.response.LoginServiceResponse;
import org.example.nareugobackend.common.model.UserEntity;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
@RestController
@RequiredArgsConstructor
@RequestMapping("/general-login")
public class UserController {

    private final UserService userService;

    @PostMapping()
    public ResponseEntity<LoginServiceResponse> login(@RequestBody LoginServiceRequest request) {
        LoginServiceResponse response = userService.loginByEmail(request);
        return ResponseEntity.ok(response);
    }

     // 결제용 사용자 정보 조회 (기존 코드와 분리)
    @GetMapping("/users/{userId}")
    public ResponseEntity<UserEntity> getUserInfo(@PathVariable Long userId) {
        UserEntity user = userService.getUserById(userId);
        return ResponseEntity.ok(user);
    }
}