package org.example.nareugobackend.api.controller.mypage;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.controller.user.response.MyPageResponse;
import org.example.nareugobackend.api.service.mypage.MyPageService;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

@RestController
@RequiredArgsConstructor
@RequestMapping("/mypage")
public class MyPageController {

    private final MyPageService myPageService;

    @GetMapping("/user/{userId}")
    public ResponseEntity<MyPageResponse> getMyPage(@PathVariable Long userId) {
        MyPageResponse response = myPageService.getMyPage(userId);
        return ResponseEntity.ok(response);
    }
}