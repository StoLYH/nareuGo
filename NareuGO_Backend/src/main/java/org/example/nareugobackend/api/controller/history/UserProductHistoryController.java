package org.example.nareugobackend.api.controller.history;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.service.history.UserProductHistoryService;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequiredArgsConstructor
@RequestMapping("/history")
public class UserProductHistoryController {

    private final UserProductHistoryService userProductHistoryService;

    @PostMapping("/view")
    public ResponseEntity<Void> addProductView(
        @RequestParam Long userId,
        @RequestParam Long productId) {
        userProductHistoryService.addProductView(userId, productId);
        return ResponseEntity.ok().build();
    }

    @GetMapping("/user/{userId}")
    public ResponseEntity<List<Long>> getUserRecentViewedProductIds(@PathVariable Long userId) {
        List<Long> recentProductIds = userProductHistoryService.getUserRecentViewedProductIds(userId);
        return ResponseEntity.ok(recentProductIds);
    }

    @DeleteMapping("/user/{userId}")
    public ResponseEntity<Void> clearUserHistory(@PathVariable Long userId) {
        userProductHistoryService.clearUserHistory(userId);
        return ResponseEntity.ok().build();
    }
}