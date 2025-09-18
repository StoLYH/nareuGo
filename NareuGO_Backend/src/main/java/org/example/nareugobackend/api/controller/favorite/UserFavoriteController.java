package org.example.nareugobackend.api.controller.favorite;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.service.favorite.UserFavoriteService;
import org.example.nareugobackend.api.service.favorite.response.FavoriteResponse;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequiredArgsConstructor
@RequestMapping("/favorites")
public class UserFavoriteController {

    private final UserFavoriteService userFavoriteService;

    @PostMapping("/toggle")
    public ResponseEntity<FavoriteResponse> toggleFavorite(
        @RequestParam Long userId,
        @RequestParam Long productId) {
        FavoriteResponse response = userFavoriteService.toggleFavorite(userId, productId);
        return ResponseEntity.ok(response);
    }

    @GetMapping("/check")
    public ResponseEntity<Boolean> isFavorite(
        @RequestParam Long userId,
        @RequestParam Long productId) {
        boolean isFavorite = userFavoriteService.isFavorite(userId, productId);
        return ResponseEntity.ok(isFavorite);
    }

    @GetMapping("/user/{userId}")
    public ResponseEntity<List<Long>> getUserFavoriteProductIds(@PathVariable Long userId) {
        List<Long> favoriteProductIds = userFavoriteService.getUserFavoriteProductIds(userId);
        return ResponseEntity.ok(favoriteProductIds);
    }
}