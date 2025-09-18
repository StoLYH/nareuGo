package org.example.nareugobackend.api.service.favorite.response;

import lombok.Builder;
import lombok.Getter;

@Getter
@Builder
public class FavoriteResponse {
    private boolean isFavorite;
    private String message;
}