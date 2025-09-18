package org.example.nareugobackend.domain.history;

import static jakarta.persistence.GenerationType.IDENTITY;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.FetchType;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.Id;
import jakarta.persistence.JoinColumn;
import jakarta.persistence.ManyToOne;
import jakarta.persistence.Table;
import java.time.Instant;
import lombok.AccessLevel;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;
import org.example.nareugobackend.domain.user.User;

@Getter
@Entity
@Builder
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@AllArgsConstructor
@Table(name = "user_product_history")
public class UserProductHistory {

    @Id
    @GeneratedValue(strategy = IDENTITY)
    @Column(name = "history_id")
    private Long id;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "user_id", nullable = false)
    private User user;

    @Column(name = "product_id", nullable = false)
    private Long productId;

    @Column(name = "viewed_at", nullable = false)
    @Builder.Default
    private Instant viewedAt = Instant.now();

    public static UserProductHistory create(User user, Long productId) {
        return UserProductHistory.builder()
            .user(user)
            .productId(productId)
            .build();
    }
}