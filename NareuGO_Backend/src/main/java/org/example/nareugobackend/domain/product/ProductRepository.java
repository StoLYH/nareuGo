package org.example.nareugobackend.domain.product;

import java.util.List;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.repository.query.Param;

public interface ProductRepository extends JpaRepository<Product, Long> {

    List<Product> findBySellerIdOrderByCreatedAtDesc(@Param("sellerId") Long sellerId);

    List<Product> findByStatusOrderByCreatedAtDesc(@Param("status") Product.ProductStatus status);
}