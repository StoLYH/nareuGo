package org.example.nareugobackend.search;

import org.springframework.data.domain.Page;
import org.springframework.data.domain.Pageable;
import org.springframework.data.elasticsearch.repository.ElasticsearchRepository;

/**
 * Elasticsearch Repository.
 * - 파생 쿼리 메서드로 간단한 제목/설명 부분일치 검색을 제공합니다.
 * - 복잡한 검색이 필요하면 QueryBuilders를 사용하는 커스텀 리포지토리로 확장할 수 있습니다.
 */
public interface ProductSearchRepository extends ElasticsearchRepository<ProductDocument, Long> {

  Page<ProductDocument> findByTitleContainingOrDescriptionContaining(String title, String description, Pageable pageable);
}


