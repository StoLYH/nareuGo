package org.example.nareugobackend.search;

import java.time.ZoneOffset;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.controller.product.response.ProductDetailResponse;
import org.example.nareugobackend.mapper.ProductMapper;
import org.example.nareugobackend.util.S3UrlGenerator;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.PageRequest;
import org.springframework.data.domain.Pageable;
import org.springframework.data.domain.PageImpl;
import org.springframework.data.elasticsearch.client.elc.NativeQuery;
import org.springframework.data.elasticsearch.core.ElasticsearchOperations;
import org.springframework.data.elasticsearch.core.SearchHits;
import org.springframework.stereotype.Service;
import co.elastic.clients.elasticsearch._types.query_dsl.MultiMatchQuery;
import co.elastic.clients.elasticsearch._types.query_dsl.Query;
import co.elastic.clients.elasticsearch._types.query_dsl.TextQueryType;
import co.elastic.clients.elasticsearch._types.query_dsl.Operator;

/**
 * 상품 검색/색인 서비스.
 * - indexProduct: DB에서 상품을 조회해 ES에 색인합니다.
 * - deleteFromIndex: ES 문서를 제거합니다.
 * - search: 제목/설명 기반의 간단 검색을 제공합니다.
 */
@Service
@RequiredArgsConstructor
public class ProductSearchService {

  private final ProductMapper productMapper;
  private final ProductSearchRepository repository;
  private final ElasticsearchOperations operations;
  private final S3UrlGenerator s3UrlGenerator;

  /** 상품을 ES에 색인 */
  public void indexProduct(Long productId) {
    // 상품 기본 정보 조회
    ProductDetailResponse src = productMapper.selectOneProduct(productId);
    if (src == null) {
      return;
    }

    // 이미지 정보 조회
    List<String> imageKeys = productMapper.selectProductImages(productId);

    ProductDocument doc = new ProductDocument();
    doc.setId(src.getProductId());
    doc.setTitle(src.getTitle());
    doc.setDescription(src.getDescription());
    doc.setApartmentName(src.getApartmentName());
    doc.setSiDo(src.getSiDo());
    doc.setSiGunGu(src.getSiGunGu());
    doc.setEupMyeonDong(src.getEupMyeonDong());
    doc.setStatus(src.getStatus());
    if (src.getCreatedAt() != null) {
      doc.setCreatedAt(src.getCreatedAt().toInstant(ZoneOffset.UTC));
    }
    doc.setPrice(src.getPrice());
    doc.setImageUrls(imageKeys);

    repository.save(doc);
  }

  /** ES 상품 제거 */
  public void deleteFromIndex(Long productId) {
    repository.deleteById(productId);
  }


  /** 제목 제안 상위 N개만 반환 (문자열 배열) */
  public List<String> suggest(String q, int size) {
    if (q == null || q.isBlank()) {
      return List.of();
    }

    MultiMatchQuery.Builder mmBuilder = new MultiMatchQuery.Builder();
    mmBuilder.query(q); 
    mmBuilder.type(TextQueryType.BoolPrefix);
    mmBuilder.fields("title", "title._2gram", "title._3gram");

    Query esQuery = new Query.Builder().multiMatch(mmBuilder.build()).build();

    NativeQuery nativeQuery = NativeQuery.builder()
        .withQuery(esQuery)
        .withPageable(PageRequest.of(0, size))
        .build();

    SearchHits<ProductDocument> hits = operations.search(nativeQuery, ProductDocument.class);

    Set<String> unique = new LinkedHashSet<>();
    List<String> result = new ArrayList<>();

    for (org.springframework.data.elasticsearch.core.SearchHit<ProductDocument> hit : hits) {
      ProductDocument doc = hit.getContent();
      if (doc != null) {
        String title = doc.getTitle();
        if (title != null && !title.isBlank() && !unique.contains(title)) {
          unique.add(title);
          result.add(title);
          if (result.size() >= size) {
            break;
          }
        }
      }
    }

    return result;
  }


  /**
   * Nori 기반 전체 텍스트 검색 (제목 + 설명)
   * */
  public Page<ProductDocument> searchNori(String q, int page, int size) {
    Pageable pageable = PageRequest.of(page, size);
    if (q == null || q.isBlank()) {
      return Page.empty(pageable);
    }

    MultiMatchQuery.Builder mmBuilder = new MultiMatchQuery.Builder();
    mmBuilder.query(q);
    mmBuilder.operator(Operator.And);
    mmBuilder.fuzziness("AUTO");
    // 제목과 설명을 함께 검색 (둘 다 nori 애널라이저 사용)
    mmBuilder.fields("title^2", "description");

    Query esQuery = new Query.Builder().multiMatch(mmBuilder.build()).build();

    NativeQuery nativeQuery = NativeQuery.builder()
        .withQuery(esQuery)
        .withPageable(pageable)
        .build();

    SearchHits<ProductDocument> hits = operations.search(nativeQuery, ProductDocument.class);
    List<ProductDocument> contents = new ArrayList<>();
    for (org.springframework.data.elasticsearch.core.SearchHit<ProductDocument> hit : hits) {
      ProductDocument doc = hit.getContent();
      convertImageKeysToUrls(doc);
      contents.add(doc);
    }
    return new PageImpl<>(contents, pageable, hits.getTotalHits());
  }



  /**
   * ProductDocument의 S3 키를 Presigned URL로 변환
   */
  private void convertImageKeysToUrls(ProductDocument doc) {
    if (doc.getImageUrls() != null && !doc.getImageUrls().isEmpty()) {
      List<String> presignedUrls = new ArrayList<>();
      for (String s3Key : doc.getImageUrls()) {
        String presignedUrl = s3UrlGenerator.generatePresignedDownloadUrl(s3Key);
        presignedUrls.add(presignedUrl);
      }
      doc.setImageUrls(presignedUrls);
    }
  }
}


