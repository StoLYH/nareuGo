package org.example.nareugobackend.search;

import java.math.BigDecimal;
import java.time.Instant;
import org.springframework.data.annotation.Id;
import org.springframework.data.elasticsearch.annotations.Document;
import org.springframework.data.elasticsearch.annotations.Setting;
import org.springframework.data.elasticsearch.annotations.Field;
import org.springframework.data.elasticsearch.annotations.FieldType;

/**
 * Elasticsearch 문서 모델.
 * - DB의 상품 엔티티를 검색 친화적으로 변환하여 색인합니다.
 * - 검색은 title/description 위주로 수행하며, 지역/상태 등은 필터에 활용할 수 있습니다.
 */
@Document(indexName = "products")
@Setting(settingPath = "elasticsearch/products-settings.json")
public class ProductDocument {

  /** 상품 ID (PK) */
  @Id
  private Long id;

  /** 제목 (자동완성용 search_as_you_type + nori) */
  @Field(type = FieldType.Search_As_You_Type, analyzer = "korean_nori", searchAnalyzer = "korean_nori")
  private String title;
  // 최초 토큰화 + 검색 => korean_nori 애널라이저 사용.
  // 자동완성 기능 -> Search_As_You_Type


  /** 설명 (전문 검색 대상 + nori) */
  @Field(type = FieldType.Text, analyzer = "korean_nori", searchAnalyzer = "korean_nori")
  private String description;

  /** 아파트명 (정확 일치 필터) */
  @Field(type = FieldType.Keyword)
  private String apartmentName;

  /** 시/도 (정확 일치 필터) */
  @Field(type = FieldType.Keyword)
  private String siDo;

  /** 시/군/구 (정확 일치 필터) */
  @Field(type = FieldType.Keyword)
  private String siGunGu;

  /** 읍/면/동 (정확 일치 필터) */
  @Field(type = FieldType.Keyword)
  private String eupMyeonDong;

  /** 상태 (FOR_SALE, SOLD 등) */
  @Field(type = FieldType.Keyword)
  private String status;

  /** 가격 (정렬/범위검색 용도 가능) */
  @Field(type = FieldType.Double)
  private BigDecimal price;

  /** 생성일시 (정렬용) */
  @Field(type = FieldType.Date)
  private Instant createdAt;

  /** 이미지 URL 목록 (첫 번째 이미지가 대표 이미지) */
  @Field(type = FieldType.Keyword)
  private java.util.List<String> imageUrls;

  public Long getId() { return id; }
  public void setId(Long id) { this.id = id; }
  public String getTitle() { return title; }
  public void setTitle(String title) { this.title = title; }
  public String getDescription() { return description; }
  public void setDescription(String description) { this.description = description; }
  public String getApartmentName() { return apartmentName; }
  public void setApartmentName(String apartmentName) { this.apartmentName = apartmentName; }
  public String getSiDo() { return siDo; }
  public void setSiDo(String siDo) { this.siDo = siDo; }
  public String getSiGunGu() { return siGunGu; }
  public void setSiGunGu(String siGunGu) { this.siGunGu = siGunGu; }
  public String getEupMyeonDong() { return eupMyeonDong; }
  public void setEupMyeonDong(String eupMyeonDong) { this.eupMyeonDong = eupMyeonDong; }
  public String getStatus() { return status; }
  public void setStatus(String status) { this.status = status; }
  public BigDecimal getPrice() { return price; }
  public void setPrice(BigDecimal price) { this.price = price; }
  public Instant getCreatedAt() { return createdAt; }
  public void setCreatedAt(Instant createdAt) { this.createdAt = createdAt; }
  public java.util.List<String> getImageUrls() { return imageUrls; }
  public void setImageUrls(java.util.List<String> imageUrls) { this.imageUrls = imageUrls; }
}


