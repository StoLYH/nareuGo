-- 토스페이먼츠 orderId 지원을 위한 orders 테이블 업데이트
-- 실행 필요: 이 SQL을 데이터베이스에서 실행해야 함

ALTER TABLE orders 
ADD COLUMN toss_order_id VARCHAR(64) NULL COMMENT '토스페이먼츠용 orderId (영문+숫자+특수문자, 6-64자)';

-- 인덱스 추가 (토스페이먼츠 결제 확인 시 빠른 조회를 위해)
CREATE INDEX idx_orders_toss_order_id ON orders(toss_order_id);
