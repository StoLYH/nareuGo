-- -----------------------------------------------------
-- 1. 회원 관련 테이블
-- -----------------------------------------------------
CREATE TABLE IF NOT EXISTS `users` (
  `user_id` BIGINT NOT NULL AUTO_INCREMENT COMMENT '사용자 고유 ID',
  `email` VARCHAR(100) NOT NULL COMMENT '로그인 이메일',
  `nickname` VARCHAR(50) NOT NULL COMMENT '닉네임',
  `building_dong` VARCHAR(20) NOT NULL COMMENT 'GIS 인증을 위한 거주 동',
  `building_ho` VARCHAR(20) NOT NULL COMMENT 'GIS 인증을 위한 거주 호',
  `address_verified` TINYINT(1) NOT NULL DEFAULT 0 COMMENT 'GIS 인증 여부 (0: false, 1: true)',
  `provider` ENUM('LOCAL', 'KAKAO', 'NAVER') NOT NULL COMMENT '가입 경로',
  `created_at` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '가입일시',
  `updated_at` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`user_id`),
  UNIQUE INDEX `email_UNIQUE` (`email` ASC),
  UNIQUE INDEX `nickname_UNIQUE` (`nickname` ASC)
) ENGINE = InnoDB COMMENT = '회원 정보';

-- -----------------------------------------------------
-- 2. 상품 관련 테이블
-- -----------------------------------------------------
CREATE TABLE IF NOT EXISTS `products` (
  `product_id` BIGINT NOT NULL AUTO_INCREMENT COMMENT '상품 고유 ID',
  `seller_id` BIGINT NOT NULL COMMENT '판매자 ID',
  `title` VARCHAR(100) NOT NULL COMMENT '상품명',
  `description` TEXT NOT NULL COMMENT '상품 설명',
  `price` DECIMAL(10,0) NOT NULL COMMENT '가격',
  `status` ENUM('FOR_SALE', 'SOLD') NOT NULL DEFAULT 'FOR_SALE' COMMENT '판매 상태',
  `created_at` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '등록일시',
  `updated_at` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`product_id`),
  INDEX `fk_products_users_idx` (`seller_id` ASC),
  CONSTRAINT `fk_products_users`
    FOREIGN KEY (`seller_id`)
    REFERENCES `users` (`user_id`)
    ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE = InnoDB COMMENT = '판매 상품 정보';

CREATE TABLE IF NOT EXISTS `product_images` (
  `image_id` BIGINT NOT NULL AUTO_INCREMENT COMMENT '이미지 ID',
  `product_id` BIGINT NOT NULL COMMENT '상품 ID',
  `image_url` VARCHAR(255) NOT NULL COMMENT 'S3 이미지 URL',
  PRIMARY KEY (`image_id`),
  INDEX `fk_product_images_products_idx` (`product_id` ASC),
  CONSTRAINT `fk_product_images_products`
    FOREIGN KEY (`product_id`)
    REFERENCES `products` (`product_id`)
    ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE = InnoDB COMMENT = '상품 이미지 정보';

-- -----------------------------------------------------
-- 3. 주문 및 배송 관련 테이블
-- -----------------------------------------------------
CREATE TABLE IF NOT EXISTS `orders` (
  `order_id` BIGINT NOT NULL AUTO_INCREMENT COMMENT '주문 고유 ID',
  `product_id` BIGINT NOT NULL COMMENT '주문된 상품 ID',
  `buyer_id` BIGINT NOT NULL COMMENT '구매자 ID',
  `status` ENUM('PAYMENT_COMPLETED', 'IN_DELIVERY', 'DELIVERY_COMPLETED', 'CANCELLED') NOT NULL COMMENT '주문 상태',
  `created_at` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '주문(결제) 일시',
  `updated_at` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP COMMENT '상태 변경 일시',
  PRIMARY KEY (`order_id`),
  UNIQUE INDEX `product_id_UNIQUE` (`product_id` ASC),
  INDEX `fk_orders_buyers_idx` (`buyer_id` ASC),
  CONSTRAINT `fk_orders_products`
    FOREIGN KEY (`product_id`)
    REFERENCES `products` (`product_id`)
    ON DELETE RESTRICT ON UPDATE CASCADE,
  CONSTRAINT `fk_orders_buyers`
    FOREIGN KEY (`buyer_id`)
    REFERENCES `users` (`user_id`)
    ON DELETE RESTRICT ON UPDATE CASCADE
) ENGINE = InnoDB COMMENT = '주문 및 거래 상태 정보';

CREATE TABLE IF NOT EXISTS `payments` (
  `payment_id` BIGINT NOT NULL AUTO_INCREMENT COMMENT '결제 ID',
  `order_id` BIGINT NOT NULL COMMENT '주문 ID',
  `payment_key` VARCHAR(255) NOT NULL COMMENT 'Toss 결제 고유 키',
  `amount` DECIMAL(10,0) NOT NULL COMMENT '결제 금액',
  `status` ENUM('DONE', 'CANCELED') NOT NULL COMMENT '결제 상태 (완료, 취소)',
  `approved_at` DATETIME NOT NULL COMMENT '결제 승인 일시',
  PRIMARY KEY (`payment_id`),
  UNIQUE INDEX `payment_key_UNIQUE` (`payment_key` ASC),
  INDEX `fk_payments_orders_idx` (`order_id` ASC),
  CONSTRAINT `fk_payments_orders`
    FOREIGN KEY (`order_id`)
    REFERENCES `orders` (`order_id`)
    ON DELETE RESTRICT ON UPDATE CASCADE
) ENGINE = InnoDB COMMENT = 'Toss 결제 정보';

CREATE TABLE IF NOT EXISTS `robots` (
  `robot_id` BIGINT NOT NULL AUTO_INCREMENT COMMENT '로봇 ID',
  `robot_name` VARCHAR(50) NOT NULL COMMENT '로봇 식별 이름',
  `status` ENUM('IDLE', 'IN_DELIVERY', 'MAINTENANCE') NOT NULL DEFAULT 'IDLE' COMMENT '로봇 상태',
  PRIMARY KEY (`robot_id`),
  UNIQUE INDEX `robot_name_UNIQUE` (`robot_name` ASC)
) ENGINE = InnoDB COMMENT = '자율주행 로봇 정보';

CREATE TABLE IF NOT EXISTS `deliveries` (
  `delivery_id` BIGINT NOT NULL AUTO_INCREMENT COMMENT '배송 ID',
  `order_id` BIGINT NOT NULL COMMENT '주문 ID',
  `robot_id` BIGINT NULL COMMENT '배송 담당 로봇 ID',
  `status` ENUM('REQUESTED', 'IN_TRANSIT', 'DELIVERED') NOT NULL COMMENT '배송 상태',
  `request_time` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '판매자가 로봇 호출한 시간',
  `complete_time` DATETIME NULL COMMENT '배송 완료 시간',
  PRIMARY KEY (`delivery_id`),
  UNIQUE INDEX `order_id_UNIQUE` (`order_id` ASC),
  INDEX `fk_deliveries_robots_idx` (`robot_id` ASC),
  CONSTRAINT `fk_deliveries_orders`
    FOREIGN KEY (`order_id`)
    REFERENCES `orders` (`order_id`)
    ON DELETE CASCADE ON UPDATE CASCADE,
  CONSTRAINT `fk_deliveries_robots`
    FOREIGN KEY (`robot_id`)
    REFERENCES `robots` (`robot_id`)
    ON DELETE SET NULL ON UPDATE CASCADE
) ENGINE = InnoDB COMMENT = '로봇 배송 정보';

-- -----------------------------------------------------
-- 4. 커뮤니케이션 관련 테이블
-- -----------------------------------------------------
CREATE TABLE IF NOT EXISTS `chat_rooms` (
  `room_id` BIGINT NOT NULL AUTO_INCREMENT COMMENT '채팅방 ID',
  `product_id` BIGINT NOT NULL COMMENT '관련 상품 ID',
  `buyer_id` BIGINT NOT NULL COMMENT '구매 희망자 ID',
  `created_at` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`room_id`),
  INDEX `fk_chat_rooms_products_idx` (`product_id` ASC),
  INDEX `fk_chat_rooms_users_idx` (`buyer_id` ASC),
  CONSTRAINT `fk_chat_rooms_products`
    FOREIGN KEY (`product_id`)
    REFERENCES `products` (`product_id`)
    ON DELETE CASCADE ON UPDATE CASCADE,
  CONSTRAINT `fk_chat_rooms_users`
    FOREIGN KEY (`buyer_id`)
    REFERENCES `users` (`user_id`)
    ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE = InnoDB COMMENT = '채팅방 정보';

CREATE TABLE IF NOT EXISTS `chat_messages` (
  `message_id` BIGINT NOT NULL AUTO_INCREMENT COMMENT '메시지 ID',
  `room_id` BIGINT NOT NULL COMMENT 'c채팅방 ID',
  `sender_id` BIGINT NOT NULL COMMENT '발신자 ID',
  `message` TEXT NOT NULL COMMENT '메시지 내용',
  `sent_at` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '전송 시간',
  PRIMARY KEY (`message_id`),
  INDEX `fk_chat_messages_rooms_idx` (`room_id` ASC),
  INDEX `fk_chat_messages_users_idx` (`sender_id` ASC),
  CONSTRAINT `fk_chat_messages_rooms`
    FOREIGN KEY (`room_id`)
    REFERENCES `chat_rooms` (`room_id`)
    ON DELETE CASCADE ON UPDATE CASCADE,
  CONSTRAINT `fk_chat_messages_users`
    FOREIGN KEY (`sender_id`)
    REFERENCES `users` (`user_id`)
    ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE = InnoDB COMMENT = '채팅 메시지';

CREATE TABLE IF NOT EXISTS `notifications` (
  `notification_id` BIGINT NOT NULL AUTO_INCREMENT COMMENT '알림 고유 ID',
  `user_id` BIGINT NOT NULL COMMENT '알림 수신자 ID',
  `order_id` BIGINT NULL COMMENT '관련 주문 ID',
  `notification_type` ENUM('PAYMENT_COMPLETED', 'DELIVERY_STARTED', 'DELIVERY_COMPLETED', 'ORDER_CANCELLED', 'NEW_MESSAGE', 'SYSTEM') NOT NULL COMMENT '알림 종류',
  `content` VARCHAR(255) NOT NULL COMMENT '알림 내용',
  `is_read` TINYINT(1) NOT NULL DEFAULT 0 COMMENT '읽음 여부 (0: false, 1: true)',
  `created_at` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '알림 발생 시간',
  PRIMARY KEY (`notification_id`),
  INDEX `fk_notifications_users_idx` (`user_id` ASC),
  INDEX `fk_notifications_orders_idx` (`order_id` ASC),
  CONSTRAINT `fk_notifications_users`
    FOREIGN KEY (`user_id`)
    REFERENCES `users` (`user_id`)
    ON DELETE CASCADE ON UPDATE CASCADE,
  CONSTRAINT `fk_notifications_orders`
    FOREIGN KEY (`order_id`)
    REFERENCES `orders` (`order_id`)
    ON DELETE SET NULL ON UPDATE CASCADE
) ENGINE = InnoDB COMMENT = '실시간 알림 정보';