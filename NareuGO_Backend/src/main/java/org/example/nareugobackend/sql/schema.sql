-----------------------------------------------------
-- 1. 회원 관련 테이블
-- -----------------------------------------------------
CREATE TABLE `user` (
                        `id` bigint NOT NULL AUTO_INCREMENT,
                        `birth` varchar(255) DEFAULT NULL,
                        `deleted_at` datetime(6) DEFAULT NULL,
                        `email` varchar(255) NOT NULL,
                        `is_active` bit(1) DEFAULT NULL,
                        `name` varchar(50) DEFAULT NULL,
                        `phone_number` varchar(255) DEFAULT NULL,
                        `provider_id` varchar(255) DEFAULT NULL,
                        `provider_type` enum('KAKAO','NAVER') DEFAULT NULL,
                        `role` enum('SELLER','USER') DEFAULT NULL,
                        `sex` enum('MAN','WOMAN') DEFAULT NULL,
                        `apartment_name` varchar(255) DEFAULT NULL,
                        `building_dong` int DEFAULT NULL,
                        `building_ho` int DEFAULT NULL,
                        `eup_myeon_dong` varchar(255) DEFAULT NULL,
                        `si_do` varchar(255) DEFAULT NULL,
                        `si_gun_gu` varchar(255) DEFAULT NULL,
                        `nickname` varchar(255) DEFAULT NULL,
                        PRIMARY KEY (`id`),
                        UNIQUE KEY `UKob8kqyqqgmefl0aco34akdtpe` (`email`)
) ENGINE=InnoDB AUTO_INCREMENT=7 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci

-- -----------------------------------------------------
-- 2. 상품 관련 테이블
-- -----------------------------------------------------
CREATE TABLE `products` (
                            `product_id` bigint NOT NULL AUTO_INCREMENT COMMENT '상품 고유 ID',
                            `seller_id` bigint NOT NULL COMMENT '판매자 ID',
                            `title` varchar(100) NOT NULL COMMENT '상품명',
                            `description` text NOT NULL COMMENT '상품 설명',
                            `price` decimal(10,0) NOT NULL COMMENT '가격',
                            `status` enum('FOR_SALE','SOLD') NOT NULL DEFAULT 'FOR_SALE' COMMENT '판매 상태',
                            `created_at` datetime NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '등록일시',
                            `updated_at` datetime NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
                            `apartment_name` varchar(100) NOT NULL COMMENT '아파트 이름',
                            `si_do` varchar(50) NOT NULL COMMENT '시/도',
                            `si_gun_gu` varchar(50) NOT NULL COMMENT '시/군/구',
                            `eup_myeon_dong` varchar(50) NOT NULL COMMENT '읍/면/동',
                            PRIMARY KEY (`product_id`),
                            KEY `fk_products_users_idx` (`seller_id`),
                            KEY `idx_region` (`si_do`,`si_gun_gu`,`eup_myeon_dong`,`apartment_name`),
                            CONSTRAINT `fk_products_users` FOREIGN KEY (`seller_id`) REFERENCES `users` (`user_id`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB AUTO_INCREMENT=8 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='판매 상품 정보'



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


-- orders 테이블 수정 필요
ALTER TABLE orders 
ADD COLUMN amount DECIMAL(10,0) NOT NULL COMMENT '주문 금액';

-- orders 테이블 status ENUM 수정 필요  
ALTER TABLE orders 
MODIFY COLUMN status ENUM('PAYMENT_PENDING', 'PAYMENT_COMPLETED', 'IN_DELIVERY', 'DELIVERY_COMPLETED', 'CANCELLED') NOT NULL;



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

-- -----------------------------------------------------
-- 5. 마이페이지 관련 테이블
-- -----------------------------------------------------

-- 사용자 좋아요(관심목록) 테이블
CREATE TABLE IF NOT EXISTS `user_favorites` (
    `favorite_id` BIGINT NOT NULL AUTO_INCREMENT COMMENT '좋아요 ID',
    `user_id` BIGINT NOT NULL COMMENT '사용자 ID',
    `product_id` BIGINT NOT NULL COMMENT '상품 ID',
    `created_at` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '좋아요 등록 시간',
    PRIMARY KEY (`favorite_id`),
    UNIQUE INDEX `unique_user_product` (`user_id` ASC, `product_id` ASC),
    INDEX `fk_user_favorites_users_idx` (`user_id` ASC),
    INDEX `fk_user_favorites_products_idx` (`product_id` ASC),
    CONSTRAINT `fk_user_favorites_users`
        FOREIGN KEY (`user_id`)
        REFERENCES `users` (`user_id`)
        ON DELETE CASCADE ON UPDATE CASCADE,
    CONSTRAINT `fk_user_favorites_products`
        FOREIGN KEY (`product_id`)
        REFERENCES `products` (`product_id`)
        ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE = InnoDB COMMENT = '사용자 관심목록(좋아요)';

-- 사용자 상품 조회 이력 테이블
CREATE TABLE IF NOT EXISTS `user_product_history` (
    `history_id` BIGINT NOT NULL AUTO_INCREMENT COMMENT '이력 ID',
    `user_id` BIGINT NOT NULL COMMENT '사용자 ID',
    `product_id` BIGINT NOT NULL COMMENT '상품 ID',
    `viewed_at` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '조회 시간',
    PRIMARY KEY (`history_id`),
    INDEX `fk_user_product_history_users_idx` (`user_id` ASC),
    INDEX `fk_user_product_history_products_idx` (`product_id` ASC),
    INDEX `idx_user_viewed_at` (`user_id` ASC, `viewed_at` DESC),
    CONSTRAINT `fk_user_product_history_users`
        FOREIGN KEY (`user_id`)
        REFERENCES `users` (`user_id`)
        ON DELETE CASCADE ON UPDATE CASCADE,
    CONSTRAINT `fk_user_product_history_products`
        FOREIGN KEY (`product_id`)
        REFERENCES `products` (`product_id`)
        ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE = InnoDB COMMENT = '사용자 상품 조회 이력';

-- 동네 인증 테이블
CREATE TABLE IF NOT EXISTS `neighborhood_verification` (
    `verification_id` BIGINT NOT NULL AUTO_INCREMENT COMMENT '인증 ID',
    `user_id` BIGINT NOT NULL COMMENT '사용자 ID',
    `verification_type` ENUM('PHOTO', 'DOCUMENT', 'MANUAL') NOT NULL COMMENT '인증 방식',
    `verification_data` TEXT COMMENT '인증 데이터 (사진 URL, 문서 정보 등)',
    `status` ENUM('PENDING', 'APPROVED', 'REJECTED') NOT NULL DEFAULT 'PENDING' COMMENT '인증 상태',
    `submitted_at` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '제출 시간',
    `processed_at` DATETIME NULL COMMENT '처리 시간',
    `admin_notes` TEXT COMMENT '관리자 메모',
    PRIMARY KEY (`verification_id`),
    INDEX `fk_neighborhood_verification_users_idx` (`user_id` ASC),
    CONSTRAINT `fk_neighborhood_verification_users`
        FOREIGN KEY (`user_id`)
        REFERENCES `users` (`user_id`)
        ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE = InnoDB COMMENT = '동네 인증 정보';