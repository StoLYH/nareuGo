-----------------------------------------------------
-- 1. 회원 관련 테이블
-- -----------------------------------------------------
CREATE TABLE `users` (
                         `user_id` bigint NOT NULL AUTO_INCREMENT,
                         `birth` varchar(255) DEFAULT NULL,
                         `deleted_at` datetime(6) DEFAULT NULL,
                         `email` varchar(255) NOT NULL,
                         `is_active` bit(1) DEFAULT NULL,
                         `name` varchar(255) NOT NULL,
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
                         `address_verified` tinyint(1) DEFAULT '0' COMMENT '주소 인증 여부',
                         `verification_date` datetime DEFAULT NULL COMMENT '인증 완료 날짜',
                         PRIMARY KEY (`user_id`),
                         UNIQUE KEY `UKob8kqyqqgmefl0aco34akdtpe` (`email`),
                         UNIQUE KEY `UK6dotkott2kjsp8vw4d0m25fb7` (`email`),
                         KEY `idx_user_address_verified` (`address_verified`)
) ENGINE=InnoDB AUTO_INCREMENT=7 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci

-- -----------------------------------------------------
-- 2. 상품 관련 테이블
-- -----------------------------------------------------
CREATE TABLE `products` (
                            `product_id` bigint NOT NULL AUTO_INCREMENT COMMENT '상품 고유 ID',
                            `seller_id` bigint NOT NULL COMMENT '판매자 ID',
                            `title` varchar(255) DEFAULT NULL,
                            `description` varchar(255) DEFAULT NULL,
                            `price` decimal(38,2) DEFAULT NULL,
                            `status` enum('FOR_SALE','SOLD') NOT NULL DEFAULT 'FOR_SALE' COMMENT '판매 상태',
                            `created_at` datetime NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '등록일시',
                            `updated_at` datetime NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
                            `apartment_name` varchar(255) DEFAULT NULL,
                            `si_do` varchar(50) NOT NULL COMMENT '시/도',
                            `si_gun_gu` varchar(50) NOT NULL COMMENT '시/군/구',
                            `eup_myeon_dong` varchar(255) DEFAULT NULL,
                            PRIMARY KEY (`product_id`),
                            KEY `fk_products_users_idx` (`seller_id`),
                            KEY `idx_region` (`si_do`,`si_gun_gu`,`eup_myeon_dong`,`apartment_name`),
                            CONSTRAINT `FKbgw3lyxhsml3kfqnfr45o0vbj` FOREIGN KEY (`seller_id`) REFERENCES `users` (`user_id`)
) ENGINE=InnoDB AUTO_INCREMENT=32 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='판매 상품 정보'


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
CREATE TABLE `orders` (
                          `order_id` bigint NOT NULL AUTO_INCREMENT COMMENT '주문 고유 ID',
                          `product_id` bigint NOT NULL COMMENT '주문된 상품 ID',
                          `buyer_id` bigint NOT NULL COMMENT '구매자 ID',
                          `status` varchar(20) DEFAULT NULL,
                          `created_at` datetime NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '주문(결제) 일시',
                          `updated_at` datetime NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP COMMENT '상태 변경 일시',
                          `amount` decimal(38,2) DEFAULT NULL,
                          `toss_order_id` varchar(64) DEFAULT NULL COMMENT '토스페이먼츠용 orderId (영문+숫자+특수문자, 6-64자)',
                          PRIMARY KEY (`order_id`),
                          UNIQUE KEY `product_id_UNIQUE` (`product_id`),
                          KEY `fk_orders_buyers_idx` (`buyer_id`),
                          KEY `idx_orders_toss_order_id` (`toss_order_id`),
                          CONSTRAINT `fk_orders_products` FOREIGN KEY (`product_id`) REFERENCES `products` (`product_id`) ON DELETE RESTRICT ON UPDATE CASCADE,
                          CONSTRAINT `FKhtx3insd5ge6w486omk4fnk54` FOREIGN KEY (`buyer_id`) REFERENCES `users` (`user_id`)
) ENGINE=InnoDB AUTO_INCREMENT=68 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='주문 및 거래 상태 정보'


CREATE TABLE `payments` (
                            `payment_id` bigint NOT NULL AUTO_INCREMENT COMMENT '결제 ID',
                            `order_id` bigint NOT NULL COMMENT '주문 ID',
                            `payment_key` varchar(255) NOT NULL COMMENT 'Toss 결제 고유 키',
                            `amount` decimal(10,0) NOT NULL COMMENT '결제 금액',
                            `status` enum('DONE','CANCELED') NOT NULL COMMENT '결제 상태 (완료, 취소)',
                            `approved_at` datetime NOT NULL COMMENT '결제 승인 일시',
                            PRIMARY KEY (`payment_id`),
                            UNIQUE KEY `payment_key_UNIQUE` (`payment_key`),
                            KEY `fk_payments_orders_idx` (`order_id`),
                            CONSTRAINT `fk_payments_orders` FOREIGN KEY (`order_id`) REFERENCES `orders` (`order_id`) ON DELETE RESTRICT ON UPDATE CASCADE
) ENGINE=InnoDB AUTO_INCREMENT=15 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='Toss 결제 정보'


CREATE TABLE `robots` (
                          `robot_id` bigint NOT NULL AUTO_INCREMENT COMMENT '로봇 ID',
                          `robot_name` varchar(50) NOT NULL COMMENT '로봇 식별 이름',
                          `status` varchar(10) DEFAULT NULL,
                          PRIMARY KEY (`robot_id`),
                          UNIQUE KEY `robot_name_UNIQUE` (`robot_name`)
) ENGINE=InnoDB AUTO_INCREMENT=2 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='자율주행 로봇 정보'

CREATE TABLE `deliveries` (
                              `actual_delivery_time` datetime(6) DEFAULT NULL,
                              `created_at` datetime(6) DEFAULT NULL,
                              `delivery_id` bigint NOT NULL AUTO_INCREMENT,
                              `estimated_delivery_time` datetime(6) DEFAULT NULL,
                              `order_id` bigint DEFAULT NULL,
                              `updated_at` datetime(6) DEFAULT NULL,
                              `delivery_address` varchar(255) DEFAULT NULL,
                              `tracking_number` varchar(255) DEFAULT NULL,
                              `status` varchar(20) DEFAULT NULL,
                              PRIMARY KEY (`delivery_id`),
                              UNIQUE KEY `UKk36n9p5v7dd96hpgkwybvbogt` (`order_id`),
                              CONSTRAINT `FK7isx0rnbgqr1dcofd5putl6jw` FOREIGN KEY (`order_id`) REFERENCES `orders` (`order_id`)
) ENGINE=InnoDB AUTO_INCREMENT=30 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci

-- -----------------------------------------------------
-- 4. 커뮤니케이션 관련 테이블 (WebSocket 채팅용)
-- -----------------------------------------------------
CREATE TABLE `chat_rooms` (
                              `room_id` bigint NOT NULL AUTO_INCREMENT COMMENT '채팅방 ID',
                              `user1_id` varchar(50) NOT NULL COMMENT '사용자1 ID',
                              `user2_id` varchar(50) NOT NULL COMMENT '사용자2 ID',
                              `created_at` varchar(255) NOT NULL COMMENT '생성 시간',
                              `last_message_at` varchar(255) DEFAULT NULL COMMENT '마지막 메시지 시간',
                              `last_message` text COMMENT '마지막 메시지 내용',
                              `product_id` bigint NOT NULL COMMENT '상품 ID',
                              PRIMARY KEY (`room_id`),
                              UNIQUE KEY `unique_users_product` (`user1_id`,`user2_id`,`product_id`),
                              KEY `idx_user1` (`user1_id`),
                              KEY `idx_user2` (`user2_id`),
                              KEY `idx_product` (`product_id`)
) ENGINE=InnoDB AUTO_INCREMENT=16 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='WebSocket 채팅방 정보'

CREATE TABLE `chat_messages` (
                                 `message_id` bigint NOT NULL AUTO_INCREMENT COMMENT '메시지 ID',
                                 `room_id` bigint NOT NULL COMMENT '채팅방 ID',
                                 `sender_id` varchar(50) NOT NULL COMMENT '발신자 ID',
                                 `receiver_id` varchar(50) NOT NULL COMMENT '수신자 ID',
                                 `content` text NOT NULL COMMENT '메시지 내용',
                                 `timestamp` varchar(255) NOT NULL COMMENT '전송 시간',
                                 PRIMARY KEY (`message_id`),
                                 KEY `fk_chat_messages_rooms_idx` (`room_id`),
                                 KEY `idx_sender` (`sender_id`),
                                 KEY `idx_receiver` (`receiver_id`),
                                 KEY `idx_timestamp` (`timestamp`),
                                 CONSTRAINT `fk_chat_messages_rooms` FOREIGN KEY (`room_id`) REFERENCES `chat_rooms` (`room_id`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB AUTO_INCREMENT=41 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='WebSocket 채팅 메시지'

CREATE TABLE `notifications` (
                                 `notification_id` bigint NOT NULL AUTO_INCREMENT COMMENT '알림 고유 ID',
                                 `user_id` bigint NOT NULL COMMENT '알림 수신자 ID',
                                 `order_id` bigint DEFAULT NULL COMMENT '관련 주문 ID',
                                 `notification_type` enum('PAYMENT_COMPLETED','DELIVERY_STARTED','DELIVERY_COMPLETED','ORDER_CANCELLED','NEW_MESSAGE','SYSTEM') NOT NULL COMMENT '알림 종류',
                                 `content` varchar(255) NOT NULL COMMENT '알림 내용',
                                 `is_read` tinyint(1) NOT NULL DEFAULT '0' COMMENT '읽음 여부 (0: false, 1: true)',
                                 `created_at` datetime NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '알림 발생 시간',
                                 PRIMARY KEY (`notification_id`),
                                 KEY `fk_notifications_users_idx` (`user_id`),
                                 KEY `fk_notifications_orders_idx` (`order_id`),
                                 CONSTRAINT `fk_notifications_orders` FOREIGN KEY (`order_id`) REFERENCES `orders` (`order_id`) ON DELETE SET NULL ON UPDATE CASCADE,
                                 CONSTRAINT `fk_notifications_users` FOREIGN KEY (`user_id`) REFERENCES `users` (`user_id`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='실시간 알림 정보'
-- -----------------------------------------------------
-- 5. 마이페이지 관련 테이블
-- -----------------------------------------------------
CREATE TABLE `user_favorites` (
                                  `created_at` datetime(6) NOT NULL,
                                  `favorite_id` bigint NOT NULL AUTO_INCREMENT,
                                  `product_id` bigint NOT NULL,
                                  `user_id` bigint NOT NULL,
                                  PRIMARY KEY (`favorite_id`),
                                  UNIQUE KEY `unique_user_product` (`user_id`,`product_id`),
                                  CONSTRAINT `FK4sv7b9w9adr0fjnc4u10exlwm` FOREIGN KEY (`user_id`) REFERENCES `users` (`user_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci

CREATE TABLE `user_product_history` (
                                        `history_id` bigint NOT NULL AUTO_INCREMENT,
                                        `product_id` bigint NOT NULL,
                                        `user_id` bigint NOT NULL,
                                        `viewed_at` datetime(6) NOT NULL,
                                        PRIMARY KEY (`history_id`),
                                        KEY `FK4k7ro4pr6ym5ebvbxc5u47ep5` (`user_id`),
                                        CONSTRAINT `FK4k7ro4pr6ym5ebvbxc5u47ep5` FOREIGN KEY (`user_id`) REFERENCES `users` (`user_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci


-- 동네 인증 테이블
CREATE TABLE `neighborhood_verification` (
                                             `processed_at` datetime(6) DEFAULT NULL,
                                             `submitted_at` datetime(6) NOT NULL,
                                             `user_id` bigint NOT NULL,
                                             `verification_id` bigint NOT NULL AUTO_INCREMENT,
                                             `admin_notes` text,
                                             `verification_data` text,
                                             `status` enum('APPROVED','PENDING','REJECTED') NOT NULL,
                                             `verification_type` enum('DOCUMENT','MANUAL','PHOTO') NOT NULL,
                                             PRIMARY KEY (`verification_id`),
                                             KEY `FK2io3hrk0nnr98jg7nxf2b9shy` (`user_id`),
                                             CONSTRAINT `FK2io3hrk0nnr98jg7nxf2b9shy` FOREIGN KEY (`user_id`) REFERENCES `users` (`user_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci