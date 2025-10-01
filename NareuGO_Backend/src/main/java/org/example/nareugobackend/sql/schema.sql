-- MySQL dump 10.13  Distrib 8.0.41, for Win64 (x86_64)
--
-- Host: j13a501.p.ssafy.io    Database: a501
-- ------------------------------------------------------
-- Server version	9.4.0

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!50503 SET NAMES utf8 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `chat_messages`
--

DROP TABLE IF EXISTS `chat_messages`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `chat_messages` (
                                 `message_id` bigint NOT NULL AUTO_INCREMENT COMMENT '메시지 ID',
                                 `room_id` bigint NOT NULL COMMENT '채팅방 ID',
                                 `sender_id` varchar(50) NOT NULL COMMENT '발신자 ID',
                                 `receiver_id` varchar(50) NOT NULL COMMENT '수신자 ID',
                                 `content` text NOT NULL,
                                 `timestamp` varchar(255) NOT NULL COMMENT '전송 시간',
                                 PRIMARY KEY (`message_id`),
                                 KEY `fk_chat_messages_rooms_idx` (`room_id`),
                                 CONSTRAINT `fk_chat_messages_rooms` FOREIGN KEY (`room_id`) REFERENCES `chat_rooms` (`room_id`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB AUTO_INCREMENT=44 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='WebSocket 채팅 메시지';
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `chat_messages`
--

LOCK TABLES `chat_messages` WRITE;
/*!40000 ALTER TABLE `chat_messages` DISABLE KEYS */;
INSERT INTO `chat_messages` VALUES (3,2,'2','3','1','2025-09-22T06:01:19.370729168'),(4,3,'1','3','df','2025-09-22T15:03:15.935961'),(5,4,'2','1','1','2025-09-22T15:21:12.4062194'),(6,4,'2','1','2','2025-09-22T06:29:38.060169328'),(7,4,'2','1','3','2025-09-22T06:29:39.082727202'),(8,5,'2','1','1','2025-09-22T06:29:59.708583265'),(9,5,'2','1','2','2025-09-22T06:29:59.97836711'),(10,5,'2','1','3','2025-09-22T06:30:00.261267376'),(11,5,'2','1','999','2025-09-22T06:30:02.214741716'),(12,6,'1','2','ㅎㅇ','2025-09-22T15:30:45.5011264'),(13,2,'2','3','test','2025-09-23T14:09:47.7328212'),(14,10,'1','3','asda\\','2025-09-23T16:01:33.7901896'),(15,10,'3','1','ef','2025-09-23T17:09:55.3342827'),(16,10,'3','1','hi','2025-09-23T17:09:57.2284706'),(17,10,'3','1','hih','2025-09-23T17:09:57.5076186'),(18,10,'3','1','i','2025-09-23T17:09:57.6720279'),(19,10,'3','1','hih','2025-09-23T17:09:57.9806628'),(20,10,'1','3','dfd','2025-09-23T17:11:21.2161595'),(21,10,'1','3','안녕','2025-09-23T17:11:32.1092966'),(22,10,'1','3','ㅎㅇ','2025-09-23T17:15:12.1288399'),(23,10,'1','3','ㅁㄴㄷㄹㄷㄴ','2025-09-23T17:15:23.1581318'),(24,10,'1','3','ㄷㄹㄷ','2025-09-23T17:15:42.9304073'),(25,10,'1','3','ㄷㄹㄷㄹㄷ','2025-09-23T17:15:45.7495561'),(26,10,'1','3','1123','2025-09-23T17:15:47.0936845'),(27,10,'1','3','ㅂㅈㄷ','2025-09-23T17:16:02.5168761'),(28,10,'1','3','ㄴㅇㄹ','2025-09-23T17:18:37.9192465'),(29,10,'1','3','ㅁㄷㄴㄹ','2025-09-23T17:18:39.8599093'),(30,10,'1','3','ㅍㄹ호','2025-09-23T17:18:49.0499729'),(31,10,'1','3','ㄴㄹ소','2025-09-23T17:18:50.9890229'),(32,10,'3','1','df','2025-09-23T17:22:29.0597051'),(33,6,'2','1','ㅎㅇ','2025-09-24T10:42:53.3873538'),(34,5,'1','2','ㅎㅇ','2025-09-24T07:04:56.698334527'),(35,5,'2','2','qq','2025-09-24T07:05:35.841721357'),(36,5,'2','1','gd','2025-09-24T07:05:39.109410272'),(37,5,'2','1','gd','2025-09-24T07:05:44.803993253'),(38,5,'2','2','11','2025-09-24T07:05:46.617203735'),(39,5,'2','1','??','2025-09-24T07:05:51.742729874'),(40,5,'1','2','dd','2025-09-24T07:06:36.558735748'),(41,5,'2','1','??','2025-09-24T07:06:39.147902353'),(42,5,'2','1','dd','2025-09-24T07:06:41.513487088'),(43,5,'1','2','11111','2025-09-24T07:06:43.351312631');
/*!40000 ALTER TABLE `chat_messages` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `chat_rooms`
--

DROP TABLE IF EXISTS `chat_rooms`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `chat_rooms` (
                              `room_id` bigint NOT NULL AUTO_INCREMENT COMMENT '채팅방 ID',
                              `user1_id` varchar(50) NOT NULL COMMENT '사용자1 ID',
                              `user2_id` varchar(50) NOT NULL COMMENT '사용자2 ID',
                              `created_at` varchar(255) NOT NULL COMMENT '생성 시간',
                              `last_message_at` varchar(255) DEFAULT NULL,
                              `last_message` text,
                              `product_id` bigint NOT NULL,
                              PRIMARY KEY (`room_id`),
                              UNIQUE KEY `unique_users_product` (`user1_id`,`user2_id`,`product_id`),
                              KEY `idx_user1` (`user1_id`),
                              KEY `idx_user2` (`user2_id`),
                              KEY `idx_product` (`product_id`)
) ENGINE=InnoDB AUTO_INCREMENT=15 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='WebSocket 채팅방 정보';
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `chat_rooms`
--

LOCK TABLES `chat_rooms` WRITE;
/*!40000 ALTER TABLE `chat_rooms` DISABLE KEYS */;
INSERT INTO `chat_rooms` VALUES (2,'2','3','2025-09-22T06:01:17.161390024','2025-09-23T14:09:47.7328212','test',10),(3,'1','3','2025-09-22T15:03:13.9983922','2025-09-22T15:03:15.935961','df',10),(4,'2','1','2025-09-22T15:21:09.807427','2025-09-22T06:29:39.082727202','3',2),(5,'2','1','2025-09-22T06:29:58.346473106','2025-09-24T07:06:43.351312631','11111',3),(6,'1','2','2025-09-22T15:30:42.331837','2025-09-24T10:42:53.3873538','ㅎㅇ',15),(7,'2','1','2025-09-22T06:55:58.983885576','2025-09-22T06:55:58.983885576',NULL,14),(8,'3','1','2025-09-23T14:22:39.8017744','2025-09-23T14:22:39.8017744',NULL,14),(9,'3','1','2025-09-23T14:24:46.7243305','2025-09-23T14:24:46.7243305',NULL,4),(10,'1','3','2025-09-23T14:51:54.9621607','2025-09-23T17:22:29.0597051','df',18),(11,'2','3','2025-09-23T15:13:57.3807496','2025-09-23T15:13:57.3807496',NULL,18),(12,'2','1','2025-09-23T07:20:27.571990072','2025-09-23T07:20:27.571990072',NULL,6),(13,'2','1','2025-09-24T01:31:53.502806858','2025-09-24T01:31:53.502806858',NULL,5),(14,'2','1','2025-09-24T10:53:42.1261928','2025-09-24T10:53:42.1261928',NULL,4);
/*!40000 ALTER TABLE `chat_rooms` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `deliveries`
--

DROP TABLE IF EXISTS `deliveries`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
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
) ENGINE=InnoDB AUTO_INCREMENT=26 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `deliveries`
--

LOCK TABLES `deliveries` WRITE;
/*!40000 ALTER TABLE `deliveries` DISABLE KEYS */;
INSERT INTO `deliveries` VALUES ('2025-09-26 09:50:14.443361',NULL,1,NULL,1,'2025-09-26 09:50:14.443367','역삼래미안아파트 101동 402호','207257238323','RECEIPT_COMPLETED');
/*!40000 ALTER TABLE `deliveries` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `fcm_tokens`
--

DROP TABLE IF EXISTS `fcm_tokens`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `fcm_tokens` (
                              `token_id` bigint NOT NULL AUTO_INCREMENT,
                              `created_at` datetime(6) NOT NULL,
                              `device_type` varchar(50) DEFAULT NULL,
                              `is_active` bit(1) DEFAULT NULL,
                              `token` varchar(500) NOT NULL,
                              `updated_at` datetime(6) NOT NULL,
                              `user_id` bigint NOT NULL,
                              PRIMARY KEY (`token_id`),
                              KEY `FKj2kob865pl9dv5vwrs2pmshjv` (`user_id`),
                              CONSTRAINT `FKj2kob865pl9dv5vwrs2pmshjv` FOREIGN KEY (`user_id`) REFERENCES `users` (`user_id`)
) ENGINE=InnoDB AUTO_INCREMENT=19 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `fcm_tokens`
--

LOCK TABLES `fcm_tokens` WRITE;
/*!40000 ALTER TABLE `fcm_tokens` DISABLE KEYS */;
INSERT INTO `fcm_tokens` VALUES (1,'2025-09-23 14:15:57.401157','WEB',_binary '\0','ev92kVnCVtiuCAhvVfyRSv:APA91bEMCeJAi5pHhj4FPJ2fJF7967UUgrk0uDcr-hQJL0RzCs_eXZu26y8t9YMlPxPbURanYWPVtTvZkdPA-0w-udEeKTEAxg6qMCtqVrJd6gjIlwm8F3g','2025-09-25 14:44:31.400780',1),(3,'2025-09-24 11:18:26.043559','web',_binary '\0','cXxXxXxXxXx:APA91bEfGhIjKlMnOpQrStUvWxYzAbCdEfGhIjKlMnOpQrStUvWxYzAbCdEfGhIjKlMnOpQrStUvWxYzAbCdEfGhIjKlMnOpQrStUvWxYzAbCdEfGhIjKlMnOpQrStUvWxYzAbCdEfGhIjKlMnOpQrSt','2025-09-24 11:18:51.805870',3),(4,'2025-09-24 11:23:39.973404','web',_binary '\0','browser_1758680619929_00wlzi33f8ieg','2025-09-24 11:24:08.403257',3),(5,'2025-09-25 23:50:09.880599','WEB',_binary '\0','ckwvW4XsOJHcYTimSywngp:APA91bFH0FmzSuvhtsl_9rkiFguKEwBhmbM1kQqzF0HepwCDAkMNAauIPR4DdCC9leiawrNz4gBN2Uc8x3tt_OyzZ1Bwk0V41sXhxl-fAetKvDixYy1wNOQ','2025-09-26 00:08:44.517658',1),(6,'2025-09-25 23:54:02.708459','WEB',_binary '\0','ckwvW4XsOJHcYTimSywngp:APA91bFNjWd3BZ4kkdgGN9jThkrdUIL4lgIJxqyGeAiEZ3kzdQ3xr1myy3WZZop4zpUd1IGBP7hd_voIpfTaUBNs_iA9z6j2KixK3NzCgLRIFJz-g1T98-Y','2025-09-26 00:08:44.634734',1),(7,'2025-09-25 23:54:53.265008','WEB',_binary '\0','ckwvW4XsOJHcYTimSywngp:APA91bHrSQiuYyRvqlFk6rjIr4ELYIfXssBqdzO4RFL-Zl9LtN1ii_bD8DehwRr7yzj9m_Yp53iB9LXdpcfdDojMpfHAeOMaKmklUr6ExzEi_Cbo4Kepy14','2025-09-26 00:08:44.752612',1),(8,'2025-09-26 00:00:02.893379','WEB',_binary '\0','ckwvW4XsOJHcYTimSywngp:APA91bGm8FnYO85BmQB23Lck8prGIlOORS8b-OCA6U5JvHzUxB7jKAsqGhzPfbk39YmiRCOy0UuT8qhIdsmfn0eQ56YsrSg6bABHbs5wqiodRwTgVlb2pEg','2025-09-26 00:08:44.865066',1),(9,'2025-09-26 00:00:39.281058','WEB',_binary '\0','ckwvW4XsOJHcYTimSywngp:APA91bHBTSco8Npad5kgaOYRNmcPyiouaPLP2E66NsNk0drE_QjeNSnN0yG2f_oVjFghpUlh8K0smp8k0zaMYppHdr_S7lPcV5p12EYedBEz0JX7PGdvusg','2025-09-26 00:01:21.663637',3),(10,'2025-09-26 00:00:46.879085','WEB',_binary '\0','ckwvW4XsOJHcYTimSywngp:APA91bFY8l0mbIiK_4uMYU16clViWxIaxPwyGN-jINpvbnBcESbKLDsDVAtIMbCadVw4N3I5uiLJ3IDK91YXfZGGzqdgzPueJ_MiZbv2jXjSNez93Kbpxt0','2025-09-26 00:05:50.808203',3),(11,'2025-09-26 00:04:44.908223','WEB',_binary '\0','ckwvW4XsOJHcYTimSywngp:APA91bHaf9ZoQfynbEqd5Q9RjrWX1UVjJV_BK3RzdYF2CUpV1JUGJ2wk5qSq__k7-xWKUBhVwGT5-QUkNGadULvrnuN3Bkg2OAXj1EpasFrxt4UCL0vIhl0','2025-09-26 00:05:51.157838',3),(12,'2025-09-26 00:05:29.668807','WEB',_binary '\0','ckwvW4XsOJHcYTimSywngp:APA91bFUE5-usKFW6ZRXdWOBLkQzvJVcAPPmzIe3ckpLKI-m2GM0Ceb6Lm-8PLmVw0hyQaLrNXjcXZy_MQQ_SjfKiaGMGPo6seM7o8vXsvwX7e4Y2uqI6MA','2025-09-26 00:05:51.462003',3),(13,'2025-09-26 00:06:46.941769','WEB',_binary '\0','ckwvW4XsOJHcYTimSywngp:APA91bF-mCK3nOasrSVUB7pxMHuXOqOgAD_N4ult1y-P0aoUiVS2WhHTwS_VcAlyXJuf2wVgtXx7tcMYBRPKJFHNrzM4sJQ8-v6-EmHZFJVkK8KYJwjLnDY','2025-09-26 08:56:34.598037',3),(14,'2025-09-26 00:17:39.766295','WEB',_binary '\0','ccLqZeTcNFtaDQumjPJe-L:APA91bH_GRwXSE_px8gyUyVPh6Azi3Kuu9wIcpaaJ2M4NtfRO55gI5Nc9Q28u4RAtMxanPuYkV2CMJATngDAjtsO-arOFgAvy4rxx6WryAexCKzwEKyj7e4','2025-09-26 00:38:21.111273',1),(15,'2025-09-26 00:37:16.594429','WEB',_binary '\0','ccLqZeTcNFtaDQumjPJe-L:APA91bHUegN6xYD8qKjJF-6vEeh780GtLto9sbKvMmc83g18q047um9bNDBlgKwwwfDwsfiyKNMbvfz0BxHl-E8Na-mC5GdgsP41ZKSyC07R8qujKaokcrQ','2025-09-26 00:38:21.235259',1),(16,'2025-09-26 00:37:22.760665','WEB',_binary '\0','ccLqZeTcNFtaDQumjPJe-L:APA91bFGdXPzEEK7ja8evKoXhBZztLOa2sTPkWhI9uqOsIEqhKh2taZCbcSdl1SAYQcaBw__EelvMhxW3Mh4NL6VQ7bm567TLcPyCdwjHhWDG5RaRH-UPCI','2025-09-26 08:57:32.603159',1),(17,'2025-09-26 08:55:00.217593','WEB',_binary '','ccLqZeTcNFtaDQumjPJe-L:APA91bFSg5cq41a9GGno_J9GukXMuaPnEcn3u4s7G6WlgXlTX1xquM-q4cmAQUzP5UdDcIE2wwN0AjiwIQTyPdlSrzLVtooTiHi0cHe2MZjUss-WDWl3vaA','2025-09-26 08:55:00.217608',1),(18,'2025-09-26 08:55:06.629187','WEB',_binary '','ckwvW4XsOJHcYTimSywngp:APA91bH2YBASF06mWZjy7QfeCj35mzR5J0EzLT0bsX3-HSfYJsEGYd9zvo2xWhBkxAkwAYotdpjldcSnr6NfmxDqeJdQ8JxQJB3snEA2FD760wLdYF_POLk','2025-09-26 08:55:06.629197',3);
/*!40000 ALTER TABLE `fcm_tokens` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `neighborhood_verification`
--

DROP TABLE IF EXISTS `neighborhood_verification`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
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
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `neighborhood_verification`
--

LOCK TABLES `neighborhood_verification` WRITE;
/*!40000 ALTER TABLE `neighborhood_verification` DISABLE KEYS */;
/*!40000 ALTER TABLE `neighborhood_verification` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `notifications`
--

DROP TABLE IF EXISTS `notifications`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `notifications` (
                                 `id` bigint NOT NULL AUTO_INCREMENT,
                                 `content` varchar(1000) DEFAULT NULL,
                                 `created_at` datetime(6) NOT NULL,
                                 `is_read` bit(1) NOT NULL,
                                 `message` varchar(1000) NOT NULL,
                                 `read_at` datetime(6) DEFAULT NULL,
                                 `title` varchar(255) NOT NULL,
                                 `type` varchar(50) NOT NULL,
                                 `user_id` bigint NOT NULL,
                                 PRIMARY KEY (`id`),
                                 KEY `FK9y21adhxn0ayjhfocscqox7bh` (`user_id`),
                                 CONSTRAINT `FK9y21adhxn0ayjhfocscqox7bh` FOREIGN KEY (`user_id`) REFERENCES `users` (`user_id`)
) ENGINE=InnoDB AUTO_INCREMENT=91 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `notifications`
--

LOCK TABLES `notifications` WRITE;
/*!40000 ALTER TABLE `notifications` DISABLE KEYS */;
INSERT INTO `notifications` VALUES (1,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 11:16:22.184177',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(2,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 11:18:50.968406',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(3,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 11:24:08.107416',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(4,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 12:24:20.256658',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(5,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 12:24:57.084629',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(6,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 12:25:07.499646',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(7,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 12:28:14.795636',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(8,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 12:29:04.624949',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(9,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 12:45:42.846928',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(10,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 12:45:49.768701',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(11,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 12:45:57.355399',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(12,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 12:46:21.435189',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(13,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 12:48:30.207903',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(14,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 12:49:10.269816',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(15,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 12:57:21.595633',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(16,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 12:57:31.268184',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(17,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 12:58:08.377254',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(18,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 12:59:40.001578',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(19,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 12:59:45.888643',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(20,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 13:16:45.568577',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(21,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 13:27:39.058638',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(22,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 13:39:39.441418',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(23,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 14:51:40.866858',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(24,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 15:57:22.447490',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(25,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-24 15:57:36.983733',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(26,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 12:09:33.384905',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(27,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 12:19:49.555359',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(28,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 12:24:25.476082',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(29,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 12:25:07.606204',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(30,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 12:25:35.619664',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(31,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 12:25:53.284095',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(32,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 12:29:20.455928',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(33,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 12:50:31.070823',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(34,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 12:56:44.991719',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(35,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 13:17:18.027414',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(36,'\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호','2025-09-25 13:20:25.202640',_binary '\0','\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호',NULL,'나르고가 도착했습니다!','BUYER_ARRIVAL',1),(37,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 13:47:02.703893',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(38,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 13:53:29.803925',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(39,'\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호','2025-09-25 13:53:44.252705',_binary '\0','\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호',NULL,'나르고가 도착했습니다!','BUYER_ARRIVAL',1),(40,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 14:03:45.662935',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(41,'\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호','2025-09-25 14:03:59.801173',_binary '\0','\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호',NULL,'나르고가 도착했습니다!','BUYER_ARRIVAL',1),(42,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 14:27:15.068584',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(43,'\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호','2025-09-25 14:27:34.847994',_binary '\0','\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호',NULL,'나르고가 도착했습니다!','BUYER_ARRIVAL',1),(44,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 14:44:14.898091',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(45,'\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호','2025-09-25 14:44:30.816588',_binary '\0','\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호',NULL,'나르고가 도착했습니다!','BUYER_ARRIVAL',1),(46,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 14:49:43.120084',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(47,'\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호','2025-09-25 14:50:09.819375',_binary '\0','\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호',NULL,'나르고가 도착했습니다!','BUYER_ARRIVAL',1),(48,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 15:29:56.284876',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(49,'\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호','2025-09-25 15:30:08.332218',_binary '\0','\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호',NULL,'나르고가 도착했습니다!','BUYER_ARRIVAL',1),(50,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 15:31:40.813830',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(51,'\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호','2025-09-25 15:31:53.795189',_binary '\0','\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호',NULL,'나르고가 도착했습니다!','BUYER_ARRIVAL',1),(52,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 17:15:28.899548',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(53,'\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호','2025-09-25 17:15:49.912614',_binary '\0','\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호',NULL,'나르고가 도착했습니다!','BUYER_ARRIVAL',1),(54,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 17:32:50.918994',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(55,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 17:33:06.874200',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(56,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 23:33:55.098157',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(57,'\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호','2025-09-25 23:34:17.053196',_binary '\0','\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호',NULL,'나르고가 도착했습니다!','BUYER_ARRIVAL',1),(58,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 23:36:22.938090',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(59,'\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호','2025-09-25 23:36:51.394739',_binary '\0','\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호',NULL,'나르고가 도착했습니다!','BUYER_ARRIVAL',1),(60,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 23:42:25.460102',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(61,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-25 23:50:36.935761',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(62,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-26 00:01:20.993686',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(63,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-26 00:02:56.219437',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(64,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-26 00:03:43.090534',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(65,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-26 00:05:50.326349',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(66,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-26 00:06:32.024995',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(67,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-26 00:07:07.436633',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(68,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-26 00:07:43.712826',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(69,'\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호','2025-09-26 00:08:44.262412',_binary '\0','\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호',NULL,'나르고가 도착했습니다!','BUYER_ARRIVAL',1),(70,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-26 00:10:47.015226',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(71,'\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호','2025-09-26 00:11:04.704095',_binary '\0','\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호',NULL,'나르고가 도착했습니다!','BUYER_ARRIVAL',1),(72,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-26 00:18:19.926445',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(73,'\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호','2025-09-26 00:18:35.449089',_binary '\0','\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호',NULL,'나르고가 도착했습니다!','BUYER_ARRIVAL',1),(74,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-26 00:21:20.192639',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(75,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-26 00:27:24.907512',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(76,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-26 00:28:03.777227',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(77,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-26 00:31:42.386728',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(78,'\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호','2025-09-26 00:36:55.249598',_binary '\0','\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호',NULL,'나르고가 도착했습니다!','BUYER_ARRIVAL',1),(79,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-26 00:37:54.611388',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(80,'\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호','2025-09-26 00:38:20.871508',_binary '\0','\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호',NULL,'나르고가 도착했습니다!','BUYER_ARRIVAL',1),(81,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-26 00:44:05.852786',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(82,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-26 08:56:33.980140',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(83,'\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호','2025-09-26 08:57:32.306816',_binary '\0','\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호',NULL,'나르고가 도착했습니다!','BUYER_ARRIVAL',1),(84,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-26 09:05:52.804853',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(85,'\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호','2025-09-26 09:09:43.694638',_binary '\0','\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호',NULL,'나르고가 도착했습니다!','BUYER_ARRIVAL',1),(86,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-26 09:20:04.871287',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(87,'\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호','2025-09-26 09:22:58.405689',_binary '\0','\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호',NULL,'나르고가 도착했습니다!','BUYER_ARRIVAL',1),(88,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-26 09:42:53.334811',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3),(89,'\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호','2025-09-26 09:45:44.086948',_binary '\0','\'고래팝니다\' 상품이 도착했습니다. 물건을 가져가주세요. 판매자: 김승호',NULL,'나르고가 도착했습니다!','BUYER_ARRIVAL',1),(90,'\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원','2025-09-26 10:07:11.655341',_binary '\0','\'고래팝니다\' 상품을 로봇에 넣어주세요. 구매자: 오세원',NULL,'나르고가 도착했습니다!','SELLER_ARRIVAL',3);
/*!40000 ALTER TABLE `notifications` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `orders`
--

DROP TABLE IF EXISTS `orders`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `orders` (
                          `order_id` bigint NOT NULL AUTO_INCREMENT COMMENT '주문 고유 ID',
                          `product_id` bigint NOT NULL COMMENT '주문된 상품 ID',
                          `buyer_id` bigint NOT NULL COMMENT '구매자 ID',
                          `status` varchar(20) DEFAULT NULL,
                          `created_at` datetime NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '주문(결제) 일시',
                          `updated_at` datetime NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP COMMENT '상태 변경 일시',
                          `amount` decimal(38,2) DEFAULT NULL,
                          `toss_order_id` varchar(64) DEFAULT NULL COMMENT '토스페이먼츠용 orderId',
                          PRIMARY KEY (`order_id`),
                          UNIQUE KEY `product_id_UNIQUE` (`product_id`),
                          KEY `fk_orders_buyers_idx` (`buyer_id`),
                          KEY `idx_orders_toss_order_id` (`toss_order_id`),
                          CONSTRAINT `fk_orders_products` FOREIGN KEY (`product_id`) REFERENCES `products` (`product_id`) ON DELETE RESTRICT ON UPDATE CASCADE,
                          CONSTRAINT `FKhtx3insd5ge6w486omk4fnk54` FOREIGN KEY (`buyer_id`) REFERENCES `users` (`user_id`)
) ENGINE=InnoDB AUTO_INCREMENT=69 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='주문 및 거래 상태 정보';
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `orders`
--

LOCK TABLES `orders` WRITE;
/*!40000 ALTER TABLE `orders` DISABLE KEYS */;
INSERT INTO `orders` VALUES (1,10,1,'PAYMENT_COMPLETED','2025-09-22 06:03:28','2025-09-26 01:05:17',10000.00,'ORDER_10_1_1758521006304');
/*!40000 ALTER TABLE `orders` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `payments`
--

DROP TABLE IF EXISTS `payments`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `payments` (
                            `payment_id` bigint NOT NULL AUTO_INCREMENT COMMENT '결제 ID',
                            `order_id` bigint NOT NULL COMMENT '주문 ID',
                            `payment_key` varchar(255) NOT NULL COMMENT 'Toss 결제 고유 키',
                            `amount` decimal(10,0) NOT NULL COMMENT '결제 금액',
                            `status` enum('DONE','CANCELED') NOT NULL COMMENT '결제 상태',
                            `approved_at` datetime NOT NULL COMMENT '결제 승인 일시',
                            PRIMARY KEY (`payment_id`),
                            UNIQUE KEY `payment_key_UNIQUE` (`payment_key`),
                            KEY `fk_payments_orders_idx` (`order_id`),
                            CONSTRAINT `fk_payments_orders` FOREIGN KEY (`order_id`) REFERENCES `orders` (`order_id`) ON DELETE RESTRICT ON UPDATE CASCADE
) ENGINE=InnoDB AUTO_INCREMENT=26 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='Toss 결제 정보';
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `payments`
--

LOCK TABLES `payments` WRITE;
/*!40000 ALTER TABLE `payments` DISABLE KEYS */;
INSERT INTO `payments` VALUES (1,1,'tviva20250922150339lfKj1',10000,'DONE','2025-09-22 06:04:27');
/*!40000 ALTER TABLE `payments` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `product_images`
--

DROP TABLE IF EXISTS `product_images`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `product_images` (
                                  `image_id` bigint NOT NULL AUTO_INCREMENT COMMENT '이미지 ID',
                                  `product_id` bigint NOT NULL COMMENT '상품 ID',
                                  `image_url` varchar(255) NOT NULL COMMENT 'S3 이미지 URL',
                                  PRIMARY KEY (`image_id`),
                                  KEY `fk_product_images_products_idx` (`product_id`),
                                  CONSTRAINT `fk_product_images_products` FOREIGN KEY (`product_id`) REFERENCES `products` (`product_id`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB AUTO_INCREMENT=63 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='상품 이미지 정보';
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `product_images`
--

LOCK TABLES `product_images` WRITE;
/*!40000 ALTER TABLE `product_images` DISABLE KEYS */;
INSERT INTO `product_images` VALUES (9,2,'product/2/1'),(10,2,'product/2/2'),(11,2,'product/2/3'),(12,2,'product/2/4'),(13,3,'product/3/1'),(14,3,'product/3/2'),(15,3,'product/3/3'),(16,4,'product/4/1'),(17,4,'product/4/2'),(18,5,'product/5/1'),(19,5,'product/5/2'),(20,5,'product/5/3'),(21,6,'product/6/1'),(22,6,'product/6/2'),(23,6,'product/6/3'),(33,10,'product/10/1'),(44,14,'product/14/1'),(45,14,'product/14/2'),(46,14,'product/14/3'),(47,14,'product/14/4'),(48,15,'product/15/1'),(49,15,'product/15/2'),(50,15,'product/15/3'),(51,15,'product/15/4'),(52,15,'product/15/5'),(53,15,'product/15/6'),(62,18,'product/18/1');
/*!40000 ALTER TABLE `product_images` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `products`
--

DROP TABLE IF EXISTS `products`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
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
) ENGINE=InnoDB AUTO_INCREMENT=19 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='판매 상품 정보';
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `products`
--

LOCK TABLES `products` WRITE;
/*!40000 ALTER TABLE `products` DISABLE KEYS */;
INSERT INTO `products` VALUES (2,1,'갤럭시 Z 플립7 거의 새거 급처','선물 받았는데 제 스타일이 아니라 거의 안 썼습니다.\n구매 시기 2025년 8월, 영수증 있고 무상 A/S 기간 남아 있습니다.\n기스 하나도 없고 강화유리 붙여둔 상태입니다.\n직거래는 단지 정문 쪽에서만 가능해요.',50000.00,'FOR_SALE','2025-09-22 05:46:31','2025-09-25 01:40:03','역삼래미안아파트','서울특별시','강남구','삼성동'),(3,1,'갤럭시 워치6 클래식 43mm 판매합니다!','작년에 구매했는데 손목에 잘 안 맞아서 사용 횟수가 많지 않습니다.\n\n화면 보호 필름 붙여놔서 스크래치 전혀 없고, 기본 충전 크래들 포함해서 드립니다.\n\n블랙 가죽 스트랩 추가로 같이 드려요.',30000.00,'FOR_SALE','2025-09-22 05:49:17','2025-09-22 05:49:17','역삼래미안아파트','서울특별시','강남구','삼성동'),(4,1,'갤럭시 Z 폴드5 화이트, 상태 양호','배터리 성능 97% 이상으로 하루 종일 무리 없이 사용 가능합니다.\n\n생활 기스 약간 있지만 사용에 지장 없습니다.\n\n케이스 2개랑 S펜도 같이 드립니다.\n',60000.00,'FOR_SALE','2025-09-22 05:51:19','2025-09-23 05:34:57','역삼래미안아파트','서울특별시','강남구','삼성동'),(5,1,'갤럭시 탭 S9 울트라, 최상','영화 보고 필기용으로만 사용했는데, 최근 노트북을 새로 사서 잘 안 쓰게 되어 판매합니다.\n\n1년 미만 사용했고, 화면 보호 필름 붙여놔서 기스 없습니다.\n\n펜과 기본 충전기 모두 포함입니다.\n',55000.00,'FOR_SALE','2025-09-22 05:52:42','2025-09-24 04:12:02','역삼래미안아파트','서울특별시','강남구','삼성동'),(6,1,'갤럭시 Z 폴드5 화이트, 미개봉','선물 받은 건데 기존에 쓰던 폰이 있어서 개봉하지 않고 그대로 보관 중입니다.\n\n완전 미개봉 새 제품이라 필름도 안 뜯었고, 정품 박스 상태 그대로입니다.\n\n모델은 갤럭시 Z 폴드5 화이트, 256GB입니다.\n\n정품 보증 기간도 아직 시작 전이라 걱정 없으실 거예요.\n',63000.00,'FOR_SALE','2025-09-22 05:56:18','2025-09-24 05:43:13','역삼래미안아파트','서울특별시','강남구','삼성동'),(10,3,'고래팝니다','고래',10000.00,'SOLD','2025-09-22 05:59:32','2025-09-22 06:04:28','역삼래미안아파트','서울특별시','강남구','삼성동'),(14,1,'삼성 갤럭시 워치, 거의 새거','작년에 구매했는데 손목에 잘 안 맞아서 사용 횟수가 많지 않습니다.\n\n화면 보호 필름 붙여놔서 스크래치 전혀 없고, 기본 충전 크래들 포함해서 드립니다.\n\n블랙 가죽 스트랩 추가로 같이 드려요.',37000.00,'FOR_SALE','2025-09-22 06:24:55','2025-09-25 05:08:49','역삼래미안아파트','서울특별시','강남구','삼성동'),(15,2,'중고용','수고요',123.00,'SOLD','2025-09-22 06:30:34','2025-09-25 01:37:42','역삼래미안아파트','서울특별시','강남구','삼성동'),(18,3,'소가죽지갑','명품 소가죽 지갑 팝니다',60000.00,'SOLD','2025-09-23 00:33:50','2025-09-25 01:37:42','역삼래미안아파트','서울특별시','강남구','삼성동');
/*!40000 ALTER TABLE `products` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `robots`
--

DROP TABLE IF EXISTS `robots`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `robots` (
                          `robot_id` bigint NOT NULL AUTO_INCREMENT COMMENT '로봇 ID',
                          `robot_name` varchar(50) NOT NULL COMMENT '로봇 식별 이름',
                          `status` varchar(10) DEFAULT NULL,
                          PRIMARY KEY (`robot_id`),
                          UNIQUE KEY `robot_name_UNIQUE` (`robot_name`)
) ENGINE=InnoDB AUTO_INCREMENT=2 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='자율주행 로봇 정보';
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `robots`
--

LOCK TABLES `robots` WRITE;
/*!40000 ALTER TABLE `robots` DISABLE KEYS */;
INSERT INTO `robots` VALUES (1,'NareuGO','VALID');
/*!40000 ALTER TABLE `robots` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `user`
--

DROP TABLE IF EXISTS `user`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `user` (
                        `id` bigint NOT NULL AUTO_INCREMENT,
                        `apartment_name` varchar(255) DEFAULT NULL,
                        `birth` varchar(255) DEFAULT NULL,
                        `building_dong` int DEFAULT NULL,
                        `building_ho` int DEFAULT NULL,
                        `deleted_at` datetime(6) DEFAULT NULL,
                        `email` varchar(255) NOT NULL,
                        `eup_myeon_dong` varchar(255) DEFAULT NULL,
                        `is_active` bit(1) DEFAULT NULL,
                        `name` varchar(255) NOT NULL,
                        `nickname` varchar(255) DEFAULT NULL,
                        `phone_number` varchar(255) DEFAULT NULL,
                        `provider_id` varchar(255) DEFAULT NULL,
                        `provider_type` enum('KAKAO','NAVER') DEFAULT NULL,
                        `role` enum('SELLER','USER') DEFAULT NULL,
                        `sex` enum('MAN','WOMAN') DEFAULT NULL,
                        `si_do` varchar(255) DEFAULT NULL,
                        `si_gun_gu` varchar(255) DEFAULT NULL,
                        PRIMARY KEY (`id`),
                        UNIQUE KEY `UKob8kqyqqgmefl0aco34akdtpe` (`email`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `user`
--

LOCK TABLES `user` WRITE;
/*!40000 ALTER TABLE `user` DISABLE KEYS */;
/*!40000 ALTER TABLE `user` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `user_favorites`
--

DROP TABLE IF EXISTS `user_favorites`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `user_favorites` (
                                  `created_at` datetime(6) NOT NULL,
                                  `favorite_id` bigint NOT NULL AUTO_INCREMENT,
                                  `product_id` bigint NOT NULL,
                                  `user_id` bigint NOT NULL,
                                  PRIMARY KEY (`favorite_id`),
                                  UNIQUE KEY `unique_user_product` (`user_id`,`product_id`),
                                  CONSTRAINT `FK4sv7b9w9adr0fjnc4u10exlwm` FOREIGN KEY (`user_id`) REFERENCES `users` (`user_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `user_favorites`
--

LOCK TABLES `user_favorites` WRITE;
/*!40000 ALTER TABLE `user_favorites` DISABLE KEYS */;
/*!40000 ALTER TABLE `user_favorites` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `user_product_history`
--

DROP TABLE IF EXISTS `user_product_history`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `user_product_history` (
                                        `history_id` bigint NOT NULL AUTO_INCREMENT,
                                        `product_id` bigint NOT NULL,
                                        `user_id` bigint NOT NULL,
                                        `viewed_at` datetime(6) NOT NULL,
                                        PRIMARY KEY (`history_id`),
                                        KEY `FK4k7ro4pr6ym5ebvbxc5u47ep5` (`user_id`),
                                        CONSTRAINT `FK4k7ro4pr6ym5ebvbxc5u47ep5` FOREIGN KEY (`user_id`) REFERENCES `users` (`user_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `user_product_history`
--

LOCK TABLES `user_product_history` WRITE;
/*!40000 ALTER TABLE `user_product_history` DISABLE KEYS */;
/*!40000 ALTER TABLE `user_product_history` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `users`
--

DROP TABLE IF EXISTS `users`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `users` (
                         `user_id` bigint NOT NULL AUTO_INCREMENT,
                         `birth` varchar(255) DEFAULT NULL,
                         `deleted_at` datetime(6) DEFAULT NULL,
                         `email` varchar(255) NOT NULL,
                         `is_active` bit(1) DEFAULT NULL,
                         `name` varchar(255) NOT NULL,
                         `phone_number` varchar(255) DEFAULT NULL,
                         `provider_id` varchar(255) DEFAULT NULL,
                         `provider_type` varchar(50) DEFAULT NULL,
                         `role` enum('SELLER','USER') DEFAULT NULL,
                         `sex` varchar(10) DEFAULT NULL,
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
                         UNIQUE KEY `UK_users_email` (`email`),
                         KEY `idx_user_address_verified` (`address_verified`)
) ENGINE=InnoDB AUTO_INCREMENT=8 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `users`
--

LOCK TABLES `users` WRITE;
/*!40000 ALTER TABLE `users` DISABLE KEYS */;
INSERT INTO `users` VALUES (1,'1998-08-05',NULL,'tpdnjs7878@naver.com',_binary '','오세원','010-5845-7111','kakao_001','KAKAO','USER','MALE','역삼래미안아파트',3,202,'삼성동','서울특별시','강남구','네고왕',1,'2025-09-22 05:22:29'),(2,'1999-01-02',NULL,'yy3111@daum.net',_binary '','이용훈','010-4948-5690','kakao_001','KAKAO','USER','MALE','역삼래미안아파트',1,202,'삼성동','서울특별시','강남구','중고왕',1,'2025-09-22 05:22:58'),(3,'1996-11-20',NULL,'qhddnjssla3@naver.com',_binary '','김승호','010-9983-9132','kakao_001','KAKAO','USER','MALE','역삼래미안아파트',1,301,'삼성동','서울특별시','강남구','영희',1,NULL),(6,NULL,NULL,'cerealblue@naver.com',_binary '','박현빈',NULL,'google_id_abcxyz','GOOGLE','USER',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,0,NULL),(7,NULL,NULL,'ssafyA501@naver.com',NULL,'나르고','010-5121-5451',NULL,NULL,'USER','MALE','역삼래미안아파트',7,777,'삼성동','서울특별시','강남구','나르고',0,NULL);
/*!40000 ALTER TABLE `users` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2025-09-26 10:07:32

