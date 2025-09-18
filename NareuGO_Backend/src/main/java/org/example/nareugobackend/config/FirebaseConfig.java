package org.example.nareugobackend.config;

import com.google.auth.oauth2.GoogleCredentials;
import com.google.firebase.FirebaseApp;
import com.google.firebase.FirebaseOptions;
import com.google.firebase.messaging.FirebaseMessaging;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.boot.autoconfigure.condition.ConditionalOnProperty;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.core.io.ClassPathResource;

import java.io.IOException;
import java.io.InputStream;

@Slf4j
@Configuration
public class FirebaseConfig {

    @Value("${firebase.service-account-key:firebase-service-account.json}")
    private String serviceAccountKeyPath;

    @Value("${firebase.project-id:your-project-id}")
    private String projectId;

    @Bean
    @ConditionalOnProperty(name = "firebase.enabled", havingValue = "true", matchIfMissing = false)
    public FirebaseMessaging firebaseMessaging() throws IOException {
        try {
            // 서비스 계정 키 파일 로드
            InputStream serviceAccount = new ClassPathResource(serviceAccountKeyPath).getInputStream();

            FirebaseOptions options = FirebaseOptions.builder()
                .setCredentials(GoogleCredentials.fromStream(serviceAccount))
                .setProjectId(projectId)
                .build();

            FirebaseApp app;
            if (FirebaseApp.getApps().isEmpty()) {
                app = FirebaseApp.initializeApp(options);
                log.info("Firebase application initialized successfully");
            } else {
                app = FirebaseApp.getInstance();
            }

            return FirebaseMessaging.getInstance(app);

        } catch (Exception e) {
            log.error("Failed to initialize Firebase", e);
            throw e;
        }
    }
}