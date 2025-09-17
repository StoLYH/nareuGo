package org.example.nareugobackend.api.controller.chat.request;

import lombok.Data;

@Data
public class ChatMessage {
    private Long roomId;
    private String senderId;
    private String receiverId;
    private String content;
    private String timestamp;
}
