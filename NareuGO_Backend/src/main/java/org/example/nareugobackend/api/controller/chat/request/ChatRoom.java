package org.example.nareugobackend.api.controller.chat.request;

import lombok.Data;

@Data
public class ChatRoom {
    private Long roomId;
    private String user1Id;
    private String user2Id;
    private String createdAt;
    private String lastMessageAt;
    private String lastMessage;
}