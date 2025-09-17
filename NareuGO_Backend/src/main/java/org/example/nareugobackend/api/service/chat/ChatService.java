package org.example.nareugobackend.api.service.chat;

import org.example.nareugobackend.api.controller.chat.request.ChatMessage;
import org.example.nareugobackend.api.controller.chat.request.ChatRoom;

import java.util.List;

public interface ChatService {
    
    // 메시지 저장
    void saveMessage(ChatMessage message);
    
    // 채팅방 생성 또는 기존 방 찾기
    Long findOrCreateChatRoom(String user1Id, String user2Id);
    
    // 채팅방의 메시지 목록 조회
    List<ChatMessage> getChatMessages(Long roomId);
    
    // 사용자의 채팅방 목록 조회
    List<ChatRoom> getUserChatRooms(String userId);
}