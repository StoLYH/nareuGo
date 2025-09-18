package org.example.nareugobackend.mapper;

import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Param;
import org.example.nareugobackend.api.controller.chat.request.ChatMessage;
import org.example.nareugobackend.api.controller.chat.request.ChatRoom;

import java.util.List;

@Mapper
public interface ChatMapper {
    
    // 메시지 저장
    void insertMessage(ChatMessage message);
    
    // 채팅방 생성
    void insertChatRoom(ChatRoom chatRoom);
    
    // 채팅방 찾기 (두 사용자 간)
    ChatRoom findChatRoom(@Param("user1Id") String user1Id, @Param("user2Id") String user2Id);
    
    // 채팅방의 메시지 목록 조회
    List<ChatMessage> findMessagesByRoomId(@Param("roomId") Long roomId);
    
    // 사용자의 채팅방 목록 조회
    List<ChatRoom> findChatRoomsByUserId(@Param("userId") String userId);
    
    // 채팅방 최근 메시지 업데이트
    void updateRoomLastMessage(@Param("roomId") Long roomId, 
                              @Param("lastMessage") String lastMessage, 
                              @Param("lastMessageAt") String lastMessageAt);
}
