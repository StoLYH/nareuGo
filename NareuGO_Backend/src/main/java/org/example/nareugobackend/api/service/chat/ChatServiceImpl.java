package org.example.nareugobackend.api.service.chat;

import org.example.nareugobackend.api.controller.chat.request.ChatMessage;
import org.example.nareugobackend.api.controller.chat.request.ChatRoom;
import org.example.nareugobackend.mapper.ChatMapper;
import org.springframework.stereotype.Service;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.List;

@Service
public class ChatServiceImpl implements ChatService {

    private final ChatMapper chatMapper;

    public ChatServiceImpl(ChatMapper chatMapper) {
        this.chatMapper = chatMapper;
    }

    @Override
    public void saveMessage(ChatMessage message) {
        // 현재 시간 설정
        message.setTimestamp(LocalDateTime.now().format(DateTimeFormatter.ISO_LOCAL_DATE_TIME));
        
        // 채팅방이 없다면 생성 (메시지 저장 시에는 productId가 필수)
        if (message.getRoomId() == null) {
            // 메시지에 productId가 없으면 예외 처리
            if (message.getProductId() == null) {
                throw new IllegalArgumentException("메시지 저장 시 productId는 필수입니다.");
            }
            Long roomId = findOrCreateChatRoom(message.getSenderId(), message.getReceiverId(), message.getProductId());
            message.setRoomId(roomId);
        }
        
        // 메시지 저장
        chatMapper.insertMessage(message);
        
        // 채팅방 최근 메시지 업데이트
        chatMapper.updateRoomLastMessage(message.getRoomId(), message.getContent(), message.getTimestamp());
    }

    @Override
    public Long findOrCreateChatRoom(String user1Id, String user2Id, Long productId) {
        // 기존 채팅방 찾기 (상품별로 구분)
        ChatRoom existingRoom = chatMapper.findChatRoom(user1Id, user2Id, productId);
        
        if (existingRoom != null) {
            return existingRoom.getRoomId();
        }
        
        // 새 채팅방 생성
        ChatRoom newRoom = new ChatRoom();
        newRoom.setUser1Id(user1Id);
        newRoom.setUser2Id(user2Id);
        newRoom.setProductId(productId);
        newRoom.setCreatedAt(LocalDateTime.now().format(DateTimeFormatter.ISO_LOCAL_DATE_TIME));
        
        chatMapper.insertChatRoom(newRoom);
        return newRoom.getRoomId();
    }

    @Override
    public List<ChatMessage> getChatMessages(Long roomId) {
        return chatMapper.findMessagesByRoomId(roomId);
    }

    @Override
    public List<ChatRoom> getUserChatRooms(String userId) {
        return chatMapper.findChatRoomsByUserId(userId);
    }
}