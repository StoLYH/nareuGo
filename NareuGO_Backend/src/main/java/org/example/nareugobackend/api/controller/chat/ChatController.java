package org.example.nareugobackend.api.controller.chat;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.service.chat.ChatService;
import org.example.nareugobackend.api.controller.chat.request.ChatMessage;
import org.example.nareugobackend.api.controller.chat.request.ChatRoom;
import org.springframework.http.ResponseEntity;
import org.springframework.messaging.handler.annotation.MessageMapping;
import org.springframework.messaging.simp.SimpMessagingTemplate;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.*;

import java.util.List;


@RequiredArgsConstructor
@RestController
@RequestMapping("/chat")
public class ChatController {

    private final SimpMessagingTemplate template;
    private final ChatService chatService;


    @MessageMapping("/chat.send")   // 클라이언트 → 서버 (/app/chat.send)
    public void send(ChatMessage msg) {
        // 1. DB 저장 (roomId, senderId, receiverId, content)
        chatService.saveMessage(msg);

        // 2. 수신자에게만 전송
        template.convertAndSend("/queue/user." + msg.getReceiverId(), msg);
    }

    // REST API: 사용자의 채팅방 목록 조회
    @GetMapping("/rooms/{userId}")
    public ResponseEntity<List<ChatRoom>> getUserChatRooms(@PathVariable String userId) {
        List<ChatRoom> chatRooms = chatService.getUserChatRooms(userId);
        return ResponseEntity.ok(chatRooms);
    }

    // REST API: 특정 채팅방의 메시지 목록 조회
    @GetMapping("/rooms/{roomId}/messages")
    public ResponseEntity<List<ChatMessage>> getChatMessages(@PathVariable Long roomId) {
        List<ChatMessage> messages = chatService.getChatMessages(roomId);
        return ResponseEntity.ok(messages);
    }

    // REST API: 채팅방 생성 또는 찾기
    @PostMapping("/rooms")
    public ResponseEntity<Long> findOrCreateChatRoom(@RequestParam String user1Id, 
                                                    @RequestParam String user2Id) {
        Long roomId = chatService.findOrCreateChatRoom(user1Id, user2Id);
        return ResponseEntity.ok(roomId);
    }
}
