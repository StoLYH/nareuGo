package org.example.nareugobackend.api.service.payment;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.example.nareugobackend.domain.payment.Order;
import org.example.nareugobackend.domain.payment.OrderMapper;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.util.List;

@Service
@RequiredArgsConstructor
@Slf4j
public class OrderExpirationService {

    private final OrderMapper orderMapper;

    @Scheduled(fixedRate = 300000) // 5분마다 실행 (300000ms = 5분)
    @Transactional
    public void expirePendingOrders() {
        try {
            List<Order> expiredOrders = orderMapper.findExpiredPendingOrders();
            
            if (expiredOrders.isEmpty()) {
                log.debug("만료된 결제 대기 주문이 없습니다.");
                return;
            }

            List<Long> orderIds = expiredOrders.stream()
                .map(Order::getOrderId)
                .toList();

            orderMapper.expireOrders(orderIds);
            
            log.info("만료된 결제 대기 주문 {}건을 취소 처리했습니다. orderIds: {}", 
                expiredOrders.size(), orderIds);
                
        } catch (Exception e) {
            log.error("결제 대기 주문 만료 처리 중 오류 발생", e);
        }
    }
}
