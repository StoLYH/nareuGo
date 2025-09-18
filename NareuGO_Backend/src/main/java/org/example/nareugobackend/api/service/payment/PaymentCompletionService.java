package org.example.nareugobackend.api.service.payment;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.service.notification.FcmService;
import org.example.nareugobackend.common.model.Order;
import org.example.nareugobackend.common.model.OrderStatus;
import org.example.nareugobackend.mapper.OrderMapper;
import org.example.nareugobackend.domain.product.Product;
import org.example.nareugobackend.domain.product.ProductRepository;
import org.example.nareugobackend.domain.user.User;
import org.example.nareugobackend.domain.user.UserRepository;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

@Service
@RequiredArgsConstructor
public class PaymentCompletionService {

    private final OrderMapper orderMapper;
    private final FcmService fcmService;
    private final ProductRepository productRepository;
    private final UserRepository userRepository;

    @Transactional
    public void completePayment(Long orderId) {
        Order order = orderMapper.findById(orderId);
        if (order == null) {
            throw new IllegalArgumentException("존재하지 않는 주문입니다.");
        }

        if (order.getStatus() != OrderStatus.PAYMENT_PENDING) {
            throw new IllegalStateException("결제 완료할 수 없는 주문 상태입니다.");
        }

        // 주문 상태를 PAYMENT_COMPLETED로 변경
        orderMapper.updateStatus(order.getOrderId(), OrderStatus.PAYMENT_COMPLETED);

        // 구매자에게 판매 확정 알림 발송
        Product product = productRepository.findById(order.getProductId()).orElse(null);
        if (product != null) {
            User seller = product.getSeller();
            String sellerName = seller.getNickname() != null ? seller.getNickname() : seller.getName();

            fcmService.sendSaleConfirmationNotification(
                order.getBuyerId(),
                product.getTitle()
            );
        }
    }
}