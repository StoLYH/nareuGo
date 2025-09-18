package org.example.nareugobackend.api.controller.transaction;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.api.service.transaction.TransactionService;
import org.example.nareugobackend.api.service.transaction.response.TransactionHistoryResponse;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequiredArgsConstructor
@RequestMapping("/transactions")
public class TransactionController {

    private final TransactionService transactionService;

    @GetMapping("/sales/{sellerId}")
    public ResponseEntity<List<TransactionHistoryResponse>> getSalesHistory(@PathVariable Long sellerId) {
        List<TransactionHistoryResponse> salesHistory = transactionService.getSalesHistory(sellerId);
        return ResponseEntity.ok(salesHistory);
    }

    @GetMapping("/purchases/{buyerId}")
    public ResponseEntity<List<TransactionHistoryResponse>> getPurchaseHistory(@PathVariable Long buyerId) {
        List<TransactionHistoryResponse> purchaseHistory = transactionService.getPurchaseHistory(buyerId);
        return ResponseEntity.ok(purchaseHistory);
    }
}