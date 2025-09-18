package org.example.nareugobackend.common.util;

import java.security.SecureRandom;
import java.util.Random;

public class OrderNumberGenerator {

    private static final Random RANDOM = new SecureRandom();
    private static final int ORDER_NUMBER_LENGTH = 12;

    public static String generateOrderNumber() {
        StringBuilder orderNumber = new StringBuilder();

        for (int i = 0; i < ORDER_NUMBER_LENGTH; i++) {
            orderNumber.append(RANDOM.nextInt(10));
        }

        return orderNumber.toString();
    }
}