package org.example.nareugobackend.common.exception.product;

import org.example.nareugobackend.common.exception.ErrorCode;
import org.example.nareugobackend.common.exception.NAREUGOException;

public class ProductException extends NAREUGOException {

    public ProductException(ErrorCode errorCode) {
        super(errorCode);
    }

}
