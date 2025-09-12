package org.example.nareugobackend.common.exception.auth;

import org.example.nareugobackend.common.exception.ErrorCode;

public class AuthException extends RuntimeException {

  public AuthException(ErrorCode errorCode) {
    super(errorCode.getCode());
  }
  
  public AuthException(String message) {
    super(message);
  }
}
