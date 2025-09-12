package org.example.nareugobackend.common.exception;

import lombok.Getter;

@Getter
public class NAREUGOException extends RuntimeException {

  private final ErrorCode errorCode;

  public NAREUGOException(ErrorCode errorCode) {
    super(errorCode.getCode());
    this.errorCode = errorCode;
  }
}