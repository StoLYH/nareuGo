package org.example.nareugobackend.common.exception.user;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.common.exception.ErrorCode;
import org.springframework.http.HttpStatus;

@RequiredArgsConstructor
public enum UserErrorCode implements ErrorCode {

  USER_NOT_FOUND(HttpStatus.NOT_FOUND, "U001"),
  USER_ALREADY_EXISTS(HttpStatus.CONFLICT, "U002"),
  INVALID_USER_DATA(HttpStatus.BAD_REQUEST, "U003");

  private final HttpStatus httpStatus;
  private final String code;

  @Override
  public HttpStatus getHttpStatus() {
    return httpStatus;
  }

  @Override
  public String getCode() {
    return code;
  }
}