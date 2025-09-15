package org.example.nareugobackend.common.exception.auth;

import static org.springframework.http.HttpStatus.BAD_REQUEST;
import static org.springframework.http.HttpStatus.UNAUTHORIZED;

import lombok.RequiredArgsConstructor;
import org.example.nareugobackend.common.exception.ErrorCode;
import org.springframework.http.HttpStatus;

@RequiredArgsConstructor
public enum AuthErrorCode implements ErrorCode {

  // Social Login -> 400
  UNSUPPORTED_PROVIDER(BAD_REQUEST, "NAREUGO001"),
  LOGIN_FAILED(BAD_REQUEST, "NAREUGO002"),

  // Token -> 401
  TOKEN_INVALID(UNAUTHORIZED, "NAREUGO003"),
  TOKEN_EXPIRED(UNAUTHORIZED, "NAREUGO004"),
  TOKEN_UNSUPPORTED(UNAUTHORIZED, "NAREUGO005"),
  TOKEN_WRONG(UNAUTHORIZED, "NAREUGO006"),
  TOKEN_NOT_MATCHED(UNAUTHORIZED, "NAREUGO007"),
  REFRESH_TOKEN_NOT_EXIST_IN_COOKIE(UNAUTHORIZED, "NAREUGO009"),
  REFRESH_TOKEN_EXPIRED(UNAUTHORIZED, "NAREUGO009");

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