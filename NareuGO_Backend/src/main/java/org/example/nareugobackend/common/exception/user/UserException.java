package org.example.nareugobackend.common.exception.user;

import org.example.nareugobackend.common.exception.NAREUGOException;

public class UserException extends NAREUGOException {

  public UserException(UserErrorCode errorCode) {
    super(errorCode);
  }
}