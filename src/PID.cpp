#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::PID(const PID &pid) {
  Kp = pid.Kp;
  Ki = pid.Ki;
  Kd = pid.Kd;

  p_error = pid.p_error;
  i_error = pid.i_error;
  d_error = pid.d_error;
}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0;
  i_error = 0;
  d_error = 0;
}

void PID::UpdateError(double cte) {
  d_error = p_error - cte;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  return Kp*p_error + Ki*i_error + Kd*d_error;
}

void PID::ModifyK(int i, double value) {
  switch(i) {
    case 0 :
      Kp = value;
      break;
    case 1 :
      Ki = value;
      break;
    case 2 :
      Kd = value;
      break;
  }
}

void PID::setKp(double kp) {
  Kp = kp;
}

void PID::setKi(double ki) {
  Ki = ki;
}

void PID::setKd(double kd) {
  Kd = kd;
}

double PID::getKp() const {
  return Kp;
}

double PID::getKi() const {
  return Ki;
}

double PID::getKd() const {
  return Kd;
}

double PID::getPError() const {
  return p_error;
}

double PID::getIError() const {
  return i_error;
}

double PID::getDError() const {
  return d_error;
}
