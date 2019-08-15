#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */
  PID();
  PID(const PID &pid);

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /**
   * Modify Kx value : 0 => Kp, 1 => Ki, 2 => Kd
   * Used in koef optimization
   */
  void ModifyK(int i, double value);

  void setKp(double kp);

  void setKi(double ki);

  void setKd(double kd);

    double getKp() const;

    double getKi() const;

    double getKd() const;

    double getPError() const;

    double getIError() const;

    double getDError() const;

private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
};

#endif  // PID_H