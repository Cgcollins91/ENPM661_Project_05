// Project: ENPM662-Project1-Group1
// License: MIT
// The code in this file represents the collective work of Group 1.
// At times, code has been liberally borrowed from files provided
// with the project's instructions or from OSRF training materials.
// Please see the project report for a list of references, or contact
// the team with specific questions.

class Pid {
  private:
    double m_integral = 0;
    double m_error_previous = 0;
    double m_kp = 0;
    double m_ki = 0;
    double m_kd = 0;

  public:
    double calculate(double error, double dt) {
        //if (error < 0.00001) return 0;
        double interval_error = error - m_error_previous;
        m_integral += error * dt;
        m_error_previous = error;

        double p = m_kp * error;
        double i = m_ki * m_integral;
        double d = m_kd * (interval_error / dt);

        return p + i + d;
    }

    void set_k_values(double kp, double ki, double kd) {
        m_kp = kp;
        m_ki = ki;
        m_kd = kd;
    }

    void reset() {
        m_integral = 0;
        m_error_previous = 0;
    }
};
