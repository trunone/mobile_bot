package com.robot.balancingrobot.control;

/**
 * PID Controller implementation for self-balancing mobile robot.
 */
public class PidController {

    private float kp;
    private float ki;
    private float kd;

    private float integral = 0.0f;
    private float lastError = 0.0f;
    private float minOutput = -1000.0f;
    private float maxOutput = 1000.0f;
    private float maxIntegral = 500.0f;
    private boolean initialized = false;

    public PidController(float kp, float ki, float kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void setGains(float kp, float ki, float kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public float getKp() { return kp; }
    public float getKi() { return ki; }
    public float getKd() { return kd; }

    public void setOutputLimits(float min, float max) {
        this.minOutput = min;
        this.maxOutput = max;
    }

    public void setIntegralLimit(float maxIntegral) {
        this.maxIntegral = maxIntegral;
    }

    public void reset() {
        this.integral = 0.0f;
        this.lastError = 0.0f;
        this.initialized = false;
    }

    /**
     * Compute PID output given current process value, target setpoint, and dt.
     * @param processValue Actual value (e.g. current pitch angle)
     * @param setpoint Target value (e.g. target pitch angle)
     * @param dt Time delta in seconds
     * @param processValueDerivative Optional derivative measured directly (e.g. gyro pitch rate) or Float.NaN if computed
     * @return Output command
     */
    public float update(float processValue, float setpoint, float dt, float processValueDerivative) {
        float error = setpoint - processValue;

        if (!initialized) {
            lastError = error;
            integral = 0.0f;
            initialized = true;
        }

        // Integral computation with anti-windup clamping
        integral += error * dt;
        if (integral > maxIntegral) integral = maxIntegral;
        if (integral < -maxIntegral) integral = -maxIntegral;

        // Derivative computation (either measured pitch rate or finite difference)
        float derivative;
        if (!Float.isNaN(processValueDerivative)) {
            // Negative sign because processValue rate = - error rate when setpoint is constant
            derivative = -processValueDerivative;
        } else if (dt > 0.0001f) {
            derivative = (error - lastError) / dt;
        } else {
            derivative = 0.0f;
        }

        lastError = error;

        float output = (kp * error) + (ki * integral) + (kd * derivative);

        // Clamp output
        if (output > maxOutput) output = maxOutput;
        if (output < minOutput) output = minOutput;

        return output;
    }
}
