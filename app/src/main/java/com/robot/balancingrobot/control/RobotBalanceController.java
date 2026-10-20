package com.robot.balancingrobot.control;

/**
 * Controller class managing balancing control loop, mixing balance demand with user
 * drive/steering inputs, and enforcing tilt safety cutoffs.
 */
public class RobotBalanceController {

    private final PidController anglePid;
    private float maxTiltLimitDeg = 40.0f; // Tilt cutoff threshold in degrees
    private float forwardVelocityDemand = 0.0f; // Target forward/backward speed
    private float turnRateDemand = 0.0f;       // Target turning rate
    private boolean enabled = false;

    // Motor output speed range (e.g. -1000 to +1000 or -255 to +255)
    private float maxMotorSpeed = 1000.0f;

    public RobotBalanceController(float kp, float ki, float kd) {
        this.anglePid = new PidController(kp, ki, kd);
        this.anglePid.setOutputLimits(-maxMotorSpeed, maxMotorSpeed);
    }

    public PidController getAnglePid() {
        return anglePid;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            anglePid.reset();
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setMaxTiltLimitDeg(float limitDeg) {
        this.maxTiltLimitDeg = limitDeg;
    }

    public float getMaxTiltLimitDeg() {
        return maxTiltLimitDeg;
    }

    public void setDriveInputs(float forwardVelocity, float turnRate) {
        this.forwardVelocityDemand = forwardVelocity;
        this.turnRateDemand = turnRate;
    }

    public float getForwardVelocityDemand() {
        return forwardVelocityDemand;
    }

    public float getTurnRateDemand() {
        return turnRateDemand;
    }

    /**
     * Container for left/right motor speed commands.
     */
    public static class MotorOutputs {
        public final int leftMotor;
        public final int rightMotor;
        public final boolean safetyTripped;

        public MotorOutputs(int leftMotor, int rightMotor, boolean safetyTripped) {
            this.leftMotor = leftMotor;
            this.rightMotor = rightMotor;
            this.safetyTripped = safetyTripped;
        }
    }

    /**
     * Update loop to calculate motor speeds based on tilt pitch angle and pitch rate.
     * @param pitchAngleDeg Fused current pitch angle in degrees
     * @param pitchRateDegPerSec Current pitch angular velocity in deg/s
     * @param dt Delta time in seconds
     * @return MotorOutputs object containing left and right speeds
     */
    public MotorOutputs update(float pitchAngleDeg, float pitchRateDegPerSec, float dt) {
        // Safety cutoff check: if robot has fallen over beyond max tilt threshold, disable motors
        if (Math.abs(pitchAngleDeg) > maxTiltLimitDeg) {
            enabled = false;
            anglePid.reset();
            return new MotorOutputs(0, 0, true);
        }

        if (!enabled) {
            return new MotorOutputs(0, 0, false);
        }

        // Target pitch setpoint can be slightly tilted by forward demand to induce movement
        // forwardVelocityDemand is normalized e.g. -1.0 to 1.0 -> map to max ~5 degree lean angle
        float targetPitchAngle = forwardVelocityDemand * 5.0f;

        // Compute balance demand from PID
        float balanceOutput = anglePid.update(pitchAngleDeg, targetPitchAngle, dt, pitchRateDegPerSec);

        // Mix balance output with turning command
        // turnRateDemand range: -1.0 (full left) to +1.0 (full right)
        float turnComponent = turnRateDemand * (maxMotorSpeed * 0.4f);

        float leftDemand = balanceOutput + turnComponent;
        float rightDemand = balanceOutput - turnComponent;

        // Clamp to maxMotorSpeed
        leftDemand = Math.max(-maxMotorSpeed, Math.min(maxMotorSpeed, leftDemand));
        rightDemand = Math.max(-maxMotorSpeed, Math.min(maxMotorSpeed, rightDemand));

        return new MotorOutputs(Math.round(leftDemand), Math.round(rightDemand), false);
    }
}
