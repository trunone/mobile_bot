package com.robot.balancingrobot.sensor;

/**
 * Filter and state estimator for vertical portrait mounted Android phone.
 * Uses accelerometer and gyroscope measurements with a complementary filter
 * to calculate real-time pitch angle (degrees) and pitch rate (deg/s).
 */
public class PitchEstimator {

    private float alpha = 0.98f; // Complementary filter parameter
    private float pitchAngle = 0.0f; // Current fused pitch angle in degrees
    private float pitchRate = 0.0f;  // Current pitch rate in deg/s
    private float pitchOffset = 0.0f; // Zero-pitch offset calibration angle in degrees
    private long lastTimestampNs = 0;
    private boolean initialized = false;

    public PitchEstimator() {}

    public PitchEstimator(float alpha) {
        this.alpha = alpha;
    }

    public void setAlpha(float alpha) {
        if (alpha >= 0.0f && alpha <= 1.0f) {
            this.alpha = alpha;
        }
    }

    public float getAlpha() {
        return alpha;
    }

    public void setPitchOffset(float offsetDeg) {
        this.pitchOffset = offsetDeg;
    }

    public float getPitchOffset() {
        return pitchOffset;
    }

    public void calibrateZero() {
        this.pitchOffset = this.pitchAngle + this.pitchOffset;
    }

    public void reset() {
        this.pitchAngle = 0.0f;
        this.pitchRate = 0.0f;
        this.lastTimestampNs = 0;
        this.initialized = false;
    }

    /**
     * Calculates accelerometer pitch angle for vertically mounted phone (portrait).
     * Gravity vector is -Y when upright.
     * Pitch tilt around X-axis.
     * @param ax Accel X (m/s^2)
     * @param ay Accel Y (m/s^2)
     * @param az Accel Z (m/s^2)
     * @return Raw pitch angle in degrees.
     */
    public static float calculateAccelPitch(float ax, float ay, float az) {
        // Vertical portrait phone upright: ay = -9.81, az = 0.
        // Tilting around X-axis shifts gravity into Z-axis.
        // pitch = atan2(az, -ay) in degrees
        double pitchRad = Math.atan2(az, -ay);
        return (float) Math.toDegrees(pitchRad);
    }

    /**
     * Process sensor readings update.
     * @param timestampNs Event timestamp in nanoseconds
     * @param ax Accel X (m/s^2)
     * @param ay Accel Y (m/s^2)
     * @param az Accel Z (m/s^2)
     * @param gx Gyro X (rad/s or deg/s depending on isGyroInRad)
     * @param isGyroInRad True if gyro rate is in radians/second (Android default is rad/s)
     */
    public void update(long timestampNs, float ax, float ay, float az, float gx, boolean isGyroInRad) {
        float rawGyroDeg = isGyroInRad ? (float) Math.toDegrees(gx) : gx;
        this.pitchRate = rawGyroDeg;

        float accelPitch = calculateAccelPitch(ax, ay, az);

        if (!initialized || lastTimestampNs == 0) {
            pitchAngle = accelPitch - pitchOffset;
            lastTimestampNs = timestampNs;
            initialized = true;
            return;
        }

        float dt = (timestampNs - lastTimestampNs) / 1_000_000_000.0f;
        lastTimestampNs = timestampNs;

        if (dt <= 0 || dt > 0.5f) { // Sanity check dt
            dt = 0.01f;
        }

        // Fused pitch angle update using complementary filter
        float gyroPredictedPitch = pitchAngle + (pitchRate * dt);
        float measuredPitch = accelPitch - pitchOffset;

        pitchAngle = (alpha * gyroPredictedPitch) + ((1.0f - alpha) * measuredPitch);
    }

    public float getPitchAngle() {
        return pitchAngle;
    }

    public float getPitchRate() {
        return pitchRate;
    }
}
