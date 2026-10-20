package com.robot.balancingrobot.sensor;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

/**
 * Manages phone internal IMU sensors (Accelerometer and Gyroscope) for sensing orientation
 * when mounted vertically on the robot.
 */
public class RobotSensorManager implements SensorEventListener {

    public interface SensorDataListener {
        void onPitchUpdated(float pitchAngleDeg, float pitchRateDegPerSec);
    }

    private final SensorManager sensorManager;
    private final Sensor accelerometer;
    private final Sensor gyroscope;
    private final PitchEstimator pitchEstimator;
    private SensorDataListener listener;

    private float[] latestAccel = new float[3];
    private boolean hasAccel = false;

    public RobotSensorManager(Context context) {
        this.sensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        if (sensorManager != null) {
            this.accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
            this.gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        } else {
            this.accelerometer = null;
            this.gyroscope = null;
        }
        this.pitchEstimator = new PitchEstimator();
    }

    public PitchEstimator getPitchEstimator() {
        return pitchEstimator;
    }

    public void setSensorDataListener(SensorDataListener listener) {
        this.listener = listener;
    }

    public boolean registerSensors() {
        return registerSensors(SensorManager.SENSOR_DELAY_FASTEST);
    }

    public boolean registerSensors(int samplingPeriodUs) {
        if (sensorManager == null) return false;
        boolean accelOk = false;
        boolean gyroOk = false;

        if (accelerometer != null) {
            accelOk = sensorManager.registerListener(this, accelerometer, samplingPeriodUs);
        }
        if (gyroscope != null) {
            gyroOk = sensorManager.registerListener(this, gyroscope, samplingPeriodUs);
        }
        return accelOk && gyroOk;
    }

    public void unregisterSensors() {
        if (sensorManager != null) {
            sensorManager.unregisterListener(this);
        }
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            System.arraycopy(event.values, 0, latestAccel, 0, 3);
            hasAccel = true;
        } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            if (!hasAccel) return;

            // Phone vertical portrait orientation:
            // Gyro X = pitch angular velocity in rad/s
            float gyroX = event.values[0];
            pitchEstimator.update(event.timestamp, latestAccel[0], latestAccel[1], latestAccel[2], gyroX, true);

            if (listener != null) {
                listener.onPitchUpdated(pitchEstimator.getPitchAngle(), pitchEstimator.getPitchRate());
            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // No-op
    }
}
