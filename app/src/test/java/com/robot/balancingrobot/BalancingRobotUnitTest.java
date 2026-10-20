package com.robot.balancingrobot;

import com.robot.balancingrobot.control.PidController;
import com.robot.balancingrobot.control.RobotBalanceController;
import com.robot.balancingrobot.input.GamepadController;
import com.robot.balancingrobot.sensor.PitchEstimator;
import com.robot.balancingrobot.usb.ProtocolFormatter;

import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class BalancingRobotUnitTest {

    private PidController pid;
    private RobotBalanceController balanceController;
    private PitchEstimator pitchEstimator;

    @Before
    public void setUp() {
        pid = new PidController(10.0f, 1.0f, 0.5f);
        balanceController = new RobotBalanceController(20.0f, 0.0f, 1.0f);
        pitchEstimator = new PitchEstimator(0.98f);
    }

    @Test
    public void testPidControllerOutput() {
        pid.setOutputLimits(-100.0f, 100.0f);
        // Error = setpoint(0.0) - processValue(5.0) = -5.0
        float output = pid.update(5.0f, 0.0f, 0.1f, Float.NaN);
        // Kp * error = 10 * (-5) = -50
        // Ki * integral = 1 * (-0.5) = -0.5
        // Kd * derivative = 0.5 * ((-5 - (-5)) / 0.1) = 0
        assertEquals(-50.5f, output, 0.1f);
    }

    @Test
    public void testPitchEstimatorAccelPitchVertical() {
        // Vertical portrait upright position: gravity is along -Y (ay = -9.81)
        float pitchUpright = PitchEstimator.calculateAccelPitch(0.0f, -9.81f, 0.0f);
        assertEquals(0.0f, pitchUpright, 0.001f);

        // Tilt forward 30 degrees: gravity component shifts to +Z
        float ay = (float) (-9.81 * Math.cos(Math.toRadians(30)));
        float az = (float) (9.81 * Math.sin(Math.toRadians(30)));
        float pitchTilted = PitchEstimator.calculateAccelPitch(0.0f, ay, az);
        assertEquals(30.0f, pitchTilted, 0.1f);
    }

    @Test
    public void testPitchEstimatorComplementaryFilterUpdate() {
        long t0 = 1_000_000_000L; // 1s
        pitchEstimator.update(t0, 0.0f, -9.81f, 0.0f, 0.0f, true);
        assertEquals(0.0f, pitchEstimator.getPitchAngle(), 0.01f);

        // 0.1s later with gyro rate 10 deg/s (0.1745 rad/s)
        long t1 = t0 + 100_000_000L;
        float gyroRadPerSec = (float) Math.toRadians(10.0);
        pitchEstimator.update(t1, 0.0f, -9.81f, 0.0f, gyroRadPerSec, true);

        // Fused pitch should reflect small gyro integration
        assertTrue(pitchEstimator.getPitchAngle() > 0.0f);
    }

    @Test
    public void testBalanceControllerSafetyCutoff() {
        balanceController.setEnabled(true);
        balanceController.setMaxTiltLimitDeg(30.0f);

        // Small pitch angle: motors active
        RobotBalanceController.MotorOutputs normal = balanceController.update(5.0f, 0.0f, 0.01f);
        assertFalse(normal.safetyTripped);
        assertTrue(balanceController.isEnabled());

        // Overshoot pitch angle (> 30 deg): safety trip
        RobotBalanceController.MotorOutputs trip = balanceController.update(35.0f, 0.0f, 0.01f);
        assertTrue(trip.safetyTripped);
        assertFalse(balanceController.isEnabled());
        assertEquals(0, trip.leftMotor);
        assertEquals(0, trip.rightMotor);
    }

    @Test
    public void testGamepadDeadzone() {
        assertEquals(0.0f, GamepadController.applyDeadzone(0.05f, 0.15f), 0.001f);
        assertEquals(0.0f, GamepadController.applyDeadzone(-0.10f, 0.15f), 0.001f);

        // Above deadzone 0.15 -> rescaled
        float processed = GamepadController.applyDeadzone(0.575f, 0.15f);
        assertEquals(0.5f, processed, 0.01f);
    }

    @Test
    public void testProtocolFormatterAscii() {
        byte[] asciiData = ProtocolFormatter.formatAsciiPacket(250, -250);
        String expected = "M,250,-250\n";
        assertArrayEquals(expected.getBytes(java.nio.charset.StandardCharsets.US_ASCII), asciiData);
    }

    @Test
    public void testProtocolFormatterBinary() {
        byte[] binaryData = ProtocolFormatter.formatBinaryPacket(100, -100);
        assertEquals(9, binaryData.length);
        assertEquals((byte) 0xAA, binaryData[0]);
        assertEquals((byte) 0x55, binaryData[1]);
        assertEquals((byte) 0x0D, binaryData[7]);
        assertEquals((byte) 0x0A, binaryData[8]);
    }
}
