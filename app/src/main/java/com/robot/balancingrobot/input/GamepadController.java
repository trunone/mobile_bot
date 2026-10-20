package com.robot.balancingrobot.input;

import android.view.InputDevice;
import android.view.KeyEvent;
import android.view.MotionEvent;

/**
 * Gamepad input handler compatible with XInput / Xbox Wireless Controllers on Android.
 * Extracts analog stick values (with deadzone filtering) and button presses for robot control.
 */
public class GamepadController {

    public interface GamepadEventListener {
        void onDriveInputChanged(float forwardVelocity, float turnRate);
        void onEnableToggled();
        void onEmergencyStop();
    }

    private GamepadEventListener listener;
    private float deadzone = 0.15f; // Analog deadzone threshold
    private float forwardInput = 0.0f; // -1.0 (backward) to 1.0 (forward)
    private float turnInput = 0.0f;    // -1.0 (left) to 1.0 (right)
    private boolean controllerConnected = false;
    private String controllerName = "";

    public void setListener(GamepadEventListener listener) {
        this.listener = listener;
    }

    public void setDeadzone(float deadzone) {
        this.deadzone = deadzone;
    }

    public float getDeadzone() {
        return deadzone;
    }

    public boolean isControllerConnected() {
        return controllerConnected;
    }

    public String getControllerName() {
        return controllerName;
    }

    public static boolean isGamepadDevice(InputDevice device) {
        if (device == null) return false;
        int sources = device.getSources();
        return (sources & InputDevice.SOURCE_GAMEPAD) == InputDevice.SOURCE_GAMEPAD
                || (sources & InputDevice.SOURCE_JOYSTICK) == InputDevice.SOURCE_JOYSTICK;
    }

    /**
     * Apply deadzone filtering and non-linear response to raw joystick input.
     */
    public static float applyDeadzone(float value, float deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0.0f;
        }
        // Rescale remaining range [deadzone, 1.0] to [0.0, 1.0]
        float sign = Math.signum(value);
        return sign * (Math.abs(value) - deadzone) / (1.0f - deadzone);
    }

    /**
     * Handles MotionEvent from joystick/thumbsticks.
     * XInput mapping:
     * - Left Stick Y / Right Trigger - Left Trigger: Forward/Backward
     * - Right Stick X or Left Stick X: Turning
     */
    public boolean handleMotionEvent(MotionEvent event) {
        if ((event.getSource() & InputDevice.SOURCE_JOYSTICK) == InputDevice.SOURCE_JOYSTICK
                && event.getAction() == MotionEvent.ACTION_MOVE) {

            InputDevice device = event.getDevice();
            if (device != null) {
                controllerConnected = true;
                controllerName = device.getName();
            }

            // Left Stick Y (Up is negative, so invert)
            float rawY = -event.getAxisValue(MotionEvent.AXIS_Y);
            // Right Stick X or Left Stick X for turn
            float rawTurnX = event.getAxisValue(MotionEvent.AXIS_Z);
            if (Math.abs(rawTurnX) < 0.05f) {
                rawTurnX = event.getAxisValue(MotionEvent.AXIS_X);
            }

            // Triggers: AXIS_RTRIGGER and AXIS_LTRIGGER for forward/reverse alternative
            float rTrigger = event.getAxisValue(MotionEvent.AXIS_RTRIGGER);
            float lTrigger = event.getAxisValue(MotionEvent.AXIS_LTRIGGER);
            float triggerDrive = rTrigger - lTrigger;

            float newForward = applyDeadzone(rawY, deadzone);
            if (Math.abs(triggerDrive) > Math.abs(newForward)) {
                newForward = applyDeadzone(triggerDrive, deadzone);
            }

            float newTurn = applyDeadzone(rawTurnX, deadzone);

            if (newForward != forwardInput || newTurn != turnInput) {
                forwardInput = newForward;
                turnInput = newTurn;

                if (listener != null) {
                    listener.onDriveInputChanged(forwardInput, turnInput);
                }
            }
            return true;
        }
        return false;
    }

    /**
     * Handles KeyEvent from Xbox / XInput controller buttons.
     * - Button A or START: Enable/Toggle balancing mode
     * - Button B or BACK or SELECT: Emergency stop / Disable
     */
    public boolean handleKeyEvent(KeyEvent event) {
        if (!isGamepadDevice(event.getDevice())) {
            return false;
        }

        if (event.getAction() == KeyEvent.ACTION_DOWN) {
            int keyCode = event.getKeyCode();
            if (keyCode == KeyEvent.KEYCODE_BUTTON_A || keyCode == KeyEvent.KEYCODE_BUTTON_START) {
                if (listener != null) {
                    listener.onEnableToggled();
                }
                return true;
            } else if (keyCode == KeyEvent.KEYCODE_BUTTON_B || keyCode == KeyEvent.KEYCODE_BUTTON_SELECT) {
                if (listener != null) {
                    listener.onEmergencyStop();
                }
                return true;
            }
        }
        return false;
    }

    public float getForwardInput() {
        return forwardInput;
    }

    public float getTurnInput() {
        return turnInput;
    }
}
