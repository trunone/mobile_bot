package com.robot.balancingrobot;

import android.content.pm.PackageInfo;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import com.robot.balancingrobot.control.RobotBalanceController;
import com.robot.balancingrobot.input.GamepadController;
import com.robot.balancingrobot.sensor.RobotSensorManager;
import com.robot.balancingrobot.usb.UsbMcuManager;

import java.util.Locale;

public class MainActivity extends AppCompatActivity {

    private RobotSensorManager sensorManager;
    private RobotBalanceController balanceController;
    private GamepadController gamepadController;
    private UsbMcuManager usbMcuManager;

    private TextView tvAppVersion;
    private TextView tvUsbStatus;
    private TextView tvGamepadStatus;
    private TextView tvSystemState;
    private TextView tvPitchAngle;
    private TextView tvPitchRate;
    private TextView tvMotorSpeeds;
    private TextView tvDriveInputs;

    private TextView tvKpLabel;
    private TextView tvKiLabel;
    private TextView tvKdLabel;

    private SeekBar seekbarKp;
    private SeekBar seekbarKi;
    private SeekBar seekbarKd;

    private Button btnConnectUsb;
    private Button btnToggleEnable;
    private Button btnCalibrateZero;

    private long lastLoopTimeNs = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        initializeComponents();
        setupViews();
        setupListeners();
        displayAppVersion();
    }

    private void initializeComponents() {
        // Default PID gains: Kp = 35.0, Ki = 0.0, Kd = 2.5
        balanceController = new RobotBalanceController(35.0f, 0.0f, 2.5f);
        sensorManager = new RobotSensorManager(this);
        gamepadController = new GamepadController();
        usbMcuManager = new UsbMcuManager(this);
    }

    private void setupViews() {
        tvAppVersion = findViewById(R.id.tvAppVersion);
        tvUsbStatus = findViewById(R.id.tvUsbStatus);
        tvGamepadStatus = findViewById(R.id.tvGamepadStatus);
        tvSystemState = findViewById(R.id.tvSystemState);
        tvPitchAngle = findViewById(R.id.tvPitchAngle);
        tvPitchRate = findViewById(R.id.tvPitchRate);
        tvMotorSpeeds = findViewById(R.id.tvMotorSpeeds);
        tvDriveInputs = findViewById(R.id.tvDriveInputs);

        tvKpLabel = findViewById(R.id.tvKpLabel);
        tvKiLabel = findViewById(R.id.tvKiLabel);
        tvKdLabel = findViewById(R.id.tvKdLabel);

        seekbarKp = findViewById(R.id.seekbarKp);
        seekbarKi = findViewById(R.id.seekbarKi);
        seekbarKd = findViewById(R.id.seekbarKd);

        btnConnectUsb = findViewById(R.id.btnConnectUsb);
        btnToggleEnable = findViewById(R.id.btnToggleEnable);
        btnCalibrateZero = findViewById(R.id.btnCalibrateZero);

        updatePidUiValues();
    }

    private void displayAppVersion() {
        try {
            PackageInfo pInfo = getPackageManager().getPackageInfo(getPackageName(), 0);
            String version = pInfo.versionName;
            tvAppVersion.setText("Version: " + version);
        } catch (PackageManager.NameNotFoundException e) {
            tvAppVersion.setText("Version: Unknown");
        }
    }

    private void setupListeners() {
        btnConnectUsb.setOnClickListener(v -> {
            if (usbMcuManager.isConnected()) {
                usbMcuManager.disconnect();
            } else {
                usbMcuManager.connect();
            }
        });

        btnToggleEnable.setOnClickListener(v -> toggleEnableState());

        btnCalibrateZero.setOnClickListener(v -> {
            sensorManager.getPitchEstimator().calibrateZero();
            Toast.makeText(this, "Zero Pitch Calibrated!", Toast.LENGTH_SHORT).show();
        });

        usbMcuManager.setListener(new UsbMcuManager.UsbConnectionListener() {
            @Override
            public void onConnected(String deviceName) {
                runOnUiThread(() -> {
                    tvUsbStatus.setText("USB MCU: Connected (" + deviceName + ")");
                    tvUsbStatus.setTextColor(0xFF4CAF50);
                    btnConnectUsb.setText("Disconnect USB");
                });
            }

            @Override
            public void onDisconnected() {
                runOnUiThread(() -> {
                    tvUsbStatus.setText("USB MCU: Disconnected");
                    tvUsbStatus.setTextColor(0xFFFF5252);
                    btnConnectUsb.setText("Connect USB");
                });
            }

            @Override
            public void onError(String error) {
                runOnUiThread(() -> {
                    Toast.makeText(MainActivity.this, "USB Error: " + error, Toast.LENGTH_SHORT).show();
                });
            }
        });

        gamepadController.setListener(new GamepadController.GamepadEventListener() {
            @Override
            public void onDriveInputChanged(float forwardVelocity, float turnRate) {
                balanceController.setDriveInputs(forwardVelocity, turnRate);
                runOnUiThread(() -> {
                    tvDriveInputs.setText(String.format(Locale.US, "XInput Forward: %.2f | Turn: %.2f", forwardVelocity, turnRate));
                    updateGamepadStatus();
                });
            }

            @Override
            public void onEnableToggled() {
                runOnUiThread(() -> toggleEnableState());
            }

            @Override
            public void onEmergencyStop() {
                runOnUiThread(() -> {
                    balanceController.setEnabled(false);
                    updateSystemStateUi();
                    Toast.makeText(MainActivity.this, "EMERGENCY STOP", Toast.LENGTH_SHORT).show();
                });
            }
        });

        sensorManager.setSensorDataListener((pitchAngleDeg, pitchRateDegPerSec) -> {
            long nowNs = System.nanoTime();
            float dt = (lastLoopTimeNs == 0) ? 0.01f : (nowNs - lastLoopTimeNs) / 1_000_000_000.0f;
            lastLoopTimeNs = nowNs;

            RobotBalanceController.MotorOutputs outputs = balanceController.update(pitchAngleDeg, pitchRateDegPerSec, dt);

            if (usbMcuManager.isConnected()) {
                usbMcuManager.sendMotorCommand(outputs.leftMotor, outputs.rightMotor);
            }

            runOnUiThread(() -> {
                tvPitchAngle.setText(String.format(Locale.US, "Pitch Angle: %.2f°", pitchAngleDeg));
                tvPitchRate.setText(String.format(Locale.US, "Pitch Rate: %.2f deg/s", pitchRateDegPerSec));
                tvMotorSpeeds.setText(String.format(Locale.US, "Motors L / R: %d / %d", outputs.leftMotor, outputs.rightMotor));

                if (outputs.safetyTripped) {
                    updateSystemStateUi();
                }
            });
        });

        setupSeekBarListeners();
    }

    private void setupSeekBarListeners() {
        SeekBar.OnSeekBarChangeListener listener = new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                if (!fromUser) return;

                float kp = seekbarKp.getProgress();
                float ki = seekbarKi.getProgress() / 10.0f;
                float kd = seekbarKd.getProgress() / 10.0f;

                balanceController.getAnglePid().setGains(kp, ki, kd);
                updatePidUiValues();
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {}
        };

        seekbarKp.setOnSeekBarChangeListener(listener);
        seekbarKi.setOnSeekBarChangeListener(listener);
        seekbarKd.setOnSeekBarChangeListener(listener);
    }

    private void updatePidUiValues() {
        tvKpLabel.setText(String.format(Locale.US, "Kp: %.1f", balanceController.getAnglePid().getKp()));
        tvKiLabel.setText(String.format(Locale.US, "Ki: %.1f", balanceController.getAnglePid().getKi()));
        tvKdLabel.setText(String.format(Locale.US, "Kd: %.1f", balanceController.getAnglePid().getKd()));
    }

    private void toggleEnableState() {
        boolean newState = !balanceController.isEnabled();
        balanceController.setEnabled(newState);
        updateSystemStateUi();
    }

    private void updateSystemStateUi() {
        boolean enabled = balanceController.isEnabled();
        if (enabled) {
            tvSystemState.setText("Status: BALANCING ACTIVE");
            tvSystemState.setTextColor(0xFF4CAF50);
            btnToggleEnable.setText("DISABLE ROBOT");
            btnToggleEnable.setBackgroundColor(0xFFD32F2F);
        } else {
            tvSystemState.setText("Status: OFF / DISABLED");
            tvSystemState.setTextColor(0xFFBBBBBB);
            btnToggleEnable.setText("ENABLE ROBOT");
            btnToggleEnable.setBackgroundColor(0xFF388E3C);
        }
    }

    private void updateGamepadStatus() {
        if (gamepadController.isControllerConnected()) {
            tvGamepadStatus.setText("Gamepad: Connected (" + gamepadController.getControllerName() + ")");
            tvGamepadStatus.setTextColor(0xFF4CAF50);
        }
    }

    @Override
    public boolean onGenericMotionEvent(MotionEvent event) {
        if (gamepadController.handleMotionEvent(event)) {
            return true;
        }
        return super.onGenericMotionEvent(event);
    }

    @Override
    public boolean onKeyDown(int keyCode, KeyEvent event) {
        if (gamepadController.handleKeyEvent(event)) {
            return true;
        }
        return super.onKeyDown(keyCode, event);
    }

    @Override
    protected void onResume() {
        super.onResume();
        sensorManager.registerSensors();
    }

    @Override
    protected void onPause() {
        super.onPause();
        sensorManager.unregisterSensors();
        if (usbMcuManager.isConnected()) {
            usbMcuManager.sendMotorCommand(0, 0);
        }
    }
}
