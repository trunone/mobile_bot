package com.robot.balancingrobot.usb;

import android.content.Context;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.util.Log;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;

import java.io.IOException;
import java.util.List;

/**
 * Manages USB OTG communication with microcontroller (MCU) over USB Serial (CDC, FTDI, CH340, CP2102).
 */
public class UsbMcuManager {

    private static final String TAG = "UsbMcuManager";
    private static final int DEFAULT_BAUD_RATE = 115200;
    private static final int WRITE_TIMEOUT_MS = 200;

    private final Context context;
    private final UsbManager usbManager;
    private UsbSerialPort serialPort;
    private UsbDeviceConnection connection;
    private boolean isConnected = false;
    private ProtocolFormatter.ProtocolType protocolType = ProtocolFormatter.ProtocolType.ASCII;

    public interface UsbConnectionListener {
        void onConnected(String deviceName);
        void onDisconnected();
        void onError(String error);
    }

    private UsbConnectionListener listener;

    public UsbMcuManager(Context context) {
        this.context = context;
        this.usbManager = (UsbManager) context.getSystemService(Context.USB_SERVICE);
    }

    public void setListener(UsbConnectionListener listener) {
        this.listener = listener;
    }

    public void setProtocolType(ProtocolFormatter.ProtocolType protocolType) {
        this.protocolType = protocolType;
    }

    public ProtocolFormatter.ProtocolType getProtocolType() {
        return protocolType;
    }

    public boolean isConnected() {
        return isConnected;
    }

    /**
     * Attempts to find and open the first connected USB OTG serial device.
     */
    public boolean connect() {
        return connect(DEFAULT_BAUD_RATE);
    }

    public boolean connect(int baudRate) {
        if (usbManager == null) {
            if (listener != null) listener.onError("USB Manager service not available");
            return false;
        }

        List<UsbSerialDriver> availableDrivers = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager);
        if (availableDrivers.isEmpty()) {
            if (listener != null) listener.onError("No USB Serial devices found");
            return false;
        }

        UsbSerialDriver driver = availableDrivers.get(0);
        UsbDevice device = driver.getDevice();

        if (!usbManager.hasPermission(device)) {
            if (listener != null) listener.onError("USB permission missing for device " + device.getDeviceName());
            return false;
        }

        connection = usbManager.openDevice(device);
        if (connection == null) {
            if (listener != null) listener.onError("Failed to open USB connection");
            return false;
        }

        List<UsbSerialPort> ports = driver.getPorts();
        if (ports.isEmpty()) {
            if (listener != null) listener.onError("No USB serial ports found on device");
            connection.close();
            connection = null;
            return false;
        }

        serialPort = ports.get(0);
        try {
            serialPort.open(connection);
            serialPort.setParameters(baudRate, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
            isConnected = true;
            if (listener != null) {
                listener.onConnected(device.getProductName() != null ? device.getProductName() : device.getDeviceName());
            }
            return true;
        } catch (IOException e) {
            Log.e(TAG, "Error opening USB serial port: " + e.getMessage(), e);
            disconnect();
            if (listener != null) listener.onError("Port open error: " + e.getMessage());
            return false;
        }
    }

    /**
     * Send left and right motor speed commands to MCU over USB OTG serial port.
     * @param leftSpeed Left motor speed
     * @param rightSpeed Right motor speed
     * @return true if write succeeded, false otherwise
     */
    public boolean sendMotorCommand(int leftSpeed, int rightSpeed) {
        if (!isConnected || serialPort == null) {
            return false;
        }

        byte[] data = ProtocolFormatter.formatPacket(protocolType, leftSpeed, rightSpeed);
        try {
            serialPort.write(data, WRITE_TIMEOUT_MS);
            return true;
        } catch (IOException e) {
            Log.e(TAG, "Failed to send motor command over USB: " + e.getMessage());
            disconnect();
            if (listener != null) listener.onError("Write failed: " + e.getMessage());
            return false;
        }
    }

    public void disconnect() {
        isConnected = false;
        if (serialPort != null) {
            try {
                serialPort.close();
            } catch (IOException ignored) {}
            serialPort = null;
        }
        if (connection != null) {
            connection.close();
            connection = null;
        }
        if (listener != null) {
            listener.onDisconnected();
        }
    }
}
