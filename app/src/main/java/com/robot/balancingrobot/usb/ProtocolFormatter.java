package com.robot.balancingrobot.usb;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Utility for formatting motor command packets for transmission over USB OTG serial.
 * Supports both ASCII text framing and binary packet framing with checksum verification.
 */
public class ProtocolFormatter {

    public enum ProtocolType {
        ASCII,  // e.g. "M,leftSpeed,rightSpeed\n"
        BINARY  // 0xAA 0x55 [int16_t left] [int16_t right] [uint8_t checksum] 0x0D 0x0A
    }

    /**
     * Format motor speeds as ASCII packet string bytes.
     * Output format: "M,<leftSpeed>,<rightSpeed>\n"
     */
    public static byte[] formatAsciiPacket(int leftSpeed, int rightSpeed) {
        String command = String.format("M,%d,%d\n", leftSpeed, rightSpeed);
        return command.getBytes(java.nio.charset.StandardCharsets.US_ASCII);
    }

    /**
     * Format motor speeds into binary protocol packet.
     * Header: 0xAA, 0x55
     * Payload: Left Motor (16-bit signed integer, Big Endian), Right Motor (16-bit signed integer, Big Endian)
     * Checksum: XOR of payload bytes
     * Tail: 0x0D (\r), 0x0A (\n)
     * Total length: 9 bytes
     */
    public static byte[] formatBinaryPacket(int leftSpeed, int rightSpeed) {
        ByteBuffer buffer = ByteBuffer.allocate(9);
        buffer.order(ByteOrder.BIG_ENDIAN);

        buffer.put((byte) 0xAA);
        buffer.put((byte) 0x55);

        short left = (short) Math.max(-32768, Math.min(32767, leftSpeed));
        short right = (short) Math.max(-32768, Math.min(32767, rightSpeed));

        buffer.putShort(left);
        buffer.putShort(right);

        byte checksum = (byte) ((left >> 8) ^ (left & 0xFF) ^ (right >> 8) ^ (right & 0xFF));
        buffer.put(checksum);

        buffer.put((byte) 0x0D);
        buffer.put((byte) 0x0A);

        return buffer.array();
    }

    public static byte[] formatPacket(ProtocolType type, int leftSpeed, int rightSpeed) {
        if (type == ProtocolType.BINARY) {
            return formatBinaryPacket(leftSpeed, rightSpeed);
        } else {
            return formatAsciiPacket(leftSpeed, rightSpeed);
        }
    }
}
