/*
Copyright (c) 2023 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.camera.huskylens;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;

@I2cDeviceType
@DeviceProperties(xmlTag = "huskylens", name = "HuskyLens2", description = "CamControl V2 Vision Sensor")
public class HuskyLens2 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    // Constants
    private static final int LCD_WIDTH = 640;
    private static final int LCD_HEIGHT = 480;

    private static final int HEADER_0_INDEX = 0;
    private static final int HEADER_1_INDEX = 1;
    private static final int COMMAND_INDEX = 2;
    private static final int ALGO_INDEX = 3;
    private static final int CONTENT_SIZE_INDEX = 4;
    private static final int CONTENT_INDEX = 5;
    private static final int PROTOCOL_SIZE = 6;

    private static final int COMMAND_KNOCK = 0x20;
    private static final int COMMAND_GET_RESULT = 0x21;
    private static final int COMMAND_GET_INFO = 0x22;
    private static final int COMMAND_GET_RESULT_BY_ID = 0x23;
    private static final int COMMAND_GET_BLOCKS_BY_ID = 0x24;
    private static final int COMMAND_GET_ARROWS_BY_ID = 0x25;
    private static final int COMMAND_GET_SENSOR_LIST = 0x26;
    private static final int COMMAND_GET_RESULT_BY_INDEX = 0x27;
    private static final int COMMAND_GET_BLOCKS_BY_INDEX = 0x28;
    private static final int COMMAND_GET_ARROWS_BY_INDEX = 0x29;

    private static final int COMMAND_SET_ALGORITHM = 0x30;
    private static final int COMMAND_SET_NAME_BY_ID = 0x31;
    private static final int COMMAND_SET_MULTI_ALGORITHM = 0x32;
    private static final int COMMAND_SET_MULTI_ALGORITHM_RATIO = 0x33;
    private static final int COMMAND_SET_LEARN_BLOCK_POSITION = 0x34;

    private static final int COMMAND_RETURN_OK = 0x40;
    private static final int COMMAND_RETURN_ERROR = 0x41;
    private static final int COMMAND_RETURN_INFO = 0x42;
    private static final int COMMAND_RETURN_BLOCK = 0x43;
    private static final int COMMAND_RETURN_ARROW = 0x44;
    private static final int COMMAND_RETURN_SENSOR_LIST = 0x45;

    private static final int COMMAND_ACTION_TAKE_PHOTO = 0x50;
    private static final int COMMAND_ACTION_TAKE_SCREENSHOT = 0x51;
    private static final int COMMAND_ACTION_LEARN = 0x52;
    private static final int COMMAND_ACTION_FORGOT = 0x53;
    private static final int COMMAND_ACTION_SAVE_KNOWLEDGES = 0x54;
    private static final int COMMAND_ACTION_LOAD_KNOWLEDGES = 0x55;
    private static final int COMMAND_ACTION_DRAW_RECT = 0x56;
    private static final int COMMAND_ACTION_CLEAN_RECT = 0x57;
    private static final int COMMAND_ACTION_DRAW_TEXT = 0x58;
    private static final int COMMAND_ACTION_CLEAR_TEXT = 0x59;
    private static final int COMMAND_ACTION_PLAY_MUSIC = 0x5A;

    public static final int ALGORITHM_ANY = 0;
    public static final int ALGORITHM_FACE_RECOGNITION = 1;
    public static final int ALGORITHM_OBJECT_TRACKING = 2;
    public static final int ALGORITHM_OBJECT_RECOGNITION = 3;
    public static final int ALGORITHM_LINE_TRACKING = 4;
    public static final int ALGORITHM_COLOR_RECOGNITION = 5;
    public static final int ALGORITHM_TAG_RECOGNITION = 6;
    public static final int ALGORITHM_SELF_LEARNING_CLASSIFICATION = 7;
    public static final int ALGORITHM_OCR_RECOGNITION = 8;
    public static final int ALGORITHM_LICENSE_RECOGNITION = 9;
    public static final int ALGORITHM_QRCODE_RECOGNITION = 10;
    public static final int ALGORITHM_BARCODE_RECOGNITION = 11;
    public static final int ALGORITHM_EMOTION_RECOGNITION = 12;
    public static final int ALGORITHM_POSE_RECOGNITION = 13;
    public static final int ALGORITHM_HAND_RECOGNITION = 14;
    public static final int ALGORITHM_OBJECT_CLASSIFICATION = 15;
    public static final int ALGORITHM_BLINK_RECOGNITION = 16;
    public static final int ALGORITHM_GAZE_RECOGNITION = 17;
    public static final int ALGORITHM_FACE_ORIENTATION = 18;
    public static final int ALGORITHM_FALLDOWN_RECOGNITION = 19;
    public static final int ALGORITHM_SEGMENT = 20;
    public static final int ALGORITHM_FACE_ACTION_RECOGNITION = 21;
    public static final int ALGORITHM_CUSTOM0 = 22;
    public static final int ALGORITHM_CUSTOM1 = 23;
    public static final int ALGORITHM_CUSTOM2 = 24;
    public static final int ALGORITHM_BUILTIN_COUNT = 25;
    public static final int ALGORITHM_CUSTOM_BEGIN = 128;
    public static final int DEFAULT_I2C_ADDR = 0x50;

    private int receive_index = HEADER_0_INDEX;
    private byte[] receive_buffer = new byte[1024];
    private byte[] send_buffer = new byte[128];
    private int send_index;
    private final Map<Integer, AlgorithmResult> result = new HashMap<>();
    private final Queue<Byte> i2cBuffer = new LinkedList<>();

    public HuskyLens2(I2cDeviceSynch deviceClient) {
        this(deviceClient, DEFAULT_I2C_ADDR);
    }

    public HuskyLens2(I2cDeviceSynch deviceClient, int i2cAddress) {
        super(deviceClient, true);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(i2cAddress));
    }

    /**
     * (Re)initializes the connection to the HuskyLens using the current I2C address.
     * @return true if the knock was successful, false otherwise.
     */
    public boolean init() {
        return knock();
    }

    /**
     * Sets a new I2C address for the HuskyLens and verifies the connection.
     * This is useful if the HuskyLens is not at the default address and initialization failed.
     * @param i2cAddress The new 7-bit I2C address for the HuskyLens.
     * @return true if the knock was successful, false otherwise.
     */
    public boolean init(int i2cAddress) {
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(i2cAddress));
        return knock();
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.DFRobot;
    }

    @Override
    protected synchronized boolean doInitialize() {
        return knock();
    }

    @Override
    public String getDeviceName() {
        return "CamControl V2";
    }

    private int checksum(byte[] cmd) {
        int cs = 0;
        for (byte x : cmd) {
            cs += (x & 0xFF);
        }
        return cs & 0xff;
    }

    private void writeToHuskyLens() {
        byte[] command = new byte[send_index];
        System.arraycopy(send_buffer, 0, command, 0, send_index);
        deviceClient.write(command);
    }

    private Byte readFromHuskyLens() {
        if (i2cBuffer.isEmpty()) {
            byte[] data = deviceClient.read(32);
            for (byte b : data) {
                i2cBuffer.add(b);
            }
        }
        return i2cBuffer.poll();
    }

    public boolean knock() {
        huskyLensProtocolWriteBegin(ALGORITHM_ANY, COMMAND_KNOCK);
        huskyLensProtocolWriteEnd();
        return executeCommand(COMMAND_RETURN_OK);
    }
    
    private void huskyLensProtocolWriteBegin(int algo, int command) {
        send_buffer = new byte[128];
        send_buffer[HEADER_0_INDEX] = (byte) 0x55;
        send_buffer[HEADER_1_INDEX] = (byte) 0xAA;
        send_buffer[COMMAND_INDEX] = (byte) command;
        send_buffer[ALGO_INDEX] = (byte) algo;
        send_index = CONTENT_INDEX;
    }

    private void huskyLensProtocolWriteEnd() {
        send_buffer[CONTENT_SIZE_INDEX] = (byte) (send_index - CONTENT_INDEX);
        int cs = 0;
        for (int i = 0; i < send_index; i++) {
            cs += (send_buffer[i] & 0xFF);
        }
        send_buffer[send_index] = (byte) (cs & 0xFF);
        send_index++;
    }

    private boolean wait(int command) {
        receive_buffer = new byte[1024];
        receive_index = HEADER_0_INDEX;
        long startTime = System.currentTimeMillis();
        boolean receiving = true;
        while (receiving) {
            if (System.currentTimeMillis() - startTime > 500) {
                break;
            }
            Byte c = readFromHuskyLens();
            if (c == null) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                continue;
            }
            if (huskyLensProtocolReceive(c)) {
                receiving = false;
            }
        }
        return !receiving && receive_buffer[COMMAND_INDEX] == command;
    }

    private boolean huskyLensProtocolReceive(byte data) {
        switch (receive_index) {
            case HEADER_0_INDEX:
                if (data != (byte) 0x55) {
                    return false;
                }
                break;
            case HEADER_1_INDEX:
                if (data != (byte) 0xAA) {
                    receive_index = HEADER_0_INDEX;
                    return false;
                }
                break;
        }

        receive_buffer[receive_index] = data;

        if (receive_index >= CONTENT_INDEX && receive_index == (receive_buffer[CONTENT_SIZE_INDEX] & 0xFF) + CONTENT_INDEX) {
            int cs = checksum(Arrays.copyOf(receive_buffer, receive_index));
            return (cs & 0xFF) == (receive_buffer[receive_index] & 0xFF);
        }

        receive_index++;
        return false;
    }

    private boolean executeCommand(int waitCmd) {
        for (int i = 0; i < 3; i++) {
            writeToHuskyLens();
            if (wait(waitCmd)) {
                return true;
            }
        }
        return false;
    }
    
    public class AlgorithmResult {
        public int algo;
        public Result info;
        public List<Result> blocks = new ArrayList<>();

        public AlgorithmResult(int algo) {
            this.algo = algo;
        }
    }
    
    public class Result {
        public int algo;
        public int dataLength;
        public int nameLength;
        public int contentLength;
        public int ID;
        public int level;
        public int first;
        public int second;
        public int third;
        public int fourth;
        public String name = "";
        public String content = "";
        public boolean used = false;
        public int xCenter, yCenter, width, height; // For block-like results
        public int xTarget, yTarget, angle, length; // For arrow-like results
        public int total_results, total_results_learned, total_blocks, total_blocks_learned, total_arrows, total_arrows_learned;

        public Result(byte[] buf) {
            this.algo = buf[ALGO_INDEX] & 0xFF;
            this.dataLength = buf[CONTENT_SIZE_INDEX] & 0xFF;
            if (this.dataLength > 10) {
                this.nameLength = buf[CONTENT_INDEX + 10] & 0xFF;
                if (buf.length > CONTENT_INDEX + 11 + this.nameLength) {
                    this.contentLength = buf[CONTENT_INDEX + 11 + this.nameLength] & 0xFF;
                }
            }
            this.ID = buf[CONTENT_INDEX] & 0xFF;
            this.level = buf[CONTENT_INDEX + 1] & 0xFF;
            this.first = (buf[CONTENT_INDEX + 2] & 0xFF) | ((buf[CONTENT_INDEX + 3] & 0xFF) << 8);
            this.second = (buf[CONTENT_INDEX + 4] & 0xFF) | ((buf[CONTENT_INDEX + 5] & 0xFF) << 8);
            this.third = (buf[CONTENT_INDEX + 6] & 0xFF) | ((buf[CONTENT_INDEX + 7] & 0xFF) << 8);
            this.fourth = (buf[CONTENT_INDEX + 8] & 0xFF) | ((buf[CONTENT_INDEX + 9] & 0xFF) << 8);

            if (nameLength > 0) {
                this.name = new String(Arrays.copyOfRange(buf, CONTENT_INDEX + 11, CONTENT_INDEX + 11 + nameLength), StandardCharsets.UTF_8);
            }
            if (contentLength > 0) {
                this.content = new String(Arrays.copyOfRange(buf, CONTENT_INDEX + 11 + nameLength, CONTENT_INDEX + 11 + nameLength + contentLength), StandardCharsets.UTF_8);
            }
        }
    }
}
