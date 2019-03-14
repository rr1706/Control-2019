package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;

import java.nio.BufferUnderflowException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

// Side Wall ~ 13 in
// Front Wall ~ 5 in

public  class Lidar {
    private static I2C I2CBus = new I2C(I2C.Port.kOnboard, 0x08);
    private static byte[] dataBuffer = new byte[14];
    private static ByteBuffer compBuffer = ByteBuffer.wrap(dataBuffer);

    private static double sensor1 = 0.0;
    private static double sensor2 = 0.0;
    private static double sensor3 = 0.0;
    private static double sensor4 = 0.0;
    private static double sensor5 = 0.0;
    private static double sensor6 = 0.0;
    private static double sensor7 = 0.0;

    // Reads in mm. Converts to in
    public static void read() {

        compBuffer.rewind();
        I2CBus.transaction(new byte[0], 0, dataBuffer, 14);
        compBuffer.order(ByteOrder.BIG_ENDIAN);

        try {
            sensor1 = compBuffer.getShort() * 0.0394; //Front Right Wheel, Right Sensor
            sensor2 = compBuffer.getShort() * 0.0394; //Front Right Wheel, Front Sensor
            sensor3 = compBuffer.getShort() * 0.0394; //Front Left Wheel, Front Sensor
            sensor4 = compBuffer.getShort() * 0.0394; //Front Left Wheel, Left Sensor

            sensor5 = compBuffer.getShort() * 0.0394; //Back Left Wheel, Back Sensor
            sensor6 = compBuffer.getShort() * 0.0394; //Back Right Wheel, Back Sensor
            sensor7 = compBuffer.getShort() * 0.0394; //Cargo Intake Sensor

        } catch (BufferUnderflowException e) {
            System.err.println("Not enough data in the lidar buffer");
        }
    }

    public static double getFRRightSensor(){
        return sensor1;
    }

    public static double getFRFrontSensor(){
        return sensor2;
    }

    public static double getFLFrontSensor(){
        return sensor3;
    }

    public static double getFLLeftSensor(){
        return sensor4;
    }

    public static double getBLLeftSensor(){
        return sensor5;
    }

    public static double getBRRightSensor(){
        return sensor6;
    }

    public static double hasCargo(){
        return sensor7;
    }

}
