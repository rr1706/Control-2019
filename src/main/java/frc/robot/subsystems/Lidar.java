package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;

import java.nio.BufferUnderflowException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

// Side Wall ~ 13 in
// Front Wall ~ 5 in

public  class Lidar {
    private static I2C I2CBus = new I2C(I2C.Port.kOnboard, 0x08);
    private static byte[] dataBuffer = new byte[8];
    private static ByteBuffer compBuffer = ByteBuffer.wrap(dataBuffer);

    private static double sensor1 = 0.0;
    private static double sensor2 = 0.0;
    private static double sensor3 = 0.0;
    private static double sensor4 = 0.0;

    // Reads in mm. Converts to in
    public static void read() {

        compBuffer.rewind();
        I2CBus.transaction(new byte[0], 0, dataBuffer, 8);
        compBuffer.order(ByteOrder.BIG_ENDIAN);

        try {
            sensor1 = compBuffer.getShort() * 0.0394;
            sensor2 = compBuffer.getShort() * 0.0394;
            sensor3 = compBuffer.getShort() * 0.0394;
            sensor4 = compBuffer.getShort() * 0.0394;
        } catch (BufferUnderflowException e) {
            System.err.println("Not enough data in the lidar buffer");
        }
    }

    public static double getRightSide(){
        return sensor1;
    }

    public static double getRightFront(){
        return sensor2;
    }

    public static double getLeftFront(){
        return sensor3;
    }

    public static double getLeftSide(){
        return sensor4;
    }
}
