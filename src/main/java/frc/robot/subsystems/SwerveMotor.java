//package frc.robot.subsystems;
//
//import com.revrobotics.*;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Ds;
////import frc.robot.RRLogger;
//import frc.robot.utilities.MathUtils;
//
//class SwerveMotor {
//    private static final int CAN_TIMEOUT = 20;
//    private static final double SMALL_NUMBER = 0.015; //Was 0.05
//    private static final double MAX_RPM = 5555/*3500*/ /*80*(Ds.getBatteryVoltage()-8)*/; //Todo: Bad equation, fix later
//
////    private double[] moduleDrift;
//
//    private CANSparkMax motor;
//
//    private CANEncoder motorEncoder;
//
//    private CANPIDController motorPID;
//
//    private final double THEORETICAL_MAX = 5676;
//    private double kMaxOutput = 0.9;
//    private double kMinOutput = -0.9;
//
//    private double motorP = 1.651e-4;
//    private double motorI = 0.9e-6;
//    private double motorD = 0.9e-6;
//    private double motorF = 0.93/THEORETICAL_MAX;
//
//
////    private double lastValidVelocity1 = 0.0;
////    private double lastValidVelocity2 = 0.0;
//
//    private double lastValidDistanceClockwise = 0.0;
//    private double lastValidDistanceCounter = 0.0;
//
//    int zeros = 0;
//    int goods = 0;
//
//    private int id;
//
//    /**
//     *
//     * @param canPortC Port of the motor that moves the wheel Clockwise
//     * @param canPortCC Port of the motor that moves the wheel CounterClockwise
//     */
//    SwerveMotor(int canPortC, int canPortCC) {
//
//        motor = new CANSparkMax(canPortC, CANSparkMaxLowLevel.MotorType.kBrushless);
//
//        motor.setCANTimeout(CAN_TIMEOUT);
//
//
//        motorEncoder = new CANEncoder(motor);
//
//        motor.setInverted(false);
//
//        motorPID = motor.getPIDController();
//
//        motorPID.setP(motorP);
//        motorPID.setI(motorI);
//        motorPID.setD(motorD);
//        motorPID.setIZone(0.0);
//        motorPID.setFF(motorF);
//        motorPID.setOutputRange(kMinOutput, kMaxOutput);
//
////        motor.setSmartCurrentLimit(30, 30);
//
//        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 5); //5 is the good value
//
//        motor.burnFlash();
//    }
//
//    /**
//     *
//     * @param speedCommand Moves wheel forward
//     * @param rotationCommand Moves wheel clockwise
//     */
//    void set(double speedCommand, double rotationCommand) {
//        double clockwiseCommand = speedCommand + rotationCommand;
//        double counterCommand = speedCommand - rotationCommand;
////        double clockwiseOvershoot = Math.abs(clockwiseCommand) - 1;
////        double counterOvershoot = Math.abs(counterCommand) - 1;
////
////        if (clockwiseOvershoot > 0) {
////            clockwiseCommand -= clockwiseOvershoot * Math.signum(clockwiseCommand);
////            counterCommand -= clockwiseOvershoot * Math.signum(clockwiseCommand);
////        }
////
////        if (counterOvershoot > 0) {
////            counterCommand -= counterOvershoot * Math.signum(counterCommand);
////            clockwiseCommand -= counterOvershoot * Math.signum(counterCommand);
////        }
//
//
////        if (clockwiseCommand != 0.0 && clockwiseEncoder.getVelocity() != 0.0 || clockwiseCommand == 0.0) {
////            lastValidVelocity1 = clockwiseEncoder.getVelocity();
////        }
////
////        if (counterCommand != 0.0 && counterEncoder.getVelocity() != 0.0 || counterCommand == 0.0) {
////            lastValidVelocity2 = counterEncoder.getVelocity();
////        }
//
////        if (id == 2) {
////            System.out.println((MathUtils.resolveDeg((lastValidDistanceClockwise + lastValidDistanceCounter)*36.0)) + "| |" + clockwiseEncoder.getPosition() + "| |" + counterEncoder.getPosition());
//
////            SmartDashboard.putNumber("Wheel Temp", clockwiseMotor.getMotorTemperature());
//////            SmartDashboard.putNumber("Front Right Command", lastValidDistanceClockwise);
//////            SmartDashboard.putNumber("Front Right Velocity", clockwiseEncoder.getVelocity());
////            System.out.println(MathUtils.resolveDeg((lastValidDistanceClockwise + lastValidDistanceCounter)*36.0) + "||" + (MAX_RPM*clockwiseCommand) +"||" + (MAX_RPM*counterCommand )+ "||" + clockwiseEncoder.getVelocity() + "||" + counterEncoder.getVelocity());
////        }
////        else if (id == 2) {
////            SmartDashboard.putNumber("Front Left Ticks", lastValidDistanceClockwise);
////            SmartDashboard.putNumber("Front Left Velocity", clockwiseEncoder.getVelocity());
////        } else if (id == 3) {
////            SmartDashboard.putNumber("Back Right Ticks", lastValidDistanceClockwise);
////            SmartDashboard.putNumber("Back Right Velocity", clockwiseEncoder.getVelocity());
////        } else {
////            SmartDashboard.putNumber("Back Left Ticks", lastValidDistanceClockwise);
////            SmartDashboard.putNumber("Back Left Velocity", clockwiseEncoder.getVelocity());
////            System.out.println((lastValidDistanceCounter+lastValidDistanceClockwise)*36.0  + "| |" + lastValidDistanceClockwise + "||" + lastValidDistanceCounter + "||" + clockwiseMotor.getAppliedOutput() + "||" + counterMotor.getAppliedOutput());
////            System.out.println((lastValidDistanceCounter+lastValidDistanceClockwise)*36.0 + "| |" + (clockwiseEncoder.getVelocity()+counterEncoder.getVelocity()) + "| |" + clockwiseEncoder.getVelocity() + "| | " + counterEncoder.getVelocity());
////            System.out.println(clockwiseCommand*MAX_RPM + " | | " + clockwiseEncoder.getVelocity());
//
////            RRLogger.addData("Clockwise Motor Command", clockwiseCommand*MAX_RPM);
////            RRLogger.addData("Counter Motor Command", counterCommand*MAX_RPM);
////            RRLogger.addData("Clockwise Encoder Position", lastValidDistanceClockwise);
////            RRLogger.addData("Counter Encoder Position", lastValidDistanceCounter);
////            RRLogger.addData("Clockwise Encoder Velocity", clockwiseEncoder.getVelocity());
////            RRLogger.addData("Counter Encoder Velocity", counterEncoder.getVelocity());
////        }
//
////        if ((rotationCommand == 0.0) && (clockwiseEncoder.getVelocity() + counterEncoder.getVelocity() > 15.0)) {
////            System.out.println("id:" + id + " | " + clockwiseEncoder.getVelocity() + "| | " + counterEncoder.getVelocity() + " | | " + (clockwiseEncoder.getVelocity()+counterEncoder.getVelocity()));
////        }
//
//
//        if (Math.abs(clockwiseCommand) > SMALL_NUMBER) {
//            clockwiseCommand*=MAX_RPM;
////            clockwiseCommand = 0.0;
////            clockwisePID.setReference(clockwiseCommand, ControlType.kVelocity);
////            clockwisePID.setReference(clockwiseCommand, ControlType.kVoltage);
////            clockwiseMotor.set(clockwiseCommand);
//        } else {
////            clockwiseMotor.stopMotor();
//        }
//
//        if (Math.abs(counterCommand) > SMALL_NUMBER) {
//            counterCommand*=MAX_RPM;
////            counterCommand = 0.2*MAX_RPM;
//            counterPID.setReference(-counterCommand, ControlType.kVelocity);
////            counterPID.setReference(-counterCommand, ControlType.kVoltage);
////            counterMotor.set(-counterCommand);
//        } else {
//            counterMotor.stopMotor();
//        }
//    }
//
//    /**
//     * @return sum of both encoders, wrapped from 0 to 360
//     */
//    double getRawAngle() {
//        return /*MathUtils.resolveDeg(*/(lastValidDistanceClockwise + lastValidDistanceCounter)*36.0/*)*/;
//    }
//    double getAngle() {
//        return MathUtils.resolveDeg((lastValidDistanceClockwise + lastValidDistanceCounter)*36.0);
//    }
//
//    public double getClockwiseEncoder() {
//        return clockwiseEncoder.getPosition();
//    }
//    public double getCounterEncoder(){
//        return counterEncoder.getPosition();
//    }
//    /**
//     * @return Distance the module has translated
//     */
//    double getDistance() {
//        return motor.getPosition();
//    }
//
//    void reset() {
//        motor.setPosition(0);
//    }
//
//    void stop() {
//        motor.stopMotor();
//    }
//
//    void setID(int id) {
//        this.id = id;
//    }
//}
