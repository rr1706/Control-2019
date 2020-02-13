package frc.robot;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
//import java.nio.channels.CancelledKeyException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.sql.SQLSyntaxErrorException;
import java.util.List;
import java.util.Properties;

import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.DigitalInput;

//import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SwerveDrivetrain.WheelType;
import frc.robot.utilities.*;
//import frc.robot.RRLogger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CameraServer;

//import org.opencv.core.Mat;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update the manifest file in the
 * resource directory.
 */
public class Robot extends TimedRobot {

    private double ACCEL_SPEED= 0.1;
    private double DECEL_SPEED= 0.28; //0.3

    private SendableChooser<Integer> startingSide = new SendableChooser<>();
    private SendableChooser<Integer> autoChooser = new SendableChooser<>();

    private Limelight hatchCam = new Limelight("hatch");

    private Limelight cargoCam = new Limelight("cargo");

    private Compressor compressor;

    public static XboxController xbox1 = new XboxController(0);
    public static XboxController xbox2 = new XboxController(1);

    //	private JetsonServer jet;
//	private Thread t;
    private SwerveDrivetrain driveTrain;
    private IMU imu;

//	private boolean robotBackwards;

    private double robotOffset;
    private boolean prevXButton = false;
    private double camX = 0.0;
    private double camZ = 0.0;
    private int latencyCounter = 0;
    private boolean autoIntake = false;
    private boolean grabAlign = false;
    private boolean holdCargo = false;
    private double cargoTimer = 0.0;
    private boolean seenSensor = false;
    private double limelightSegmentDistance = 0.0;
    private double lastTargetSideDistance = 0.0;
    private double lastTargetFrontDistance = 0.0;
    private double lastTargetAngle = 0.0;
    private double hatchFrontOffset = 17.0;//22.0 degrees from center, 16 inches from front
    private double hatchSideOffset = 8.8; //8.8
    private double cargoCamOffset = 0.0;

    private double camHeadingAngle = 0.0;
    private boolean habDistanceGood = false;
    private double cargoAlignTime = 0.0;
    private double prevRumbleTimer = 0.0;
    private double rotationMax = 1.0;
    private double[] rotationValues;
    private double accelTimeChanger = 0.0;
    private int disabled = 0;
    private double prevTrigTime = 0.0;
    private double wallAlignCloseTimer = 0.0;
    private boolean trigTimeIncrease = true;
    private int wheelStopCase = 0;
    private double wallAlignDecelDistance = 90.0;
    private double habTime = 0.0;
    private double sideAlignP = 1.53e-2;
    private double frontAlignP = 1.53e-2;
    private boolean camAlignDone = false;
    private double sideAlignI = 0.0;
    private double frontAlignI = 0.0;
    private double wallAlignError = 1.1;

    private boolean habStart = false;
    private double[][] commands;
    private boolean doClimb = false;
    private boolean prevStartButton = false;
    private boolean prevBackButton = false;
    private boolean compensateToggle = true;
    private int sensorToLookAt = 0; //0 = both, 1 = right side, 2 = left side
    private int arrayIndex = -1;
    private int autoMove = 0;
//    private int translateType;
//    private double autonomousAngle;
    private double tSpeed;
    private double rSpeed;
    private double previousDistance = 0.0;
    private double currentDistance = 0.0;
    private double headingRad = 0.0;
    private double headingVector = 0.0;

    private double habLidarTimeout = 0.0;
    private double rotationAlignP = 0.0065;
    private double autoRCWP = 0.05;
    private double teleopRCWP = 0.007;

    private boolean override = false;
    private boolean driveDone = false;
    private boolean turnDone = false;
    private boolean elevatorDone = false;
    private boolean hatchDone = false;
    private boolean cargoDone = false;
    private boolean timeDone = false;
    private double prevCameraDistance = 0.0;
//    private double offsetDeg;
//    private double prevOffset = 0;
    private double timeBase;
    private boolean timeCheck;
//    private double smoothArc;
    private double smoothAccelerate;
    private double smoothAccelerateNum;
    private double initialAngle;
    private final double minSpeed = 0.2;

    private int dx = -1;

    private double FWD;
    private double STR;
    private double RCW;

    private double[] prevFWD = {0.0, 0.0, 0.0, 0.0, 0.0};
    private double[] prevSTR = {0.0, 0.0, 0.0, 0.0, 0.0};
    private double[] prevRCW = {0.0, 0.0, 0.0, 0.0, 0.0};

//    private double wheelRamp = 0;
//    private double rampRate = 0;
//    private double currentRampTime = 0;
//    private double prevRampTime = 0;

    private double keepAngle;

    private boolean autonomous = true;

    private boolean fieldOriented = true; // start on field orientation
    private boolean reverseRobotOriented = false;
    private boolean previousOrientedButton = false;
    private boolean previousReverseRobotOrientedButton = false;

    private boolean currentOrientedButton = false;
    private boolean currentReverseRobotOrientedButton = false;

    private boolean buttonTimeIncrease = false;
    private boolean wheelsGood = false;

    private PIDController SwerveCompensate;
    private PIDController wallAlignSTR = new PIDController(sideAlignP, 0.0, 0.0);
    private PIDController wallAlignFWD = new PIDController(frontAlignP, 0.0, 0.0);
    private PIDController wallAlignRCW = new PIDController(rotationAlignP, 0.0, 0.0);
    private PIDController teleopRCW = new PIDController(teleopRCWP, 0.0, 0.0);
    private PIDController autoRCW = new PIDController(autoRCWP, 0.0, 0.0);


    private double lead;

    private double useCushionDistance = 0.0;
    private double cushionMaxSpeed = 0.0;
    private double robotRotation;

    private double imuOffset = 0;

    private Properties application = new Properties();
    private File offsets = new File("/home/lvuser/deploy/SWERVE_OFFSET.txt");

//    private boolean rumble = false;
//    private int rumbleTime = 0;

    private Acceleration accel;
    private Acceleration rcwAccel;

    private Acceleration decelFWD;
    private Acceleration decelSTR;
    private Acceleration decelRCW;
    private int cmdCounter = 0;
//    private boolean decel = false;
//    private int accelState = -1;
//    private int gameState = 0;
    private int autonomousChoice;
    private int startingChoice;


    private boolean useKeepAngle = false;
    private int cameraCounter = 0;
    private double angleDelta = 0.0;
    private double[] ArcXs = new double[3];
    private double[] ArcYs = new double[3];
    double[] curveVelocity = {0.0, 0.0};
    private  double position = 0.0;
    private int currentIndex = 0;
    private int prevIndex = -1;
    private double totalArcLength = 0.0;
    private double deltaX = 0.0;
    private double deltaY = 0.0;
    private int limelightCase = 0;

    private boolean arcCalculated = false;
    private int setpoint = 0;
    private double prevButtonTime = 0.0;

    private double placeholderName = 0.45;
//    private boolean robotAligned = false;
//    private boolean elevatorDescending = false;
    private boolean toggleCompresor = false;
    private boolean prevRB = false;
    private int elevatorCase = 0;
    private boolean safeToPutHatch = false;
    private boolean wallAlignClose = false;
    private double tempForward = 0.0;

    private boolean autoOverride = false;
    private double wallAlignI = 0.0;
    private boolean wallAlignDone = false;
    private boolean cargoShipDone = false;

    private double prevWallAlignTime = 0.0;


    //	private int arcResolution = 100;
//	private int[] arcLengths = new int[arcResolution+1];
    private double[][] arcPoints = new double[3][101];

    private int alignCase = 0;
    private int cargoCase = 0;

//    private void frictionCompensate() {
//        double averageError = 0.25*(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getAngleError() + SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getAngleError() + SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getAngleError() + SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getAngleError());
//        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).checkFrictionCompensation(averageError);
//        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).checkFrictionCompensation(averageError);
//        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).checkFrictionCompensation(averageError);
//        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).checkFrictionCompensation(averageError);
//    }

    private boolean atGoodDistance (double currentDistance, double targetDistance, double allowedError) {
        return !(currentDistance <= targetDistance - allowedError || currentDistance>= targetDistance + allowedError);
    }

//    private void wallAlignClose() {
//        double frontDistance;
//        boolean strafeDone = false;
//        double front = (Lidar.getFRFrontSensor() < Lidar.getFLFrontSensor()) ? Lidar.getFRFrontSensor(): Lidar.getFLFrontSensor();
        //Done when robot is oriented
//        if (LineSensor.get() == 3) {
//            STR = 0.0;
////            System.out.println("Yay");
//            if (Math.abs(FWD) < 0.03) {
//                wallAlignDone = true;
//            }
//
//        } else {
//            wallAlignCloseTimer = Time.get();
//            if (LineSensor.get() == 1) { //If only the sensor on the right sees the line, move left
//                STR = -0.05; //Lower if oscillation happens
//                System.out.println("Left");
//            } else if (LineSensor.get() == 2) {
//                STR = 0.05;
//                System.out.println("Right");
//            }
//        }
//
//        if (STR == 0.0 && Time.get()-cargoAlignTime > 0.2) {
//            strafeDone = true;
//        }
//
//        //Only move far foward once a line is seen
////        if(strafeDone) {
////            frontDistance = 5.5;
////            if (/*rotationDone*/atGoodDistance(front, frontDistance, wallAlignError)) { //Tweak with real field values
////                FWD = Math.signum(Math.cos(imu.getAngle())) * (front-frontDistance) * 0.165; //Decreases speed with distance from cargoShip
////            } else {
////                FWD = 0.0;
////            }
////        }
////        System.out.println(FWD);
//    }
    private void acceleration() {
                    if (Math.abs(FWD) > Math.abs(prevFWD[cmdCounter])) {
                        if (Math.abs(FWD - prevFWD[cmdCounter]) > ACCEL_SPEED) {
                            if (FWD - prevFWD[cmdCounter] > 0.0) {
                                FWD = prevFWD[cmdCounter] + ACCEL_SPEED;
                            } else {
                                FWD = prevFWD[cmdCounter] - ACCEL_SPEED;
                            }
                        }
                    } else {
                        if (Math.abs(FWD - prevFWD[cmdCounter]) > DECEL_SPEED) {
                            if (FWD - prevFWD[cmdCounter] > 0.0) {
                                FWD = prevFWD[cmdCounter] + DECEL_SPEED;
                            } else {
                                FWD = prevFWD[cmdCounter] - DECEL_SPEED;
                            }
                        }
                    }

                    if (Math.abs(STR) > Math.abs(prevSTR[cmdCounter])) {
                        if (Math.abs(STR - prevSTR[cmdCounter]) > ACCEL_SPEED) {
                            if (STR - prevSTR[cmdCounter] > 0.0) {
                                STR = prevSTR[cmdCounter] + ACCEL_SPEED;
                            } else {
                                STR = prevSTR[cmdCounter] - ACCEL_SPEED;
                            }
                        }
                    } else {
                        if (Math.abs(STR - prevSTR[cmdCounter]) > DECEL_SPEED) {
                            if (STR - prevSTR[cmdCounter] > 0.0) {
                                STR = prevSTR[cmdCounter] + DECEL_SPEED;
                            } else {
                                STR = prevSTR[cmdCounter] - DECEL_SPEED;
                            }
                        }
                    }

        prevFWD[cmdCounter] = FWD;
        prevSTR[cmdCounter] = STR;
        prevRCW[cmdCounter] = RCW;
    }

    private void cargoshipAlign(double angle, int sideToFace, double autoSTR, double autoFWD) {
        keepAngle = angle;
//        System.out.println(trigTimeIncrease + "||" + (Time.get() - prevTrigTime));
        double distanceFromCargoship = (Lidar.getFRFrontSensor() < Lidar.getFLFrontSensor()) ? Lidar.getFRFrontSensor() : Lidar.getFLFrontSensor();

        if (sideToFace == -1) {
            distanceFromCargoship = (Lidar.getBLLeftSensor() < Lidar.getBRRightSensor()) ? Lidar.getBLLeftSensor() : Lidar.getBRRightSensor();
        }

        boolean rotationDone = false;
        boolean forwardDone = false;
        boolean strafeDone = false;
        double frontDistance = 11.0;

        if (Math.abs(MathUtils.calculateContinuousError(angle, imu.getAngle(), 360.0, 0.0)) >= 3.0) {
            RCW = MathUtils.calculateContinuousError(angle, imu.getAngle(), 360.0, 0.0) * 1.0e-2;
            if (Math.abs(RCW) > 0.3) {
                RCW = 0.3 * Math.signum(RCW);
            } else if (Math.abs(RCW) < 0.03) {
                RCW = 0.03 * Math.signum(RCW);
            }
        } else {
            RCW = 0.0;
            rotationDone = true;
        }

        if (!seenSensor && LineSensor.get() == 0){
            STR = autoSTR;
            FWD = autoFWD;
//            System.out.println(autoMove);
        }

        if (LineSensor.get() == 3) {
            STR = 0.0;
            seenSensor = true;
//            System.out.println("3");
        } else {
            cargoAlignTime = Time.get();
            if (/*rotationDone && */LineSensor.get() == 1) { //If only the sensor on the right sees the line, move left
                STR = -0.04; //Lower if oscillation happens
//                System.out.println("1");
                seenSensor = true;
            } else if (/*rotationDone && */LineSensor.get() == 2) {
                STR = 0.04;//0.12
//                System.out.println("2");
                seenSensor = true;
            }
        }

        if (STR == 0.0 && Time.get()-cargoAlignTime > 0.2) {
            strafeDone = true;
//            System.out.println("Here");
        }

        //Only move far foward once a line is seen
        if(strafeDone) {
            frontDistance = 6.0;
            if (rotationDone && (frontDistance-2.0 > distanceFromCargoship || distanceFromCargoship > frontDistance+2.0)) { //Tweak with real field values
                FWD = Math.signum(sideToFace) * (distanceFromCargoship-frontDistance) * 0.165; //Decreases speed with distance from cargoShip
            } else {
                forwardDone = true;
                FWD = 0.0;
            }
        }

        if (Math.abs(FWD) > 0.165) {
            FWD *= 0.165;
        }

        cargoShipDone = (rotationDone && forwardDone);

//        System.out.println(rotationDone + "|" + forwardDone + "|" + strafeDone + "|" + cargoShipDone);
    }

    private boolean cameraAlign(double angle, double front, double side) {
        cargoCam.enable();
        hatchCam.enable();

        useKeepAngle = false;

        teleopRCW.setSetpoint(angle);

        teleopRCW.setInput(imu.getAngle());

        RCW = teleopRCW.performPID();


        if (0.0 != SmartDashboard.getNumber("Hatch X", 0.0)){
            camX = SmartDashboard.getNumber("Hatch X", 0.0);
        }
        if (0.0 != SmartDashboard.getNumber("Hatch Z", 0.0)) {
            camZ = SmartDashboard.getNumber("Hatch Z", 0.0);
        }

        wallAlignSTR.setSetpoint(side); //-0.8 maybe
        wallAlignFWD.setSetpoint(front);
        wallAlignSTR.setInput(camX);
        wallAlignFWD.setInput(camZ);

        wallAlignFWD.setInputRange(-100, 0.0);
        wallAlignSTR.setInputRange(-100, 100.0);

        wallAlignFWD.setOutputRange(-0.2, 0.2);
        wallAlignSTR.setOutputRange(-0.15, 0.15);

        wallAlignFWD.setMaximumI(0.0);
        wallAlignSTR.setMaximumI(0.0);

        double frontAlign = 1.35e-2;
        double sideAlign = 1.5e-2;
        double frontAlignI = 2.4e-4;
        double sideAlignI = 2.4e-4;


        wallAlignFWD.setPID(frontAlign, 0.0, 0.0);
        wallAlignSTR.setPID(sideAlign, 0.0, 0.0);

        FWD = wallAlignFWD.performPID();
        STR = wallAlignSTR.performPID();

        return (Math.abs(STR) + Math.abs(FWD) < 0.005);
    }

    //TODO, temp test method

    private void camHeadingAngle(double angle)
    {
        camHeadingAngle = (limelightSegment("Hatch")[1] + imu.getAngle());
    }

    private boolean cameraSegment(double angle, double threshold) {
            cargoCam.enable();
            hatchCam.enable();

//            double tempAngle = 0.0;
//            if (angle < 270.0 && angle > 90.0) {
//                tempAngle = 180.0;
//            }

        //Temp for rotate only:
        teleopRCW.setSetpoint(angle);

        teleopRCW.setInput(imu.getAngle());

        RCW = teleopRCW.performPID();


        if (cameraCounter == 0) {
                limelightSegmentDistance = limelightSegment("Hatch")[0];
//                teleopRCW.setSetpoint(tempAngle);
                useKeepAngle = false;
                camX = SmartDashboard.getNumber("Hatch X", 0.0);
                camZ = SmartDashboard.getNumber("Hatch Z", 0.0);

            } else if (cameraCounter == 1) {
//                teleopRCW.setSetpoint(tempAngle);

                if (limelightSegment("Hatch")[1] != 0.0) {
                    //For now, adding the tX every pass only hurts when the tX is 0.0
//                    teleopRCW.setSetpoint(angle);
                    angleDelta = MathUtils.resolveDeg(360-(-limelightSegment("Hatch")[1] + angle-imu.getAngle()));

                    headingVector = Math.toRadians(angleDelta);

                    cameraCounter++;
                }
            }

        //If it sees it and returns 8.8 or 18.0, that's valid
            if (camX != 0.0 && camZ != 0.0 && limelightSegmentDistance > 0.0 /*&& limelightSegmentDistance != hatchFrontOffset*/ && cameraCounter == 0) {
                cameraCounter++;
            }

            if (cameraCounter == 2) {

                double headingDeg = angleDelta;
//
//                if (limelightSegment("Hatch")[1] != 0.0) {
//                    headingDeg = MathUtils.resolveDeg(-limelightSegment("Hatch")[1] + angleDelta);
//                }
//
                headingVector = Math.toRadians(headingDeg);

//                teleopRCW.setInput(imu.getAngle());
//
//                RCW = teleopRCW.performPID();

                double robotDistanceGone = SmartDashboard.getNumber("Robot Distance", currentDistance) - prevCameraDistance;

                if (((limelightSegment("Hatch")[1] == 0.0) && (limelightSegment("Hatch")[0] <= 0.0) &&
                        robotDistanceGone > 0.9*(Math.abs(camX-hatchSideOffset) + Math.abs(camZ+hatchFrontOffset))) ||
                        robotDistanceGone < 0.9*(Math.abs(camX-hatchSideOffset) + Math.abs(camZ+hatchFrontOffset))) { //Todo, only goes 90%, do 3 segments automatically

                    if (xbox1.LTrig() > 0.1 && robotDistanceGone < Math.abs(camX-hatchSideOffset)) {
                        if (angleDelta > 180.0) {
                            STR = -1.0;
                        } else {
                            STR = 1.0;
                        }

                        STR *= 0.1;

                    } else if (threshold < Math.abs(SmartDashboard.getNumber("Hatch Z", 0.0)) && xbox1.LTrig() > 0.1){ //if (xbox1.LTrig() > 0.1 && robotDistanceGone < Math.abs(limelightSegmentDistance*Math.cos(headingVector))) {
                        FWD = 0.2;
                    }


//                    if (xbox1.LTrig() > 0.1 && Math.abs(RCW) < 0.08) {
//                        FWD = Math.cos(headingVector) / 4.0;

//                        if (angleDelta > 180.0) {
//                            STR = -1.0;
//                        } else {
//                            STR = 1.0;
//                        }

//                        STR *= Math.sin(headingVector) / 4.0;
//                    }

                } else if ((limelightSegment("Hatch")[1] != 0.0) && (limelightSegment("Hatch")[0] >= 0.0)) {
                    cameraCounter = 0;
                    prevCameraDistance = SmartDashboard.getNumber("Robot Distance", currentDistance);
                }


//                System.out.println(camX + "  |      |   " + camZ + "  |      |   " + robotDistanceGone);

            }
            if (grabAlign) {
                STR = 0.0;
                FWD = 0.0;
            }
            camAlignDone = (Math.abs(STR) + Math.abs(FWD) < 0.005);
            return (Math.abs(STR) + Math.abs(FWD) < 0.005);
    }

    private boolean wallAlign(double angle,  double moveAngle, double sideDistance, double frontDistance) {
        lastTargetFrontDistance = frontDistance;
        lastTargetSideDistance = sideDistance;
        lastTargetAngle = angle;
        double tempAngle = 0.0;
        wallAlignRCW.setInput(imu.getAngle());

        double minSpeed =0.045;
        double robotAngle = imu.getAngle();
        int done = 0;
        double[] distances = {Lidar.getFRRightSensor(), Lidar.getFRFrontSensor(), Lidar.getFLFrontSensor(), Lidar.getFLLeftSensor()};

        double side;
        if (sensorToLookAt == 2) {
            side = Lidar.getFLLeftSensor();

        } else if (sensorToLookAt == 1) {
            side = Lidar.getFRRightSensor();

        } else {
            side = (Lidar.getFRRightSensor() < Lidar.getFLLeftSensor()) ? Lidar.getFRRightSensor() : Lidar.getFLLeftSensor();
        }

        double front = (Lidar.getFRFrontSensor() < Lidar.getFLFrontSensor()) ? Lidar.getFRFrontSensor() : Lidar.getFLFrontSensor();

        double farSide = MathUtils.instertionSort(distances)[2];

//        System.out.println(alignCase);

        if (atGoodDistance(front, frontDistance, 4.1)) {
            sideAlignI = 2.4e-4;
            frontAlignI = 2.4e-4;
            wallAlignFWD.setMaximumI(0.0);
            wallAlignSTR.setMaximumI(0.0);

//            if (LineSensor.get() != 0) {
//                wallAlignClose = true; //The robot can align faster using the line sensors
//            } else {
//                wallAlignClose = false;
//            }

        } else {
            wallAlignClose = false;

            wallAlignFWD.resetI();
            wallAlignSTR.resetI();

            sideAlignI = 0.0;
            frontAlignI = 0.0;
        }

        wallAlignFWD.setPID(frontAlignP, frontAlignI, 0.0);
        wallAlignSTR.setPID(sideAlignP, sideAlignI, 0.0);


        switch (alignCase) {
            case 0:
                if (!xbox1.LB() && Time.get() - prevWallAlignTime < 1.0 && (front < 13.0 || side < 13.0)) { //Back away from walls
                    if (front < 14.0) {
                        FWD = -0.3/Math.cos(Math.toRadians(robotAngle));
                    }
                    if (side < 14.0) {
                        STR = -0.1*Math.signum(Lidar.getFLLeftSensor() - Lidar.getFRRightSensor()) / Math.cos(Math.toRadians(robotAngle)); //Should be robot oriented and based off side that is closer
                    }
                    RCW = 0.0;
                } else {
                    alignCase = 1;
                }
                break;
            case 1: //Rotate fast to about right angle
//                rotationAlignP = 0.01;
                wallAlignRCW.setOutputRange(-1.0, 1.0);
                useKeepAngle = false;
                FWD = 0.0;
                STR = 0.0;


//                System.out.println(wallAlignRCW.getError() + "|" + wallAlignRCW.performPID());
//                System.out.println(angle + " | | " + robotAngle);
                //Temp angle code to make robot move at 90 degree orientations
//                if (angle < 90.0 || angle > 270.0) {
//                    tempAngle = 0.0;
//                    System.out.println("Here");
//                } else {
//                    tempAngle = 180.0;
//                }

                wallAlignRCW.setSetpoint(angle);

                if (Math.abs(MathUtils.calculateContinuousError(angle, robotAngle, 360.0, 0.0)) >= 5.0) {
                    RCW = wallAlignRCW.performPID(); //0.016
//                    System.out.println(RCW);
//                    if (Math.abs(RCW) > 1.0) { //0.3
//                        RCW = 1.0 * Math.signum(RCW); //0.3
//                    } else if (Math.abs(RCW) < 0.2) {
//                        RCW = 0.2 * Math.signum(RCW);
//                    }

                } else {
                    alignCase = 2;
                }

                break;
            case 2: //Slowly rotate to better angle and translate to position
//                sideDistance = 2+(sideDistance * Math.abs(Math.sin(Math.toRadians(90.0-angle))));

                if (Math.abs(MathUtils.calculateContinuousError(angle, imu.getAngle(), 360.0, 0.0)) > 3.5 || front > 25.0) {
                    sideDistance = 5+(sideDistance * Math.abs(Math.sin(Math.toRadians(90.0-angle))));
                    //Fixme, might cause extra moving out
                }

                wallAlignSTR.setSetpoint(sideDistance);
                wallAlignFWD.setSetpoint(frontDistance);
                wallAlignSTR.setInput(side);
                wallAlignFWD.setInput(front);
                wallAlignFWD.setOutputRange(-0.5, 0.5);
                wallAlignSTR.setOutputRange(-0.3, 0.3);
                wallAlignRCW.setOutputRange(-0.3, 0.3);
                wallAlignRCW.setSetpoint(angle);

                if (Math.abs(MathUtils.calculateContinuousError(angle, imu.getAngle(), 360.0, 0.0)) >= 2.8) {
                    RCW = wallAlignRCW.performPID();
                    if (Math.abs(RCW) > rotationMax) {
                        RCW = rotationMax * Math.signum(RCW);
                    } else if (Math.abs(RCW) < 0.05) {
                        RCW = 0.05 * Math.signum(RCW);
                    }
                    useKeepAngle = false;
                } else {
                    keepAngle = angle;
                    useKeepAngle = true;
                    RCW = 0.0;
                    done++;
                }

//                if (!wallAlignClose) {
                    if (!atGoodDistance(side, sideDistance, wallAlignError)) { //Decrease allowed error as much as possible. Log lidar values in comp. to get experimental mean and standard deviation
                        STR = Math.sin(Math.toRadians(moveAngle)) * -wallAlignSTR.performPID();

                        if (Math.abs(STR) < minSpeed) {
                            STR = Math.signum(STR) * minSpeed;
                        }

                    } else {
                        STR = 0.0;
                        done++;
                    }
//                }
//                System.out.println(side  +  "     ||      " + sideDistance +  "     ||      " +  wallAlignSTR.performPID() +  "     ||      " +   !atGoodDistance(side, sideDistance, wallAlignError));

//                tempForward = FWD;

                if (!atGoodDistance(front, frontDistance, wallAlignError)) {
                    FWD = Math.cos(Math.toRadians(moveAngle)) * -wallAlignFWD.performPID(); //0.025

                    if (Math.abs(FWD) < minSpeed) {
                        FWD = Math.signum(FWD) * minSpeed;
                    }

                } else {
                    FWD = 0.0;
                    done++;
                }
                break;
        }

//        System.out.println(FWD + "     " + STR + "     " + RCW);

        wallAlignDone = (done == 3);
        return wallAlignDone;
    }

    private boolean HABAlign(){

        boolean rotateDone = false;
        boolean fwdDone = false;
        double robotAngle = imu.getAngle();
        double front = (Lidar.getFRFrontSensor() < Lidar.getFLFrontSensor() ? Lidar.getFRFrontSensor(): Lidar.getFLFrontSensor());

//        if(Math.abs(MathUtils.calculateContinuousError(180.0, robotAngle, 360.0, 0.0)) >= 7.0){
//            RCW = MathUtils.calculateContinuousError(180.0, robotAngle, 360.0, 0.0) * 0.011;
//
//            if(Math.abs(RCW) > 0.1){
//                RCW = Math.signum(RCW) * 0.1;
//            }
//
//            keepAngle = 180.0;
//            habLidarTimeout = Time.get();
//
//        } else{
//            RCW = 0.0;

        //Fixme add back in later
//            if (Time.get() - habLidarTimeout > 1.75) {
//                fwdDone = true;
//            }
//Use this for testing:
//        habDistanceGood = true;
//        fwdDone = true;
            if (front < 14.5){ //When a sensor is brokem, it will return 9999.0, so it will climb automatically if both are bad
                FWD = -0.04;
                System.out.println("Backing up");
            } else {
                habDistanceGood = true;
                System.out.println("Distance good");
                fwdDone = true;
        }

//        System.out.println(fwdDone + " " + rotateDone);

        return  (fwdDone);
    }

    private boolean sideRocketAlign (double angle) {
        int done = 0;
        double robotAngle = imu.getAngle();
        double frontRight = Lidar.getBRRightSensor();
        double frontLeft = Lidar.getBLLeftSensor();

        switch (cargoCase) {
            case 0:
                if (Math.abs(MathUtils.calculateContinuousError(angle, robotAngle, 360.0, 0.0)) >= 2.8) {
                    RCW = MathUtils.calculateContinuousError(angle, robotAngle, 360.0, 0.0) *0.02;
                    if (RCW > 0.2) {
                        RCW = 0.2;
                    }
                    keepAngle = angle;
                } else {
                    RCW = 0.0;
                    cargoCase = 1;
                }
                break;
            case 1:
                if (frontLeft < 11.0 || frontRight < 11.0) {
                    cargoCase = 2;
                } else {
                    FWD = -0.25; //Scale to distance from
                }
                break;
            case 2:
                if (Math.abs(MathUtils.calculateContinuousError(angle, robotAngle, 360.0, 0.0)) >= 2.8) {
                    RCW = MathUtils.calculateContinuousError(angle, robotAngle, 360.0, 0.0) *0.011;
                    if (RCW > 0.16) {
                        RCW = 0.16;
                    }
                    keepAngle = angle;
                } else {
                    RCW = 0.0;
                    done++;
                }

                if (LineSensor.get() == 0.0) { //FIXME, tweak with real world
                    if (frontLeft < 7.5 && frontRight < 7.5) {
                        done++;
                    } else if (frontLeft >= 7.0) { //If only BL edge is off cargo, move robot right
                        STR = 0.08 + (frontLeft - frontRight) * 0.0035;
                    } else if (frontRight >= 7.0) {
                        STR = -0.08 - (frontRight - frontLeft) * 0.0035;
                    }
                } else {
                    if (LineSensor.get() == 3) {
                        STR = 0.0;
//                        System.out.println("Yay 2");
                        wallAlignDone = true;
                    } else {
                        cargoAlignTime = Time.get();
                        if (LineSensor.get() == 1) { //If only the sensor on the right sees the line, move left
                            STR = -0.04; //Lower if oscillation happens
//                            System.out.println("Left 2");
                        } else if (LineSensor.get() == 2) {
                            STR = 0.04;
//                            System.out.println("Right 2");
                        }
                    }
                }

                if (frontLeft > 11.0 && frontRight > 11.0) {
                    FWD = -0.2;
                }
        }

        SmartDashboard.putBoolean("Cargo Aligned", (done == 2));
        return (done == 2);
    }

    private void keepAngle() {
        // LABEL keepAngle
        SwerveCompensate.enable();

        double leadNum = SmartDashboard.getNumber("leadNum", 0);
        lead = RCW * leadNum;


        // This will update the angle to keep the robot's orientation
        if (useKeepAngle && //If wallAlign wants to use keepAngle
                Math.abs(xbox1.RStickX()) > 0.05 || // If right stick is pressed
                (Math.abs(FWD) < 0.05 && Math.abs(STR) < 0.05) && // If left stick is not pressed
                        (xbox1.buttonPad() == -1) && // If dpad is not pressed
                        (!autonomous)) { // If teleop

            SwerveCompensate.setPID(0.015/*0.015*/, 0.0, 0.0);
            keepAngle = imu.getAngle();
        } else if (useKeepAngle){

//            System.out.println("Here");
            SwerveCompensate.setPID(0.008/*0.008*/, 0.0, 0.0);

            SwerveCompensate.setInput(imu.getAngle());
            SwerveCompensate.setSetpoint(keepAngle);

            if (!SwerveCompensate.onTarget()) {
//                System.out.println("Correcting");
                SwerveCompensate.setPID(0.005/*0.007*/, SmartDashboard.getNumber("CompensateI", 0.0), SmartDashboard.getNumber("CompensateD", 0.0));
            }

            robotRotation = SwerveCompensate.performPID();

            RCW = robotRotation;
        }
    }

    /**
     * Each potentiometer is positioned slightly differently so its initial value is different than the others,
     * so even when the wheels are pointing straight there are differences.
     * Proper values may be found and must be calculated for each wheel.
     */
    private void loadOffsets() {
        // LABEL load offsets

        // Set the position of each wheel from a file on the roborio
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setPosition(Vector.load(application.getProperty("front_right_pos", "0.0,0.0")));
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setPosition(Vector.load(application.getProperty("front_left_pos", "0.0,0.0")));
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setPosition(Vector.load(application.getProperty("back_left_pos", "0.0,0.0")));
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setPosition(Vector.load(application.getProperty("back_right_pos", "0.0,0.0")));

        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setOffset(0.0);
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setOffset(0.0);
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setOffset(0.0);
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setOffset(0.0);
    }

//    private void autonomousAngle(double angle) {
//        // LABEL autonomous angle
//
//        SwerveCompensate.setInput(imu.getAngle());
//        SwerveCompensate.setSetpoint(angle);
//
//        SwerveCompensate.setTolerance(7);
//        if (!SwerveCompensate.onTarget()) {
//            SwerveCompensate.setPID(0.013, SmartDashboard.getNumber("CompensateI", 0.0), SmartDashboard.getNumber("CompensateD", 0.0));
//        } else {
//            SwerveCompensate.setPID(0.015, SmartDashboard.getNumber("CompensateI", 0.0), SmartDashboard.getNumber("CompensateD", 0.0));
//        }
//
//        robotRotation = SwerveCompensate.performPID();
//
//        RCW = (robotRotation);
//    }

    private double calculateLength(double[] ArcXs, double[] ArcYs) {
        double vx = 2 * (ArcXs[1] - ArcXs[0]);
        double vy = 2 * (ArcYs[1] - ArcYs[0]);
        double wx = ArcXs[2] - 2 * ArcXs[1] + ArcXs[0];
        double wy = ArcYs[2] - 2 * ArcYs[1] + ArcYs[0];

        double uu = 4 * (Math.pow(wx, 2) + Math.pow(wy, 2));
        if (uu < 0.00001) {
//            System.out.println(Math.sqrt(Math.pow(ArcXs[2] - ArcXs[0], 2) + Math.pow(ArcYs[2] - ArcYs[0], 2)));
        }

        double vv = 4 * (vx * wx + vy * wy);
        double ww = Math.pow(vx, 2) + Math.pow(vy, 2);

        double t1 = (float) (2 * Math.sqrt(uu * (uu + vv + ww)));
        double t2 = 2 * uu + vv;
        double t3 = vv * vv - 4 * uu * ww;
        double t4 = (float) (2 * Math.sqrt(uu * ww));

        return (((t1 * t2 - t3 * Math.log(t2 + t1) - (vv * t4 - t3 * Math.log(vv + t4))) / (8 * Math.pow(uu, 1.5))));
    }

    private void makePoints(double curveLength, double[] ArcXs, double[] ArcYs) {
        double[] prevPoint = new double[2];
        double distance = 0.0;
        double aX = ArcXs[0];
        double bX = ArcXs[1];
        double cX = ArcXs[2];
        double aY = ArcYs[0];
        double bY = ArcYs[1];
        double cY = ArcYs[2];
        for (int index = 0; index <= 100; index += 1) { //If curve length is 100 inches, t will be 1/100
            double t  = index / 100.0;

            arcPoints[0][index] = getPoint(t, aX, bX, cX, aY, bY, cY)[0];
            arcPoints[1][index] = getPoint(t, aX, bX, cX, aY, bY, cY)[1];
            if (index > 0) {
                distance += Math.sqrt(Math.pow(arcPoints[0][index] - prevPoint[0], 2) + Math.pow(arcPoints[1][index] - prevPoint[1], 2));
                arcPoints[2][index] = distance/curveLength; //Percent of arcDistance the returned point will be on the arc
            } else {
                arcPoints[2][index] = 0.0; //Percent of arcDistance the returned point will be on the arc
            }
            prevPoint[0] = arcPoints[0][index];
            prevPoint[1] = arcPoints[1][index];
        }
    }

    //This function will return the x and ys of the curve at a given time when given the points
    private double[] getPoint(double t, double aX, double bX, double cX, double aY, double bY, double cY) {
        double[] point = new double[2];
        point[0] = (1-t)*(1-t)*aX + 2*t*(1-t)*bX + t*t*cX;
        point[1] = (1-t)*(1-t)*aY + 2*t*(1-t)*bY + t*t*cY;
        return point;
    }

    //This function will return the scaled FWD and STR commands based off the arc points
    private double[] calculatePath(double distance) {
        double[] velocity = new double[2];

        while (distance  > totalArcLength*arcPoints[2][currentIndex+1] && currentIndex <99) {
            currentIndex++;
        }
        if (currentIndex > prevIndex) {
            deltaX = arcPoints[0][currentIndex+1] - arcPoints[0][currentIndex];
            deltaY = arcPoints[1][currentIndex+1] - arcPoints[1][currentIndex];
        }


        velocity[0] = deltaY/(totalArcLength*(arcPoints[2][currentIndex+1]-arcPoints[2][currentIndex])); //FWD
        velocity[1] = deltaX/(totalArcLength*(arcPoints[2][currentIndex+1]-arcPoints[2][currentIndex])); //STR
        prevIndex = currentIndex;
        return velocity;
    }

    private double[] limelightSegment(String side) {
        double distanceToTarget = MathUtils.pythagorean(SmartDashboard.getNumber(side + " X", 0.0), SmartDashboard.getNumber(side + " Z", 0.0));

        if (SmartDashboard.getNumber(side + " X", 0.0) == 0.0 || SmartDashboard.getNumber(side + " Z", 0.0) == 0.0) {
            distanceToTarget = -10; //Returns negative distance so
        }
//        double angleOffset = hatchFrontOffset;
//        if (side.equals("Hatch")) {
//            angleOffset = cargoCamOffset;
//        }

        double angleToTarget = SmartDashboard.getNumber(side + " Azimuth", 0.0); //Todo, it might be wrong without both limelights on

        double[] segment = {distanceToTarget, angleToTarget};
        return  segment;
    }
    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    public void robotInit() {
        wallAlignSTR.setPID(sideAlignP, sideAlignI, 0.0);
        wallAlignSTR.enable();
        wallAlignSTR.setInputRange(0.0, 30.0);
        wallAlignSTR.setOutputRange(-1.0, 1.0);
        wallAlignSTR.setContinuous(false);
        wallAlignSTR.setMaximumI(0.03);


        wallAlignFWD.setPID(frontAlignP, frontAlignI, 0.0);
        wallAlignFWD.enable();
        wallAlignFWD.setInputRange(0.0, 60.0);
        wallAlignFWD.setOutputRange(-1.0, 1.0);
        wallAlignFWD.setContinuous(false);
        wallAlignFWD.setMaximumI(0.03);

        rotationAlignP = 0.0065;
        wallAlignRCW.setPID(rotationAlignP, 0.0, 0.0);
        wallAlignRCW.enable();
        wallAlignRCW.setInputRange(0.0, 360.0);
        wallAlignRCW.setOutputRange(-rotationMax, rotationMax);
        wallAlignRCW.setContinuous(true);

        autoRCW.setPID(autoRCWP, 0.0, 0.0);
        autoRCW.enable();
        autoRCW.setInputRange(0.0, 360.0);
        autoRCW.setOutputRange(-1.0, 1.0);
        autoRCW.setContinuous(true);

        teleopRCW.setPID(teleopRCWP, 0.0, 0.0);
        teleopRCW.enable();
        teleopRCW.setInputRange(0.0, 360.0);
        teleopRCW.setOutputRange(-1.0, 1.0);
        teleopRCW.setContinuous(true);

//        RRLogger.start();
//        SmartDashboard.putString("RRLogger File", RRLogger.getFileName());
//         LABEL robot init
        // Load the wheel offset file from the roborio

        try {
            FileInputStream in = new FileInputStream(offsets);
            application.load(in);
        } catch (IOException e) {
            e.printStackTrace();
        }

        // Connect to jetson
//		try {
//			jet = new JetsonServer((short) 5800, (short) 5801);
//			t = new Thread(jet);
//			t.start();
//			jet.setDisabled();
//		} catch (IOException e) {
//			throw new RuntimeException(e);
//		}

        compressor = new Compressor(0);

        xbox1.setDeadband(0.5);

        Time.start();

        SwerveDrivetrain.loadPorts();
        driveTrain = new SwerveDrivetrain();
        loadOffsets();

//		SmartDashboard.putNumber("CompensateP", 0.02);
//		SmartDashboard.putNumber("CompensateI", 0.0);
//		SmartDashboard.putNumber("CompensateD", 0.0);

        SmartDashboard.putNumber("Autonomous Delay", 0);

        startingSide.setDefaultOption("Right", 1);
        startingSide.addOption("Left", 2);
        SmartDashboard.putData("Starting Side", startingSide);

        autoChooser.setDefaultOption("Front Hatch", 1);
        autoChooser.addOption("Back Hatch", 2);
        autoChooser.addOption("Cargo Ship", 3);
        SmartDashboard.putData("Auto File Chooser", autoChooser);

        imu = new IMU();
        imu.IMUInit();

        keepAngle = imu.getAngle();

        SwerveCompensate = new PIDController(0.015, 0.00, 0.00);
        SwerveCompensate.setContinuous(true);
        SwerveCompensate.setOutputRange(-1.0, 1.0);
        SwerveCompensate.setInputRange(0.0, 360.0);
        SwerveCompensate.setTolerance(1.0);

        SwerveCompensate.enable();

        accel = new Acceleration();
        decelFWD = new Acceleration();
        decelSTR = new Acceleration();

        decelRCW = new Acceleration();
        rcwAccel = new Acceleration();
    }

    public void autonomousInit() { //Don't move in wallAlign when lidar is invalid, but don't skip step completely
        autonomous = true;
        useKeepAngle = true;
        alignCase = 0;
        autonomousChoice = autoChooser.getSelected();
        startingChoice = startingSide.getSelected();


//        useCushionDistance = true;
        compensateToggle = true;
        // LABEL autonomous init
//		jet.setAuto(); // this line is important because it does clock synchronization

//        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).disableSpeed(2);
//        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).disableSpeed(2);
//        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).disableSpeed(2);
//        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).disableSpeed(2);

        timeCheck = true;
        imu.reset(0);
        setpoint = 1;

        String choice;
        String side;
        String type;

        if (startingChoice == 2) {
            side = "Left";
        } else /*(startingChoice == 1)*/ {
            side = "Right";
        }

        if (autonomousChoice == 3) {
            type = "CargoShip";
        } else if (autonomousChoice == 2) {
            type = "BackHatch";
        } else /*(autonomousChoice == 1)*/ {
            type = "FrontHatch";
        }

//        choice = "/home/lvuser/deploy/" + side + type + ".csv";
        choice = "/home/lvuser/deploy/Test.csv";
        SmartDashboard.putString("Autonomous File", choice);

        imu.reset(0);
        arrayIndex = 0;
        initialAngle = imu.getAngle();
        turnDone = false;
        driveDone = false;
        elevatorDone = false;
        hatchDone = false;
        cargoDone = false;
        autoOverride = false;

        // Fill the array of commands from a csv file on the roborio
        try {
            List<String> lines = Files.readAllLines(Paths.get(choice));
            commands = new double[lines.size()][];
            int l = 0;
            for (String line : lines) {
                String[] parts = line.split(",");
                double[] linel = new double[parts.length];
                int i = 0;
                for (String part : parts) {
                    linel[i++] = Double.parseDouble(part);
//					System.out.println(Double.parseDouble(part));
                }
                commands[l++] = linel;
            }
        } catch (IOException e) {
            e.printStackTrace();
        } catch (NumberFormatException e) {
//            System.err.println("Error in configuration!");
        }

//        RRLogger.addData("Autonomous Init", 0);
//        RRLogger.newLine();
//        RRLogger.writeFromQueue();

        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setID(1);
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setID(2);
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setID(4);
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setID(3);

        //Reset wheel acceleration and movement to zero
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).resetWheel();
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).resetWheel();
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).resetWheel();
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).resetWheel();

        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).resetDelta();
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).resetDelta();
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).resetDelta();
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).resetDelta();
    }

    public void disabledPeriodic() {
        alignCase = 0;
//        SmartDashboard.putNumber("Translation Scale Factor", 0.0);
//        SmartDashboard.putNumber("Rotation Scale Factor", 0.0);

        autonomousChoice = autoChooser.getSelected();
        startingChoice = startingSide.getSelected();


//        useCushionDistance = true;
        compensateToggle = true;
        // LABEL autonomous init
//		jet.setAuto(); // this line is important because it does clock synchronization

//        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).disableSpeed(2);
//        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).disableSpeed(2);
//        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).disableSpeed(2);
//        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).disableSpeed(2);

        arrayIndex = 0;
        initialAngle = imu.getAngle();
        turnDone = false;
        driveDone = false;
        elevatorDone = false;
        hatchDone = false;
        cargoDone = false;
        autoOverride = false;

        timeCheck = true;
//        imu.reset(0);
        setpoint = 1;

        String choice;
        String side;
        String type;

        if (startingChoice == 2) {
            side = "Left";
        } else /*(startingChoice == 1)*/ {
            side = "Right";
        }

        if (autonomousChoice == 3) {
            type = "CargoShip";
        } else if (autonomousChoice == 2) {
            type = "BackHatch";
        } else /*(autonomousChoice == 1)*/ {
            type = "FrontHatch";
        }

//        choice = "/home/lvuser/deploy/" + side + type + ".csv";
        choice = "/home/lvuser/deploy/Test.csv";

        SmartDashboard.putString("Autonomous File", choice);

        // Fill the array of commands from a csv file on the roborio
        try {
            List<String> lines = Files.readAllLines(Paths.get(choice));
            commands = new double[lines.size()][];
            int l = 0;
            for (String line : lines) {
                String[] parts = line.split(",");
                double[] linel = new double[parts.length];
                int i = 0;
                for (String part : parts) {
                    linel[i++] = Double.parseDouble(part);
//					System.out.println(Double.parseDouble(part));
                }
                commands[l++] = linel;
            }
        } catch (IOException e) {
            e.printStackTrace();
        } catch (NumberFormatException e) {
//            System.err.println("Error in configuration!");
        }
    }

    /**
     * This function is called periodically during autonomous
     */

    public void autonomousPeriodic() {
        if (!autoOverride) {
            // LABEL autonomous periodic

            SmartDashboard.putNumber("IMU Angle", imu.getAngle());

            if (timeCheck) {
                timeBase = Time.get();
                timeCheck = false;
            }

//		SmartDashboard.putNumber("Elapsed Time", Time.get());

            switch (autoMove) {

                // Pause the robot for x seconds at the start of auto
                case 0:
                    imu.setOffset(commands[arrayIndex][13]);
                    driveTrain.drive(new Vector(FWD, STR), 0);
                    if (Time.get() > timeBase + SmartDashboard.getNumber("Autonomous Delay", 0)) {
                        autoMove = 1;
//                        RRLogger.addData("Autonomous Periodic Start", 0.0);
                    }
                    previousDistance = currentDistance;

                    break;


                case 1:

                    SmartDashboard.putNumber("Array Index", arrayIndex);

                    /*
                     * 0 = translate speed, 1 = rotate speed, 2 = direction to translate, 3 = direction to face,
                     * 4 = distance(in), 5 = How to accelerate(0 = no modification, 1 = transition, 2 = accelerate, 3 = decelerate)
                     * 6 = ArcXs[0], 7 = ArcXs[1], 8 = ArcXs[2],  9 = ArcYs[0], 10 = ArcYs[1], 11 = ArcYs[2]
                     * 12 = time out(seconds), 13 = imu offset
                     * 14 = elevator  (0 = none, 1 = hatch low, 2 = hatch mid, 3 = hatch high, 4 = cargo low, 5 = cargo mid, 6 = cargo high),
                     * 15 = hatch (0 = none, 1 = put hatch, 2 = get hatch),
                     * 16 = cargo (0 = none, 1 = put cargo , 2 = get cargo)
                     * 17 = use wall align (0 = none, 1 = use, 2 = don't back up)
                     * 18 = side distance for wall align set-point
                     * 19 = front distance for wall align set-point
                     * 20 = cushion distance
                     * 21 = cushion max speed
                     * 22 = use cargoShipAlign (0 = no cargoAlign, 1 = use cargoAlign for hatch, -1 = use cargoAlign for cargo)
                     */

                    cargoShipDone = false;

                    //Only use translation for RCW and Arc Speed
                    tSpeed = commands[arrayIndex][0];
                    rSpeed = commands[arrayIndex][1];

                    ArcXs[0] = commands[arrayIndex][6];
                    ArcXs[1] = commands[arrayIndex][7];
                    ArcXs[2] = commands[arrayIndex][8];

                    ArcYs[0] = commands[arrayIndex][9];
                    ArcYs[1] = commands[arrayIndex][10];
                    ArcYs[2] = commands[arrayIndex][11];

                    switch ((int) commands[arrayIndex][14]) {
                        case 0:
                            Elevator.setPosition(0.0, 0.0, 0.0, false, false, false);
                            elevatorDone = true;
                            break;

                        case 1:
                            override = Elevator.setPosition(1.0, 0.0, 0.0, false, false, false);
                            break;

                        case 2:
                            override = Elevator.setPosition(0.0, 0.0, 1.0, false, false, false);
                            break;

                        case 3:
                            override = Elevator.setPosition(-1.0, 0.0, 0.0, false, false, false);
                            break;
                    }

                    switch ((int) commands[arrayIndex][15]) {
                        case 0:
                            Hatch.set(false, false, false, false);
                            hatchDone = true;
                            break;

                        case 1:
                            override = Hatch.set(false, true, false, true);
                            commands[arrayIndex][15] = 0;
                            break;

                        case 2:
                            override = Hatch.set(true, false, false, true);
                            commands[arrayIndex][15] = 0;
                            break;

                    }

                    if (commands[arrayIndex][2] != 999.0) {
                        FWD = Math.cos(Math.toRadians(commands[arrayIndex][2]));
                        STR = Math.sin(Math.toRadians(commands[arrayIndex][2]));
                    } else {
                        FWD = 0.0;
                        STR = 0.0;
                    }

                    if (ArcXs[0] != 999.0) {
                        commands[arrayIndex][4] = calculateLength(ArcXs, ArcYs);
                        totalArcLength = commands[arrayIndex][4];
                        if (!arcCalculated) {
                            makePoints(totalArcLength, ArcXs, ArcYs);
                            arcCalculated = true;
                        }
                        curveVelocity = calculatePath(SmartDashboard.getNumber("Robot Distance", 0) - previousDistance);
                        STR = -curveVelocity[0];
                        FWD = curveVelocity[1];
                    }

                    keepAngle = commands[arrayIndex][3];

                    autoRCW.setSetpoint(commands[arrayIndex][3]);
                    autoRCW.setInput(imu.getAngle());

                    RCW = autoRCW.performPID();
//                    if (Math.abs(MathUtils.getAngleError(imu.getAngle(), commands[arrayIndex][3])) < 5.0) {
//                        initialAngle = imu.getAngle();
//                    } else {
//                        double direction;
//                        direction = MathUtils.getAngleError(initialAngle, commands[arrayIndex][3]);
//                        if (Math.abs(direction) > 180.0) {
//                            direction *= -1.0;
//                        }
//                        RCW *= Math.signum(direction);
//                        turnDone = false;
//                    }

                        FWD *= tSpeed;
                        STR *= tSpeed;


                    SmartDashboard.putNumber("Previous Distance", previousDistance);

                    if ((Math.abs(SmartDashboard.getNumber("Robot Distance", 0) - previousDistance) >= commands[arrayIndex][4])) {
                        driveDone = true;
                    }

                    //Skipping step entirely? Leave in only the step that leaves the HAB and see what it does
//                    System.out.println(commands[arrayIndex][4] + "  |  |  " + Math.abs(SmartDashboard.getNumber("Robot Distance", 0)) + "  |  |  "  + previousDistance);

                    SmartDashboard.putNumber("Auto Distance Gone", Math.abs(currentDistance - previousDistance));
                    SmartDashboard.putNumber("Auto Distance Command", commands[arrayIndex][4]);

                    SwerveCompensate.setTolerance(1.0);

                    if (Time.get() > timeBase + commands[arrayIndex][12] && commands[arrayIndex][12] > 0.0) {
                        override = true;
//                        RRLogger.addData("Timeskip", 1.0);
//                        RRLogger.newLine();
                    } else if (commands[arrayIndex][12] == 0) {
                        timeDone = true;
                    }

                    imuOffset = commands[arrayIndex][13];

                    if (turnDone) {
                        keepAngle();
                    }

                    if (Math.abs(RCW) < 0.01) {
                        turnDone = true;
                    } else {
                        turnDone = false;
                    }

                    if (commands[arrayIndex][4] == 999.0 && FWD == 0.0 && STR == 0.0) {
                        driveDone = (Math.abs(RCW) < 0.01);
                    }

//				if (robotBackwards) {
//					driveTrain.drive(new Vector(-STR, FWD), RCW);
//				} else {

                    if (Lidar.valid() && commands[arrayIndex][17] != 0.0) {
//                        RRLogger.addData("FRR", Lidar.getFRRightSensor());
//                        RRLogger.addData("FRF", Lidar.getFRFrontSensor());
//                        RRLogger.addData("FLF", Lidar.getFLFrontSensor());
//                        RRLogger.addData("FLL", Lidar.getFLLeftSensor());
//                        RRLogger.newLine();
//                        RRLogger.writeFromQueue();

                        if (commands[arrayIndex][17] == 1.0 && alignCase == 0) {
//                            if (alignCase == 2 && imu.getAngle() - 20.0 > commands[arrayIndex][3] || imu.getAngle() + 20.0 < commands[arrayIndex][3]) {
                                alignCase = 1;
                        }

                        rotationMax = 0.5;

                        useCushionDistance = commands[arrayIndex][20];
                        cushionMaxSpeed = commands[arrayIndex][21];

//                        System.out.println((!xbox1.LB() +"| |"  + (Time.get() - prevWallAlignTime < 1.0)));
                        if (commands[arrayIndex][17] == 3.0) { //Only move sideways
                            if (commands[arrayIndex][2] > 180.0) { //If we want to translate left, only look at the left side sensor
                                sensorToLookAt = 2; //Left side
                            } else {
                                sensorToLookAt = 1; //Right side
                            }
                        } else {
                            sensorToLookAt = 0;
                        }

                        if (wallAlign(commands[arrayIndex][3], commands[arrayIndex][2], commands[arrayIndex][18], commands[arrayIndex][19])) {
                            override = true;
                        }

                        FWD *= commands[arrayIndex][0];
                        STR *= commands[arrayIndex][0];
//                        RCW *= commands[arrayIndex][1];

                        if (commands[arrayIndex][17] == 3.0) { //Only move sideways
                            if (Math.abs(STR) >= 0.05) {
                                FWD = 0.0;
                            } else {
                                override = true;
                            }
                        }
                        //0.7,0.0,87.0,25.0,30.0,0,999,0,0,0,0,0,0.0,0,0,0,0,0,0,0,0.0,0.0,0.0
                    } else if (!Lidar.valid()) {
//                        RRLogger.addData("LidarBroke", 0.0);
                        FWD = 0.0;
                        STR = 0.0;
                        RCW = 0.0;
                    } else {
                        prevWallAlignTime = Time.get();
                        alignCase = 0;
                    }

                    //Command positive forward when using cargoAlign so that the robot will move until it sees a line

                    if (Math.abs(xbox1.LStickX()) > 0.2 || Math.abs(xbox1.LStickY()) > 0.2) {
                        autoOverride = true;
                        FWD = 0.0;
                        STR = 0.0;
                        RCW = 0.0;
                        prevFWD[cmdCounter]= 0.0;
                        prevSTR[cmdCounter] = 0.0;
                        prevRCW[cmdCounter]= 0.0;
                        teleopInit();
                    }

                    acceleration();

                    Vector driveCommands;
                    driveCommands = MathUtils.convertOrientation(Math.toRadians(imu.getAngle()), FWD, STR);
                    FWD = driveCommands.getY();
                    STR = driveCommands.getX();
                    RCW *= rSpeed;

                    System.out.println(RCW + "      " + driveDone);

//                    System.out.println(driveDone + "  " + autoOverride + "  " + FWD + "  " + STR);

//                    System.out.println(arrayIndex + " || " + driveDone + " || " + elevatorDone + " || " + hatchDone + " || " + Elevator.atPosition());

                    if (Math.abs(commands[arrayIndex][22]) == 1.0) {
                        STR = 0.0;
                        FWD = 0.0;
//                        System.out.println(commands[arrayIndex][22]);
                        cargoshipAlign(commands[arrayIndex][3], (int) commands[arrayIndex][22], commands[arrayIndex][0], 0.0);
//                        System.out.println(cargoDone);
                        if (cargoShipDone) {
//                            override = true;
                        }
                    } else {
                        cargoShipDone = true;
                        seenSensor = false;
                    }

                    SmartDashboard.putNumber("FWD", FWD);
                    SmartDashboard.putNumber("STR", STR);
                    SmartDashboard.putNumber("RCW", RCW);

                    driveTrain.drive(new Vector(-STR, FWD), RCW);

                    if (override) {
                        driveDone = true;
                        turnDone = true;
                        elevatorDone = true;
                        hatchDone = true;
                        cargoDone = true;
                        cargoShipDone = true;
                    }

//				System.out.println("Drive: " + driveDone);
//				System.out.println("Turn: " + turnDone);
//				System.out.println("Coll: " + collisionDone);
//				System.out.println("Time: " + timeDone);
//				System.out.println("TimeNum: " + Time.get() + " | " + (timeBase + commands[arrayIndex][10]));

                    SmartDashboard.putNumber("Array", arrayIndex);

//				System.out.println(driveDone + "||" + hatchDone + "||" + cargoDone);

//                    RRLogger.addData("Array Index", arrayIndex);
//                    RRLogger.addData("IMU", imu.getAngle());
//                    RRLogger.addData("FWD", FWD);
//                    RRLogger.addData("STR", STR);
//                    RRLogger.addData("RCW", RCW);

//                    RRLogger.addData("Auto Distance Gone", Math.abs(currentDistance - previousDistance));
//                    RRLogger.addData("Auto Distance Command", commands[arrayIndex][4]);
//                    RRLogger.newLine();
//                    RRLogger.writeFromQueue();

//                    System.out.println(driveDone + "        " + elevatorDone + "        " + hatchDone + "       " + cargoDone + "       " + cargoShipDone);

                    if (driveDone && elevatorDone && hatchDone && cargoDone && Elevator.atPosition() && !Hatch.isRunning() && cargoShipDone) {
//                    robotAligned = false;
                        arcCalculated = false;
                        arrayIndex++;
                        driveDone = false;
                        initialAngle = imu.getAngle();
                        previousDistance = SmartDashboard.getNumber("Robot Distance", 0);//currentDistance;
                        turnDone = false;
                        timeDone = false;
                        override = false;
                        timeBase = Time.get();
                        currentIndex = 0;
                        prevIndex = -1;
                    }
                    break;
            }
        } else {
//            System.out.println("Here!");
            teleopPeriodic();
//            driveTrain.drive(new Vector(0.0, 0.0), 0.0);
        }
    }

    //Day 2 Todo:
    //Check autoIntake for cargo
    //Look at last match to see what to change in front align
    //Figure out why autos don't leave HAB
    public void teleopInit() {
        holdCargo = false;
        sensorToLookAt = 0; //0 = both, 1 = right side, 2 = left side
//        RRLogger.addData("Teleop Init", 0.0);
//        RRLogger.newLine();
//        RRLogger.writeFromQueue();
        rotationMax = 0.5;
//        rotationAlignP = 0.005;
        currentDistance = 0.0;
        useCushionDistance = 30.0;
        cushionMaxSpeed = 1.0;
        compensateToggle = false;
//		jet.startTeleop();

        Lift.init();
        imu.setOffset(imuOffset);
//Sets ids
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setID(1);
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setID(2);
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setID(4);
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setID(3);

        //Reset wheel acceleration and movement to zero
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).resetWheel();
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).resetWheel();
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).resetWheel();
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).resetWheel();

        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).resetDelta();
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).resetDelta();
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).resetDelta();
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).resetDelta();

        Elevator.init();
        Elevator.setPosition(1.0, 0, 0, false, false, false);
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        SmartDashboard.putBoolean("Hab Distance Good", habDistanceGood);
//        System.out.println(FWD + "| |" + STR + "| |" + RCW);
        // LABEL teleop periodic
        autonomous = false;

//        System.out.println(xbox2.RStickY());


        SmartDashboard.putNumber("IMU Angle", imu.getAngle());
        SmartDashboard.putNumber("Elevator Setpoint", position);

        xbox1.setDeadband(0.2);
        xbox2.setDeadband(0.2);

        if (Lidar.getFRFrontSensor() < 20.0 || Lidar.getFLFrontSensor() < 20.0) {
            safeToPutHatch = true;
        } else {
            safeToPutHatch = false;
    }



        SmartDashboard.putBoolean("Compressor", compressor.enabled());
        SmartDashboard.putBoolean("Compressor toggle", toggleCompresor);

        if (xbox2.RB() && !prevRB) {
		    toggleCompresor = !toggleCompresor;
            prevRB = true;
        } else if (!xbox2.RB()){
            prevRB = false;
        }

        if (toggleCompresor) {
            compressor.start();
            xbox2.rumbleRight(0.05);
        } else {
            compressor.stop();
            xbox2.stopRightRumble();
        }

        if (xbox1.DPad() != -1) {
            imu.reset(-xbox1.DPad());
        }

        // forward command (-1.0 to 1.0)
//		if (Math.abs(xbox1.LStickY()) >= 0.5) {
//        FWD = -xbox1.LStickY()/* / 10.5 * Ds.getBatteryVoltage() * 1.0*/;
//
//		}

        // strafe command (-1.0 to 1.0)
//		if (Math.abs(xbox1.LStickX()) >= 0.05) {
//        STR = xbox1.LStickX() /*/ 10.5 * Ds.getBatteryVoltage() * 1.0*/;
//		}

        if (!xbox1.RStickButton()) {
            RCW =  xbox1.RStickX() * 0.5;
        } else {
            RCW = xbox1.RStickX();
        }

        if (!xbox1.LStickButton()) {
            FWD = -xbox1.LStickY();
            STR = xbox1.LStickX();
        } else {
            FWD = -xbox1.LStickY() * 0.5;
            STR = xbox1.LStickX() * 0.5;
        }

        FWD *= 1.04-Elevator.getPosition()/100.0;
        STR *= 1.04-Elevator.getPosition()/100.0;


        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).turbo(xbox1.LStickButton() || xbox1.RStickButton());
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).turbo(xbox1.LStickButton() || xbox1.RStickButton());
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).turbo(xbox1.LStickButton() || xbox1.RStickButton());
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).turbo(xbox1.LStickButton() || xbox1.RStickButton());

//		if (cmdCounter > 4) {
//			cmdCounter = 0;
//		}

//		System.out.println(STR);


//        if (imu.collisionDetected()) {
//            xbox1.rumbleRight(1.0);
//            xbox1.rumbleLeft(1.0);
//        } else {
//            xbox1.stopLeftRumble();
//            xbox1.stopRightRumble();
//        }

//        if (rumble) {
//            xbox2.rumbleRight(0.5);
//            xbox2.rumbleLeft(0.5);
//            rumbleTime++;
//            if (rumbleTime > 20) {
//                rumble = false;
//            }
//        } else {
//            xbox2.stopRumble();
//        }

        double headingDeg = imu.getAngle(); //Maybe change heading deg based off of RCW cmd
//        headingDeg += RCW*25.0;         //If RCW is positive, have a lesser angle. If it is negative, have a larger angle
        headingRad = Math.toRadians(headingDeg);

        currentOrientedButton = xbox1.RB();
        if (currentOrientedButton && !previousOrientedButton) {
            fieldOriented = !fieldOriented;
            reverseRobotOriented = false;
        }
        previousOrientedButton = currentOrientedButton;

        //Reverse Robot Oriented Attempt #1
        currentReverseRobotOrientedButton = xbox1.LB();
        if (currentReverseRobotOrientedButton && !previousReverseRobotOrientedButton) {
            fieldOriented = false;
            reverseRobotOriented = !reverseRobotOriented;
        }
        previousReverseRobotOrientedButton = currentReverseRobotOrientedButton;

        SmartDashboard.putBoolean("Field Oriented", fieldOriented);


        RCW *= SmartDashboard.getNumber("Rotation Scale Factor", 1.0);

        FWD *= SmartDashboard.getNumber("Translation Scale Factor", 1.0);
        STR *= SmartDashboard.getNumber("Translation Scale Factor", 1.0);

        if (fieldOriented) {
            double angleOfMovement; //180 on smartDashboard at rest

            if (FWD == 0.0 && STR > 0.0) { //Because dividing by 0.0 is mean, hard code the values
                angleOfMovement = 90.0;
            } else if (FWD == 0.0 && STR < 0.0) {
                angleOfMovement = 270.0;
            } else {
                angleOfMovement = Math.toDegrees(MathUtils.resolveAngle(Math.atan2(STR, FWD)));
            }

            double polarLength = MathUtils.pythagorean(STR, FWD);
            double compensatedAngle = MathUtils.resolveDeg(angleOfMovement - RCW * SmartDashboard.getNumber("Angle Compensation", 100.0));
            //Adjust the angle of translation by a scalar of the RCW command, so it mimics the joystick maneuver that counteracts the drift
            STR = polarLength * Math.sin(Math.toRadians(compensatedAngle));
            FWD = polarLength * Math.cos(Math.toRadians(compensatedAngle));

            SmartDashboard.putNumber("Angle of Movement", angleOfMovement);

            Vector commands;
            commands = MathUtils.convertOrientation(headingRad, FWD, STR);
            FWD = commands.getY();
            STR = commands.getX();
        } else {
            FWD *= 0.5;
            STR *= 0.5;
            RCW *= 0.5;

            if (reverseRobotOriented) {
                FWD *= -1;
                STR *= -1;
            }
        }


//        System.out.println(xbox1.DPad());
//        System.out.println(MathUtils.calculateContinuousError(45.0, imu.getAngle(), 360.0, 0.0));
//        System.out.println(Math.abs(MathUtils.calculateContinuousError(45.0, imu.getAngle(), 360.0, 0.0)) >= 3);


        if (xbox1.buttonPad() != -1 && xbox1.buttonPad() != 0.0) {
            FWD = 0.0;
            STR = 0.0;
            RCW = 0.0;
        }


        if (xbox1.buttonPad() == 45) {
            sensorToLookAt = 1;
            if (!cameraAlign(30.0, -hatchFrontOffset, 8.4)) {
//                    !wallAlign(30.0, 35.0, 13.5, 6.5)) { //FIXME, was 6.5
                STR += xbox1.LStickX() * placeholderName;
                FWD += xbox1.LStickY() * placeholderName;
                RCW += xbox1.RStickX() * placeholderName;

                STR = 0.0;
                FWD = 0.0;
                RCW = 0.0;

                camHeadingAngle(30.0);
            }

        } else if (xbox1.buttonPad() == 135) {
            sensorToLookAt = 2;
            if (!cameraAlign(150.0, -hatchFrontOffset, 8.4)) {
//                    !wallAlign(150.0, 130.0, 13.5, 6.5)) {
                STR += xbox1.LStickX() * placeholderName;
                FWD += xbox1.LStickY() * placeholderName;
                RCW += xbox1.RStickX() * placeholderName;

                STR = 0.0;
                FWD = 0.0;
                RCW = 0.0;

                camHeadingAngle(150.0);
            }



        } else if (xbox1.buttonPad() == 180 && xbox1.LTrig() > 0.25) {
            grabAlign = true;
            sensorToLookAt = 1;
            if (!cameraAlign(180.0, -hatchFrontOffset, hatchSideOffset)) {
//                    wallAlign(180.0,  230.0, 12.0, 3.5)) {
                RCW += xbox1.RStickX() * placeholderName;
                STR += xbox1.LStickX() * placeholderName;
                FWD += xbox1.LStickY() * placeholderName;

                STR = 0.0;
                FWD = 0.0;
                RCW = 0.0;

                camHeadingAngle(180.0);
            }


        } else if (xbox1.buttonPad() == 180 && xbox1.RTrig() > 0.25) {
            grabAlign = true;
            sensorToLookAt = 2;
            if (!cameraAlign(180.0, -hatchFrontOffset, hatchSideOffset)) {
//                    !wallAlign(180.0, 130.0, 12.0, 3.5)) {
                RCW += xbox1.RStickX() * placeholderName;
                STR += xbox1.LStickX() * placeholderName;
                FWD += xbox1.LStickY() * placeholderName;

                STR = 0.0;
                FWD = 0.0;
                RCW = 0.0;

                camHeadingAngle(180.0);
            }


        } else if (xbox1.buttonPad() == 225) {
            sensorToLookAt = 1;
            if (!cameraAlign(210.0, -hatchFrontOffset, 8.4)) {
//                    !wallAlign(210.0, 240.0, 13.5, 6.5)) {
                STR += xbox1.LStickX() * placeholderName;
                FWD += xbox1.LStickY() * placeholderName;
                RCW += xbox1.RStickX() * placeholderName;

                STR = 0.0;
                FWD = 0.0;
                RCW = 0.0;

                camHeadingAngle(210.0);
            }

        } else if (xbox1.buttonPad() == 315) {
            sensorToLookAt = 2;
            if (!cameraAlign(330.0, -hatchFrontOffset, 8.4)) {
//                    !wallAlign(330.0, 315.0, 14.0, 6.5)) {
                STR += xbox1.LStickX() * placeholderName;
                FWD += xbox1.LStickY() * placeholderName;
                RCW += xbox1.RStickX() * placeholderName;

                STR = 0.0;
                FWD = 0.0;
                RCW = 0.0;

                camHeadingAngle(330.0);
            }
        }

        if (xbox1.buttonPad() != -1) {
//            Vector commands;
//            commands = MathUtils.convertOrientation(headingRad, FWD, STR);
//            FWD = commands.getY();
//            STR = commands.getX();
        } else {
            grabAlign = false;
            headingVector = 0.0;
            useKeepAngle = true;
            prevCameraDistance = SmartDashboard.getNumber("Robot Distance", currentDistance);
            cameraCounter = 0;
            angleDelta = 0.0;
//            cargoCam.disable();
//            hatchCam.disable();
            latencyCounter = 0;
            camX = 0.0;
            camZ = 0.0;
            camAlignDone = false;
            sensorToLookAt = 0;
            wallAlignDone = false;
            wallAlignClose = false;
            useKeepAngle = true;
            prevWallAlignTime = Time.get();
            alignCase = 0;
        }

//        if (xbox1.RTrig() > 0.1) {
//            cargoCam.enable();
////            hatchCam.enable();
//
//            if (cameraCounter == 0) {
//                limelightSegmentDistance = limelightSegment("Hatch")[0];
//                tempAngle = 330.0; //Change to setpoint in full function
//            } else if (cameraCounter == 1) {
//                if (limelightSegment("Hatch")[1] != 0.0) {
//                    tempAngle = 330.0+limelightSegment("Hatch")[1];
//                    cameraCounter++;
//                }
//            }
//
//            if (limelightSegmentDistance != 0.0 && cameraCounter == 0) {
//                cameraCounter++;
//            }
//
//            headingRad = Math.toRadians(limelightSegment("Hatch")[1]);
//
//            teleopRCW.setInput(imu.getAngle());
//
//            double robotDistanceGone = SmartDashboard.getNumber("Robot Distance", currentDistance)-prevCameraDistance;
//
//            if (robotDistanceGone < limelightSegmentDistance*0.8) {
//                if (limelightSegment("Hatch")[0] > 20.0 && limelightSegment("Hatch")[0] < 60.0) { //Range where robot won't hit rocket
////                    System.out.println("Rotating");
//                    teleopRCW.setSetpoint(330.0+limelightSegment("Hatch")[1]); //Might not work for 180 degrees
//                    tempAngle = headingRad+imu.getAngle();
//                }
////                else {
//                    teleopRCW.setSetpoint(tempAngle);
////                }
//
//                FWD = 0.2*Math.cos(headingRad);
//                STR = 0.2*Math.sin(headingRad);
//            }
//
////            RCW = teleopRCW.performPID();
//
////            System.out.println(teleopRCW.performPID() + "  |  |  " + robotDistanceGone + "  |  |  " +  limelightSegmentDistance/*  + "  |  |  " +  limelightSegment("Hatch")[1]*/  + "  |  |  " +  FWD + "  |  |  " + STR);
//        } else {
//
//        }

        if (Math.abs(RCW) > 0.5) {
            RCW = Math.signum(RCW)*0.5;
        }

        acceleration();

        if (xbox2.buttonPad() == 90.0) {
//            RRLogger.addData("Placing FRR", Lidar.getFRRightSensor());
//            RRLogger.addData("Placing FRF", Lidar.getFRFrontSensor());
//            RRLogger.addData("Placing FLF", Lidar.getFLFrontSensor());
//            RRLogger.addData("Placing FLL", Lidar.getFLLeftSensor());
//            RRLogger.newLine();
//            RRLogger.writeFromQueue();
        }

        if (xbox2.buttonPad() == 180.0) {
//            RRLogger.addData("Grabbing FRR", Lidar.getFRRightSensor());
//            RRLogger.addData("Grabbing FRF", Lidar.getFRFrontSensor());
//            RRLogger.addData("Grabbing FLF", Lidar.getFLFrontSensor());
//            RRLogger.addData("Grabbing FLL", Lidar.getFLLeftSensor());
//            RRLogger.newLine();
//            RRLogger.writeFromQueue();
        }

        if (xbox2.buttonPad() == 0.0) {
//            RRLogger.addData("Cargo Distance 1", Lidar.getBLLeftSensor());
//            RRLogger.addData("Cargo Distance 2", Lidar.getBRRightSensor());
//            RRLogger.newLine();
//            RRLogger.writeFromQueue();
        }

        Cargo.set((!Elevator.atPosition() && Elevator.getCommand() > Elevator.getPosition()), xbox2.X(), xbox2.Y(), Lidar.hasCargo(), Lidar.getBLLeftSensor(), Lidar.getBRRightSensor());
        SmartDashboard.putNumber("Hold State", Cargo.getHoldState());

        if (!Hatch.isRunning() && Elevator.getPosition() < 102.0 && Elevator.getPosition() > 1.0) {
            Elevator.setPosition(xbox2.LStickY(), xbox2.LTrig(), xbox2.LStickX(), xbox2.LStickButton(), xbox2.X(), xbox2.LB());
        }

        if (Elevator.atPosition()) {
            if (Hatch.set(xbox2.A(), xbox2.B(), false, safeToPutHatch) || xbox2.Y()) {
                FWD = 0.0;
                STR = 0.0;
                RCW = 0.0;
                Elevator.setPosition(0, 0, 0, true, false, false); //Maybe add a keepPosition boolean
            }
        }

        //Align to the sides of the rocket
        if (!buttonTimeIncrease && Time.get() - prevButtonTime > 0.3 && xbox1.buttonPad() == 90.0) {
            sideRocketAlign(270.0);
        } else if (!buttonTimeIncrease && Time.get() - prevButtonTime > 0.3 && xbox1.buttonPad() == 270.0) {
            sideRocketAlign(90.0);
        }

        if (xbox1.buttonPad() == 90.0 || xbox1.buttonPad() == 270.0) {
            buttonTimeIncrease = false;
        } else {
            buttonTimeIncrease = true;
            prevButtonTime = Time.get();
        }

//        if (wallAlignClose) {
//            wallAlignClose();
//        }

        //LTrig faces right side of cargoship align. RTrig faces left side
        /*
        if (xbox1.buttonPad() == -1 && !trigTimeIncrease && Time.get() - prevTrigTime > 0.3 && xbox1.LTrig() != 0.0) { //This will face the right side of the cargo

            if (imu.getAngle() > 180.0) { //This will move for the hatch side, IMU between 180 and 360
                cargoshipAlign(270.0, 1, STR/2.0, FWD/2.0); //Move robot forward

            } else { //This will move for the cargo side
                cargoshipAlign(90.0, -1, STR/2.0, FWD/2.0); //Move robot backward
            }

        } else if (xbox1.buttonPad() == -1 && !trigTimeIncrease && Time.get() - prevTrigTime > 0.3 && xbox1.RTrig() != 0.0) { //This will face the left side of the cargo

            if (imu.getAngle() > 180.0) { //If the hatch is on the left side of the robot, move for cargo
                cargoshipAlign(270.0, -1, STR/2.0, FWD/2.0); //Move robot backward

            } else {
                cargoshipAlign(90.0, 1, STR/2.0, FWD/2.0); //Move robot forward
            }
        } else if (!trigTimeIncrease && Time.get() - prevTrigTime > 0.3 && xbox1.buttonPad() == 0.0) {
            if (imu.getAngle() > 270 || imu.getAngle() < 90.0) { //If hatch is closer to the front of the cargoship
                cargoshipAlign(0.0, 1, STR/2.0, FWD/2.0); //Move robot forward

            } else {
                cargoshipAlign(180.0, -1, STR/2.0, FWD/2.0); //Move robot backward

            }
        } else {
            cargoShipDone = false;
            seenSensor = false;
        }
        */

        if (camAlignDone/*wallAlignDone || cargoShipDone ||
                (atGoodDistance((Lidar.getFLFrontSensor() < Lidar.getFRFrontSensor()) ? Lidar.getFLFrontSensor(): Lidar.getFRFrontSensor(), lastTargetFrontDistance, wallAlignError) &&
                atGoodDistance((Lidar.getFRRightSensor() < Lidar.getFLLeftSensor()) ? Lidar.getFRRightSensor(): Lidar.getFLLeftSensor(), lastTargetSideDistance, wallAlignError) &&
                 Math.abs(MathUtils.calculateContinuousError(lastTargetAngle, imu.getAngle(), 360.0, 0.0)) < 2.8)*/) {
            if (Time.get() - prevRumbleTimer > 0.2) {
                xbox2.rumbleLeft(1.0);
            } else {
                xbox2.stopLeftRumble();
            }
        } else {
            xbox2.stopLeftRumble();
            prevRumbleTimer = Time.get();
        }

//        System.out.println(xbox1.LTrig() + "|" + xbox1.RTrig() + "|" + xbox1.buttonPad());
        if (((xbox1.LTrig() != 0.0 || xbox1.RTrig() >= 0.0) && xbox1.buttonPad() == -1) || xbox1.buttonPad() == 0.0) {
            trigTimeIncrease = false;
        } else {
            trigTimeIncrease = true;
            prevTrigTime = Time.get();
        }

//        if ((Math.abs(RCW) > 0.1/(Math.abs(FWD) +Math.abs(STR))) && (Math.abs(FWD) + Math.abs(STR) != 0.0)) {
//            RCW *= 0.1/(Math.abs(FWD) + Math.abs(STR));
//        }

//        if ((Math.abs(RCW) > 0.2 && (Math.abs(FWD) + Math.abs(STR) > 0.1))) {
//            RCW *= 0.2;
//        }

//        System.out.println("RCW:  " + RCW);

        keepAngle();

        SmartDashboard.putNumber("FWD", FWD);
        SmartDashboard.putNumber("STR", STR);
        SmartDashboard.putNumber("RCW", RCW);
        SmartDashboard.putNumber("IMU Angle", imu.getAngle());

        habStart = xbox2.Start();

        SmartDashboard.putNumber("Lift Encoder", Lift.getDistance());
        if (!doClimb) {
            if (habStart) {
//            System.out.println("1");
//                HABAlign();
                if (HABAlign()) {
//                    if (Time.get() - habTime > 0.2) {
                        doClimb = true;
////                    System.out.println("2");
                    }
//                } else {
//                    habTime = Time.get();
//                }
            } else {
                habTime = Time.get();
                habLidarTimeout = Time.get();
            }
        }

        if (doClimb) {
//            System.out.println("3");
            System.out.println("Climbing");
            Lift.climb(xbox2.RStickY());

            if (xbox2.Back()) {
                Lift.setManual();
            }
        }


        rotationValues = MathUtils.instertionSort(new double[] {SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getRotationCommand(),
                SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getRotationCommand(),
                SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getRotationCommand(),
                SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getRotationCommand()});

        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).scaleSpeedCommand(rotationValues[3]);
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).scaleSpeedCommand(rotationValues[3]);
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).scaleSpeedCommand(rotationValues[3]);
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).scaleSpeedCommand(rotationValues[3]);

        /*
        Compensation Code:
        frictionCompensate();
         */

//        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setRotationCommand(xbox1.RStickY());
//        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setRotationCommand(xbox1.RStickY());
//        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setRotationCommand(xbox1.RStickY());
//        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setRotationCommand(xbox1.RStickY());


//        if (xbox1.RTrig() > 0.5) {
//            autoRCW.setSetpoint(330.0);
//            autoRCW.setInput(imu.getAngle());
//
//            RCW = autoRCW.performPID() *0.3;
//        }

        driveTrain.drive(new Vector(-STR, FWD), RCW);



        SmartDashboard.putNumber("FR Angle", SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getAngle());
        SmartDashboard.putNumber("FL Angle", SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getAngle());
        SmartDashboard.putNumber("BL Angle ", SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getAngle());
        SmartDashboard.putNumber("BR Angle ", SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getAngle());

        SmartDashboard.putNumber("FR Angle Command", SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getAngleCommand());
        SmartDashboard.putNumber("FL Angle Command", SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getAngleCommand());
        SmartDashboard.putNumber("BL Angle Command", SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getAngleCommand());
        SmartDashboard.putNumber("BR Angle Command", SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getAngleCommand());
    }

    //0.8,1.0,45.0,25.0,999.0,0,999,0,0,0,0,0,4.0,0,0,0,0,1.0,13.2,5.2
    //0.0,0.0,0.0,25.0,999.0,0.0,999.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,1.0,0.0,0.0,0.0,0.0
    //0.8,1.0,120.0,180.0,999.0,0,999,0,0,0,0,0,6.0,0.0,0,0,0,1.0,12.0,2.5
    //0.0,0.0,0.0,180.0,999.0,0.0,999.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,2.0,0.0,0.0,0.0,0.0
    //0.8,1.0,60.0,25.0,999.0,0,999,0,0,0,0,0,6.0,0,0,0,0,1.0,13.5,5.4
    //0.0,0.0,0.0,25.0,999.0,0.0,999.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,2.0,1.0,0.0,0.0,0.0,0.0

    public void robotPeriodic() {
        currentDistance += SwerveDrivetrain.getRobotDistance() * 1.8; //Scale factor for translation
        SmartDashboard.putNumber("Robot Distance", currentDistance);

        SmartDashboard.putNumber("Cargo Azimuth", cargoCam.tx());
        SmartDashboard.putNumber("Cargo Elevation", cargoCam.ty());

        SmartDashboard.putNumber("Hatch Azimuth", hatchCam.tx());
        SmartDashboard.putNumber("Hatch Elevation", hatchCam.ty());

        SmartDashboard.putNumber("Cargo X", cargoCam.getDistance()[0]);
        SmartDashboard.putNumber("Cargo Y", cargoCam.getDistance()[1]);
        SmartDashboard.putNumber("Cargo Z", cargoCam.getDistance()[2]);

        SmartDashboard.putNumber("Polar Distance", limelightSegment("Hatch")[0]);
        SmartDashboard.putNumber("Polar Angle", limelightSegment("Hatch")[1]);


        SmartDashboard.putNumber("Distance from Target", cargoCam.getDistance()[2]);
        SmartDashboard.putNumber("Cam Vector Absolute Angle", camHeadingAngle);



        SmartDashboard.putNumber("Hatch X", hatchCam.getDistance()[0]);
        SmartDashboard.putNumber("Hatch Y", hatchCam.getDistance()[1]);
        SmartDashboard.putNumber("Hatch Z", hatchCam.getDistance()[2]);

        if (xbox1.Start()) {
            driveTrain.resetWheels();
        }

        if (xbox1.LTrig() + xbox1.RTrig() > 0.1 || xbox1.buttonPad() != -1) {
            cargoCam.enable();
            hatchCam.enable();
        } else {
            cargoCam.disable();
            hatchCam.disable();
        }

        Lidar.read();
        LineSensor.get();

        SmartDashboard.putNumber("Sensor 1", Lidar.getFRRightSensor());
        SmartDashboard.putNumber("Sensor 2", Lidar.getFRFrontSensor());
        SmartDashboard.putNumber("Sensor 3", Lidar.getFLFrontSensor());
        SmartDashboard.putNumber("Sensor 4", Lidar.getFLLeftSensor());
        SmartDashboard.putNumber("Sensor 5", Lidar.getBLLeftSensor());
        SmartDashboard.putNumber("Sensor 6", Lidar.getBRRightSensor());
        SmartDashboard.putNumber("Sensor 7", Lidar.hasCargo());
    }

    public void disabledInit() {
//		jet.setDisabled();

        FWD = 0.0;
        STR = 0.0;
        RCW = 0.0;
        prevFWD[cmdCounter] = 0.0;
        prevSTR[cmdCounter] = 0.0;
        prevRCW[cmdCounter] = 0.0;
//Todo, was for the pin
//        Lift.reset();

        autoMove = 0;
        Elevator.setPower(0.0);
        driveTrain.drive(new Vector(0.0,0.0),  0.0);
        // When robot is turned on, disabledInit is called once

        if (disabled < 1) {
            if (Math.random() % 2 == 1) {
                System.out.println("Hello, I am Suzie");
            } else {
                System.out.println("Hello, I am Maya");
            }
            disabled++;
        } else {
            xbox2.stopRightRumble();
            xbox2.stopLeftRumble();
//            RRLogger.close();
            SwerveDrivetrain.resetDeltas();
        }
        cargoCam.disable();
        hatchCam.disable();
    }

    /**
     * This function is called periodically during test mode
     */
        public void testPeriodic() {
            // LABEL test`

            if (xbox1.LTrig() + xbox1.RTrig() > 0.1 || xbox1.buttonPad() != -1) {
                cargoCam.enable();
                hatchCam.enable();
                System.out.println("On");
            } else {
                cargoCam.disable();
                hatchCam.disable();
                System.out.println("Off");
            }
    }
}