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
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SwerveDrivetrain.WheelType;
import frc.robot.utilities.*;
import frc.robot.RRLogger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import org.opencv.core.Mat;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update the manifest file in the
 * resource directory.
 */
public class Robot extends TimedRobot {

    private double ACCEL_SPEED= 0.1/*0.031*/;
    private double DECEL_SPEED= 0.1; //0.035, 0.05

    private SendableChooser<Integer> startingSide = new SendableChooser<>();
    private SendableChooser<Integer> autoChooser = new SendableChooser<>();

    private Compressor compressor;

    public static XboxController xbox1 = new XboxController(0);
    public static XboxController xbox2 = new XboxController(1);

    //	private JetsonServer jet;
//	private Thread t;
    private SwerveDrivetrain driveTrain;
    private IMU imu;

//	private boolean robotBackwards;

    private double robotOffset;
    private boolean seenSensor = false;
    private double lastTargetSideDistance = 0.0;
    private double lastTargetFrontDistance = 0.0;
    private double lastTargetAngle = 0.0;

    private boolean habDistanceGood = false;
    private double cargoAlignTime = 0.0;
    private double prevRumbleTimer = 0.0;
    private double rotationMax = 1.0;
    private double[] rotationValues;
    private double accelTimeChanger = 0.0;
    private int disabled = 0;
    private double prevTrigTime = 0.0;
    private boolean trigTimeIncrease = true;
    private int wheelStopCase = 0;
    private double wallAlignDecelDistance = 90.0;
    private double habTime = 0.0;
    private double sideAlignP = 1.6e-2;
    private double frontAlignP = 1.6e-2;

    private double sideAlignI = 0.0;
    private double frontAlignI = 0.0;
    private double wallAlignError = 1.1;

    private boolean habStart = false;
    private double[][] commands;
    private boolean doClimb = false;
    private boolean prevStartButton = false;
    private boolean prevBackButton = false;
    private boolean compensateToggle = true;
    private int arrayIndex = -1;
    private int autoMove = 0;
//    private int translateType;
//    private double autonomousAngle;
    private double tSpeed;
    private double rSpeed;
    private double previousDistance = 0.0;
    private double currentDistance = 0.0;
    private double habLidarTimeout = 0.0;
    private double rotationAlignP = 0.005;
    private boolean override;
    private boolean driveDone;
    private boolean turnDone;
    private boolean elevatorDone;
    private boolean hatchDone;
    private boolean cargoDone;
    private boolean timeDone;
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

    private boolean autonomous;

    private boolean fieldOriented = true; // start on field orientation
    private boolean previousOrientedButton = false;
    private boolean currentOrientedButton = false;
    private boolean buttonTimeIncrease = false;
    private boolean wheelsGood = false;

    private PIDController SwerveCompensate;
    private PIDController wallAlignSTR = new PIDController(sideAlignP, 0.0, 0.0);
    private PIDController wallAlignFWD = new PIDController(frontAlignP, 0.0, 0.0);
    private PIDController wallAlignRCW = new PIDController(rotationAlignP, 0.0, 0.0);


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
    private double[] ArcXs = new double[3];
    private double[] ArcYs = new double[3];
    double[] curveVelocity = {0.0, 0.0};
    private  double position = 0.0;
    private int currentIndex = 0;
    private int prevIndex = -1;
    private double totalArcLength = 0.0;
    private double deltaX = 0.0;
    private double deltaY = 0.0;

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

    private void acceleration() {
//        if (FWD + STR != 0.0) {
//            if (Math.abs(RCW) > Math.abs(prevRCW[cmdCounter])) {
//                if (Math.abs(RCW - prevRCW[cmdCounter]) > ACCEL_SPEED) {
//                    if (RCW - prevRCW[cmdCounter] > 0.0) {
//                        RCW = prevRCW[cmdCounter] + ACCEL_SPEED;
//                    } else {
//                        RCW = prevRCW[cmdCounter] - ACCEL_SPEED;
//                    }
//                }
//            } else {
//                if (Math.abs(RCW - prevRCW[cmdCounter]) > DECEL_SPEED) {
//                    if (RCW - prevRCW[cmdCounter] > 0.0) {
//                        RCW = prevRCW[cmdCounter] + DECEL_SPEED;
//                    } else {
//                        RCW = prevRCW[cmdCounter] - DECEL_SPEED;
//                    }
//                }
//            }
//        }

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
            frontDistance = 5.5;
            if (rotationDone && (frontDistance-2.0 > distanceFromCargoship || distanceFromCargoship > frontDistance+2.0)) { //Tweak with real field values
                FWD = Math.signum(sideToFace) * (distanceFromCargoship-frontDistance) * 0.16; //Decreases speed with distance from cargoShip
            } else {
                forwardDone = true;
                FWD = 0.0;
            }
        }

        if (Math.abs(FWD) > 0.16) {
            FWD *= 0.16;
        }

        cargoShipDone = (rotationDone && forwardDone);

//        System.out.println(rotationDone + "|" + forwardDone + "|" + strafeDone + "|" + cargoShipDone);
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

        double side = (Lidar.getFRRightSensor() < Lidar.getFLLeftSensor()) ? Lidar.getFRRightSensor() : Lidar.getFLLeftSensor();
        double front = (Lidar.getFRFrontSensor() < Lidar.getFLFrontSensor()) ? Lidar.getFRFrontSensor() : Lidar.getFLFrontSensor();

        double farSide = MathUtils.instertionSort(distances)[2];

//        System.out.println(alignCase);

        if (atGoodDistance(front, frontDistance, 3.1)) {
            sideAlignI = 2.5e-4; //sideAlignP/100.0;
            frontAlignI = 2.5e-4; //frontAlignP/100.0;
            wallAlignFWD.setMaximumI(0.0);
            wallAlignSTR.setMaximumI(0.0);
        } else {
            wallAlignFWD.resetI();
            wallAlignSTR.resetI();

            sideAlignI = 0.0;
            frontAlignI = 0.0;
            wallAlignFWD.setMaximumI(0.0);
            wallAlignSTR.setMaximumI(0.0);
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


                System.out.println(wallAlignRCW.getError() + "|" + wallAlignRCW.performPID());
//                System.out.println(angle + " | | " + robotAngle);
                if (angle < 90.0 || angle > 270.0) {
                    tempAngle = 0.0;
                    System.out.println("Here");
                } else {
                    tempAngle = 180.0;
                }

                wallAlignRCW.setSetpoint(tempAngle);

                if (Math.abs(MathUtils.calculateContinuousError(tempAngle, robotAngle, 360.0, 0.0)) >= 5.0) {
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
                //Don't look while angle isn't at the right angle
                if (angle < 90.0 || angle > 270.0) {
                    tempAngle = 0.0;
                } else {
                    tempAngle = 180.0;
                }

                if (Math.abs(MathUtils.calculateContinuousError(tempAngle, imu.getAngle(), 360.0, 0.0)) < 3.5 && front < 30.0) {
                    wallAlignClose = true;
                }

                if (!wallAlignClose) {
                    angle = tempAngle;
                    sideDistance = sideDistance * Math.abs(Math.sin(Math.toRadians(90.0-angle)));
                }
//                if (!wallAlignClose && Math.abs(MathUtils.calculateContinuousError(tempAngle, imu.getAngle(), 360.0, 0.0)) > 3.5 || front > 30.0) {
//                    angle = tempAngle;
//                    sideDistance = sideDistance * Math.abs(Math.sin(Math.toRadians(90.0-angle)));
//                } else {
//                    wallAlignClose = true;
//                }

                System.out.println(wallAlignClose + "|" + imu.getAngle() + "|" + front);

                wallAlignSTR.setSetpoint(sideDistance);
                wallAlignFWD.setSetpoint(frontDistance);
                wallAlignSTR.setInput(side);
                wallAlignFWD.setInput(front);
                wallAlignFWD.setOutputRange(-0.6, 0.6);
                wallAlignSTR.setOutputRange(-0.6, 0.6);
                wallAlignRCW.setOutputRange(-0.3, 0.3);
                wallAlignRCW.setSetpoint(angle);

//                System.out.println(angle);
                if (Math.abs(MathUtils.calculateContinuousError(angle, imu.getAngle(), 360.0, 0.0)) >= 2.8) {
                    RCW = wallAlignRCW.performPID();
//                    System.out.println(RCW);
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
//                    System.out.println("RCW Done");

                    done++;
                }

//                if (robotAngle < angle-10.0 || robotAngle > angle + 10.0) { //Might need to adjust distances w/ bumper
////                    side = (front < side) ? front: side;
//                    front = farSide;
//                }

                //        min dist to wall        max dist to wall
//                System.out.println(side + "| |" + sideDistance);
                if (!atGoodDistance(side, sideDistance, wallAlignError)) { //Decrease allowed error as much as possible. Log lidar values in comp. to get experimental mean and standard deviation

                    STR = Math.sin(Math.toRadians(moveAngle)) * -wallAlignSTR.performPID();


//                    if (Math.abs(STR) < minSpeed) {
//                        STR = Math.signum(STR)*minSpeed;
//                    }
                } else {
//                    System.out.println("STR Done");
                    STR = 0.0;
                    done++;
                }
                // max speed
//                if (Math.abs(STR) > Math.sin(Math.toRadians(moveAngle)) * Math.signum(side-sideDistance) * (side/wallAlignDecelDistance)) {
//                    STR = Math.sin(Math.toRadians(moveAngle)) * Math.signum(side-sideDistance) * (side/wallAlignDecelDistance); //Scale speed to distance from wall
//                }


//                if (side < useCushionDistance) { //Farside allows robot to speed up during front cycle
//                    STR *= cushionMaxSpeed;
//                    sideAlignP = 1.45e-2;
//                } else {
//                    sideAlignP = 1.45e-2;
//                }

                //  min dist to wall                            max dist to wall
                if (!atGoodDistance(front, frontDistance, wallAlignError)) {
                    //                                                                                              setpoint                  P
                    FWD = Math.cos(Math.toRadians(moveAngle)) * -wallAlignFWD.performPID(); //0.025
//                    if (Math.abs(FWD) < minSpeed) {
//                        FWD = Math.signum(FWD)*minSpeed;
//                    }
//                    System.out.println("FWD: " + FWD);

//            System.out.println("FWD BAD");
                    // max speed
//                    if (Math.abs(FWD) > Math.cos(Math.toRadians(moveAngle)) * Math.signum(front-frontDistance) * (front/wallAlignDecelDistance)) {
//                        FWD = Math.cos(Math.toRadians(moveAngle)) * Math.signum(front-frontDistance) * (front/wallAlignDecelDistance);
//                    }

//                    if (front < useCushionDistance) {
//                        FWD *= cushionMaxSpeed;
//                        frontAlignP = 1.45e-2;
//                    } else {
//                        frontAlignP = 1.45e-2;
//                    }

                } else {
//                    System.out.println("FWD Done");
                    FWD = 0.0;
                    done++;
                }

                System.out.println(wallAlignSTR.performPID() + " | | " + wallAlignFWD.performPID());
                break;
        }

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

            if(front < 14.5){
                FWD = -0.04;
            } else {
                habDistanceGood = true;
                fwdDone = true;
//            }

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

                if (frontLeft < 9.5 && frontRight < 9.5) {
                    done++;
                } else if (frontLeft >= 9.0) { //If only BL edge is off cargo, move robot right
                    STR = 0.08 + (frontLeft-frontRight) * 0.004;
                } else if (frontRight >= 9.0) {
                    STR = -0.08 - (frontRight-frontLeft) * 0.004;
                }

                if (frontLeft > 11.0 && frontRight > 11.0) {
                    FWD = -0.2;
                }
        }

        SmartDashboard.putBoolean("Cargo Aligned", (done == 2));
        return (done == 2);
    }

    private void keepAngle() { //FIXME! Ian wants this to change. It doesn't drive straight
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

            SwerveCompensate.setPID(0.015/*0.015*/, 0.0, 0.0); //FIXME, Ian wants this to not overshoot
            keepAngle = imu.getAngle();
        } else if (useKeepAngle){
            SwerveCompensate.setPID(0.008/*0.008*/, 0.0, 0.0);

            SwerveCompensate.setInput(imu.getAngle());
            SwerveCompensate.setSetpoint(keepAngle);

            if (!SwerveCompensate.onTarget()) {
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

        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setOffset(0.0/*259.1312255859375*/);
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setOffset(0.0/*73.0799560546875*/);
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setOffset(0.0/*253.8753662109375*/);
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setOffset(0.0/*77.2393798828125*/);
//		SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDrift(application.getProperty("front_right_drift", "0.0,0.0").split(","));
//		SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDrift(application.getProperty("front_left_drift", "0.0,0.0").split(","));
//		SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDrift(application.getProperty("back_left_drift", "0.0,0.0").split(","));
//		SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDrift(application.getProperty("back_right_drift", "0.0,0.0").split(","));

//		robotBackwards = Boolean.parseBoolean(application.getProperty("robot_backwards", "false"));
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

    //Might be helpful for verification
    //Once robotDistance+= calculateLength distance, finish step
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

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    public void robotInit() {
        wallAlignSTR.setPID(sideAlignP, sideAlignI, 0.0);
        wallAlignSTR.enable();
        wallAlignSTR.setInputRange(0.0, 30.0);
        wallAlignSTR.setOutputRange(-1.0, 1.0);
        wallAlignSTR.setContinuous(false);
        wallAlignSTR.setMaximumI(0.04);


        wallAlignFWD.setPID(frontAlignP, frontAlignI, 0.0);
        wallAlignFWD.enable();
        wallAlignFWD.setInputRange(0.0, 30.0);
        wallAlignFWD.setOutputRange(-1.0, 1.0);
        wallAlignFWD.setContinuous(false);
        wallAlignFWD.setMaximumI(0.04);

//        rotationAlignP = 0.005;
        wallAlignRCW.setPID(rotationAlignP, 0.0, 0.0);
        wallAlignRCW.enable();
        wallAlignRCW.setInputRange(0.0, 360.0);
        wallAlignRCW.setOutputRange(-rotationMax, rotationMax);
        wallAlignRCW.setContinuous(true);
        RRLogger.start();
        SmartDashboard.putString("RRLogger File", RRLogger.getFileName());
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

        xbox1.setDeadband(0.01);

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

        choice = "/home/lvuser/deploy/" + side + type + ".csv";

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

//                setpoint = (int) commands[arrayIndex][14];

                    if (commands[arrayIndex][2] != 999) {
                        FWD = Math.cos(Math.toRadians(commands[arrayIndex][2]));
                        STR = Math.sin(Math.toRadians(commands[arrayIndex][2]));
                    } else {
                        FWD = 0;
                        STR = 0;
                    }

                    if (ArcXs[0] != 999) {
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

                    if (Math.abs(MathUtils.getAngleError(imu.getAngle(), commands[arrayIndex][3])) < 5.0) {
                        initialAngle = imu.getAngle();
                        turnDone = true;
                    } else {
                        double direction;
                        direction = MathUtils.getAngleError(initialAngle, commands[arrayIndex][3]);
                        if (Math.abs(direction) > 180.0) {
                            direction *= -1.0;
                        }
                        RCW = Math.signum(direction);
                        turnDone = false;
                    }


                    if (commands[arrayIndex][5] == 1.0) {
                        smoothAccelerateNum = (MathUtils.convertRange(previousDistance, previousDistance + commands[arrayIndex][4], commands[arrayIndex][0], commands[arrayIndex + 1][0], SmartDashboard.getNumber("Robot Distance", 0)));
                        smoothAccelerate = smoothAccelerateNum;
                        FWD *= smoothAccelerate;
                        STR *= smoothAccelerate;
                    } else if (commands[arrayIndex][5] == 2.0) {
                        smoothAccelerateNum = (MathUtils.convertRange(previousDistance, previousDistance + commands[arrayIndex][4], minSpeed, commands[arrayIndex][0], SmartDashboard.getNumber("Robot Distance", 0)));
                        smoothAccelerate = smoothAccelerateNum;
                        FWD *= smoothAccelerate;
                        STR *= smoothAccelerate;
                    } else if (commands[arrayIndex][5] == 3.0) {
                        smoothAccelerateNum = (MathUtils.convertRange(previousDistance, previousDistance + commands[arrayIndex][4], commands[arrayIndex][0], minSpeed, SmartDashboard.getNumber("Robot Distance", 0)));
                        smoothAccelerate = smoothAccelerateNum;
                        FWD *= smoothAccelerate;
                        STR *= smoothAccelerate;
                    } else {
                        FWD *= tSpeed;
                        STR *= tSpeed;
                    }


                    SmartDashboard.putNumber("Previous Distance", previousDistance);

                    if ((Math.abs(SmartDashboard.getNumber("Robot Distance", 0) - previousDistance) >= commands[arrayIndex][4])) {
                        driveDone = true;
                    }

                    SmartDashboard.putNumber("Auto Distance Gone", Math.abs(currentDistance - previousDistance));
                    SmartDashboard.putNumber("Auto Distance Command", commands[arrayIndex][4]);

                    SwerveCompensate.setTolerance(1);

                    if (Time.get() > timeBase + commands[arrayIndex][12] && commands[arrayIndex][12] > 0) {
                        override = true;
                    } else if (commands[arrayIndex][12] == 0) {
                        timeDone = true;
                    }

                    imuOffset = commands[arrayIndex][13];

                    if (turnDone) {
                        keepAngle();
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

//                        if (commands[arrayIndex][17] == 2.0) {
//                            if (alignCase == 2 && imu.getAngle() - 20.0 > commands[arrayIndex][3] || imu.getAngle() + 20.0 < commands[arrayIndex][3]) {
//                                alignCase = 1;
//                            }

//                            alignCase = 2;
//                            rotationAlignP = 1.3e-3;
//                            rotationAlignP = 0.32e-3;
//                            rotationMax = 0.5;
//                            rotationMax = 0.3;
//                            rotationAlignP = 8.0e-4;
                        rotationMax = 0.5;
//                        rotationAlignP = 0.005;
//                        }

                        useCushionDistance = commands[arrayIndex][20];
                        cushionMaxSpeed = commands[arrayIndex][21];

//                        System.out.println((!xbox1.LB() +"| |"  + (Time.get() - prevWallAlignTime < 1.0)));

                        if (wallAlign(commands[arrayIndex][3], commands[arrayIndex][2], commands[arrayIndex][18], commands[arrayIndex][19])) {
                            override = true;
                        }

                        FWD *= commands[arrayIndex][0];
                        STR *= commands[arrayIndex][0];
                        RCW *= commands[arrayIndex][1];
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

//                    System.out.println(driveDone + "  " + autoOverride + "  " + FWD + "  " + STR);

//                    System.out.println(arrayIndex + " || " + driveDone + " || " + elevatorDone + " || " + hatchDone + " || " + Elevator.atPosition());

                    if (Math.abs(commands[arrayIndex][22]) == 1.0) {
                        STR = 0.0;
                        FWD = 0.0;
                        System.out.println(commands[arrayIndex][22]);
                        cargoshipAlign(commands[arrayIndex][3], (int) commands[arrayIndex][22], commands[arrayIndex][0], 0.0);
//                        System.out.println(cargoDone);
                        if (cargoShipDone) {
                            override = true;
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

    public void teleopInit() {
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
            xbox2.rumbleRight(0.2);
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
            RCW = xbox1.RStickX() * 1.9;
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
        double headingRad = Math.toRadians(headingDeg);

        currentOrientedButton = xbox1.RB();
        if (currentOrientedButton && !previousOrientedButton) {
            fieldOriented = !fieldOriented;
        }
        previousOrientedButton = currentOrientedButton;

        SmartDashboard.putBoolean("Field Oriented", fieldOriented);


        if (fieldOriented) { //The wrong command is not being set above here, so the math here is offset when translating while rotating
            Vector commands;
            commands = MathUtils.convertOrientation(headingRad, FWD, STR);
            FWD = commands.getY();
            STR = commands.getX();
        } else {
            FWD *= 0.5;
            STR *= 0.5;
            RCW *= 0.5;
        }

//        System.out.println(xbox1.DPad());
//        System.out.println(MathUtils.calculateContinuousError(45.0, imu.getAngle(), 360.0, 0.0));
//        System.out.println(Math.abs(MathUtils.calculateContinuousError(45.0, imu.getAngle(), 360.0, 0.0)) >= 3);


        if (xbox1.buttonPad() != -1 && xbox1.buttonPad() != 0.0) {
            FWD = 0.0;
            STR = 0.0;
            RCW = 0.0;
        }

        //Old auto lineup
        //0.4,0.0,87.0,0.0,50.0,0,999,0,0,0,0,0,0.0,0,0,0,0,0,0,0
        //0.8,0.1,13.0,25.0,105.0,0,999,0,0,0,0,0,0,0,0,0,0,0,0,0

        if (xbox1.buttonPad() == 45) {
            if (!wallAlign(25.0, 45.0, 12.0, 5.8)) { //TN Comp Values, tweak for worlds field
                STR += xbox1.LStickX() * placeholderName;
                FWD -= xbox1.LStickY() * placeholderName;
            }

        } else if (xbox1.buttonPad() == 135) {
            if (!wallAlign(155.0, 130.0, 12.0, 5.0)) { //TN, 13.5
                STR += xbox1.LStickX() * placeholderName;
                FWD -= xbox1.LStickY() * placeholderName;
            }

        } else if (xbox1.buttonPad() == 180 && xbox1.LTrig() > 0.25) {
            if (!wallAlign(180.0,  180.0, 9.8, 4.0)) {
                STR += xbox1.LStickX() * placeholderName;
                FWD -= xbox1.LStickY() * placeholderName;
            }

        } else if (xbox1.buttonPad() == 180 && xbox1.RTrig() > 0.25) {
            if (!wallAlign(180.0, 130.0, 9.0, 4.0)) {
                STR += xbox1.LStickX() * placeholderName;
                FWD -= xbox1.LStickY() * placeholderName;
            }

        } else if (xbox1.buttonPad() == 225) {
            if (!wallAlign(205.0, 205.0, 13.3, 5.8)) {
                STR += xbox1.LStickX() * placeholderName;
                FWD -= xbox1.LStickY() * placeholderName;
            }

        } else if (xbox1.buttonPad() == 315) {
            if (!wallAlign(333.0, 333.0, 13.0, 5.8)) {
                STR += xbox1.LStickX() * placeholderName;
                FWD -= xbox1.LStickY() * placeholderName;
            }
        }

        if (xbox1.buttonPad() != -1) {
            Vector commands;
            commands = MathUtils.convertOrientation(headingRad, FWD, STR);
            FWD = commands.getY();
            STR = commands.getX();
        } else {
            wallAlignDone = false;
            wallAlignClose = false;
            useKeepAngle = true;
            prevWallAlignTime = Time.get();
            alignCase = 0;
        }

//        if (Math.abs(RCW) > 0.05 && (Math.abs(STR) + Math.abs(FWD) > 0.05)) { //Limit the acceleration for translating while rotating so that the wheels have time to get to the right spot
//            ACCEL_SPEED = 0.015; //If RCW in past second has been above 0.2, accel_Speed should be lowered for translation. Move RCW above this in that case
//            //If any of the wheels are off, give them time to align by starting slower
//        } else {
//            ACCEL_SPEED = 0.026;
//        }

//        System.out.println((SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getAngleOff() ||
//                SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getAngleOff() ||
//                SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getAngleOff() ||
//                SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getAngleOff()));

        //Optimize later, this should give wheels time to get to the right angle before commanding speed
//        if (compensateToggle && (Math.abs(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getSpeedCommand()) +
//                Math.abs(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getSpeedCommand()) +
//                Math.abs(SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getSpeedCommand()) +
//                Math.abs(SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getSpeedCommand()) < 1.0) &&  //If we're coming from 0 and wheels aren't aligned yet, stop wheels till they're aligned
//                (SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getAngleOff() ||
//                        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getAngleOff() ||
//                        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getAngleOff() ||
//                        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getAngleOff())) {
//            wheelStopCase = 0;
//        } else {
//            wheelStopCase = 1;
//        }

//        SmartDashboard.putBoolean("Toggle Disable Wheels", compensateToggle);
//        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).disableSpeed(wheelStopCase);
//        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).disableSpeed(wheelStopCase);
//        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).disableSpeed(wheelStopCase);
//        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).disableSpeed(wheelStopCase);

//        if (Math.abs(FWD) > Math.abs(prevFWD[cmdCounter])) {
//            if (Math.abs(FWD - prevFWD[cmdCounter]) > ACCEL_SPEED) {
//                if (FWD - prevFWD[cmdCounter] > 0.0) {
//                    FWD = prevFWD[cmdCounter] + ACCEL_SPEED;
//                } else {
//                    FWD = prevFWD[cmdCounter] - ACCEL_SPEED;
//                }
//            }
//        } else {
//            if (Math.abs(FWD - prevFWD[cmdCounter]) > DECEL_SPEED) {
//                if (FWD - prevFWD[cmdCounter] > 0.0) {
//                    FWD = prevFWD[cmdCounter] + DECEL_SPEED;
//                } else {
//                    FWD = prevFWD[cmdCounter] - DECEL_SPEED;
//                }
//            }
//        }

//        if (Math.abs(STR) > Math.abs(prevSTR[cmdCounter])) {
//            if (Math.abs(STR - prevSTR[cmdCounter]) > ACCEL_SPEED) {
//                if (STR - prevSTR[cmdCounter] > 0.0) {
//                    STR = prevSTR[cmdCounter] + ACCEL_SPEED;
//                } else {
//                    STR = prevSTR[cmdCounter] - ACCEL_SPEED;
//                }
//            }
//        } else {
//            if (Math.abs(STR - prevSTR[cmdCounter]) > DECEL_SPEED) {
//                if (STR - prevSTR[cmdCounter] > 0.0) {
//                    STR = prevSTR[cmdCounter] + DECEL_SPEED;
//                } else {
//                    STR = prevSTR[cmdCounter] - DECEL_SPEED;
//                }
//            }
//        }

//        if (FWD + STR != 0.0) {
//            if (Math.abs(RCW) > Math.abs(prevRCW[cmdCounter])) {
//                if (Math.abs(RCW - prevRCW[cmdCounter]) > ACCEL_SPEED) {
//                    if (RCW - prevRCW[cmdCounter] > 0.0) {
//                        RCW = prevRCW[cmdCounter] + ACCEL_SPEED;
//                    } else {
//                        RCW = prevRCW[cmdCounter] - ACCEL_SPEED;
//                    }
//                }
//            } else {
//                if (Math.abs(RCW - prevRCW[cmdCounter]) > DECEL_SPEED) {
//                    if (RCW - prevRCW[cmdCounter] > 0.0) {
//                        RCW = prevRCW[cmdCounter] + DECEL_SPEED;
//                    } else {
//                        RCW = prevRCW[cmdCounter] - DECEL_SPEED;
//                    }
//                }
//            }
//        }

        if (Math.abs(RCW) > 0.5) {
            RCW = Math.signum(RCW)*0.5;
//            prevRCW[cmdCounter] = 0.5;
        }

        //System.out.println(FWD + "|     |"  + STR + "|     |"  + prevFWD[cmdCounter] + "|     |"  +prevSTR[cmdCounter]);

//        if (Math.abs(RCW) > 0.3 || Math.abs(FWD) + Math.abs(STR) > 0.2) {
//            if (Time.get() - accelTimeChanger < 0.6) {
//                ACCEL_SPEED = 0.03;
//                DECEL_SPEED = 0.05;
//            } else {
////                System.out.println("Here");
//                ACCEL_SPEED= 0.05/*0.031*/;
//                DECEL_SPEED= 0.08; //0.035
//            }
//        } else {
//            accelTimeChanger = Time.get();
//        }
        acceleration();

//        prevRCW[cmdCounter] = RCW;
//        prevFWD[cmdCounter] = FWD;
//        prevSTR[cmdCounter] = STR;



//        Hatch.ground(xbox2.LB());

//        if (xbox2.B()) {
//            RRLogger.addData("Placing FRR", Lidar.getFRRightSensor());
//            RRLogger.addData("Placing FRF", Lidar.getFRFrontSensor());
//            RRLogger.addData("Placing FLF", Lidar.getFLFrontSensor());
//            RRLogger.addData("Placing FLL", Lidar.getFLLeftSensor());
//            RRLogger.newLine();
//            RRLogger.writeFromQueue();
//        }

//        if (xbox2.A()) {
//            RRLogger.addData("Grabbing FRR", Lidar.getFRRightSensor());
//            RRLogger.addData("Grabbing FRF", Lidar.getFRFrontSensor());
//            RRLogger.addData("Grabbing FLF", Lidar.getFLFrontSensor());
//            RRLogger.addData("Grabbing FLL", Lidar.getFLLeftSensor());
//            RRLogger.newLine();
//            RRLogger.writeFromQueue();
//        }

//        if (xbox2.Y()) {
//            RRLogger.addData("Cargo Distance 1", Lidar.getBLLeftSensor());
//            RRLogger.addData("Cargo Distance 2", Lidar.getBRRightSensor());
//            RRLogger.newLine();
//            RRLogger.writeFromQueue();
//        }
//        RRLogger.writeFromQueue();
        Cargo.set(false, xbox2.X(), xbox2.Y(), Lidar.hasCargo(), Lidar.getBLLeftSensor(), Lidar.getBRRightSensor());

        if (!Hatch.isRunning() && Elevator.getPosition() < 102.0 && Elevator.getPosition() > 1.0) {
            Elevator.setPosition(xbox2.LStickY(), xbox2.LTrig(), xbox2.LStickX(), xbox2.LStickButton(), xbox2.X(), xbox2.LB());
        }
//        else {
//            Elevator.setPower(-Math.signum(Elevator.getPosition() - 13.5) * 0.4);
//        }

        if (Elevator.atPosition()) {
            if (Hatch.set(xbox2.A(), xbox2.B(), false, safeToPutHatch) || xbox2.Y()) {
                FWD = 0.0;
                STR = 0.0;
                RCW = 0.0;
                Elevator.setPosition(0, 0, 0, true, false, false); //Maybe add a keepPosition boolean
            }
        }
//            else {
//                if (xbox2.LB()) {
//                    Elevator.setPosition(0.0, 0.0, 0.0, false, false, 0, true);
//                } else
//                }
//            }
//        } else {
//            System.out.println("Not here");

//            Hatch.set(false, false, false, false);
//            Hatch.set(xbox2.A(), xbox2.B(), xbox2.LB(), true);
//            Cargo.set(xbox2.Start(), xbox2.X(), xbox2.Y(), Lidar.hasCargo(), Lidar.getBLLeftSensor(), Lidar.getBRRightSensor());
//        }

//        if (xbox1.Back()) {
        if (!xbox1.Back() && prevBackButton) {
            compensateToggle = !compensateToggle;
//            SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setFoundFlag();
//            SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setFoundFlag();
//            SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setFoundFlag();
//            SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setFoundFlag();
        }
        prevBackButton = xbox1.Back();

//        System.out.println(speed);

//        rotateWhileMoving();

//        System.out.println(buttonTimeIncrease + "| |" + prevButtonTime + "| |" + (xbox1.buttonPad() == 90.0));


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

        //LTrig faces right side of cargoship align. RTrig faces left side
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

        if (wallAlignDone || cargoShipDone ||
                (atGoodDistance((Lidar.getFLFrontSensor() < Lidar.getFRFrontSensor()) ? Lidar.getFLFrontSensor(): Lidar.getFRFrontSensor(), lastTargetFrontDistance, wallAlignError) &&
                atGoodDistance((Lidar.getFRRightSensor() < Lidar.getFLLeftSensor()) ? Lidar.getFRRightSensor(): Lidar.getFLLeftSensor(), lastTargetSideDistance, wallAlignError) &&
                 Math.abs(MathUtils.calculateContinuousError(lastTargetAngle, imu.getAngle(), 360.0, 0.0)) < 2.8)) {
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
                if (HABAlign()) {
                    if (Time.get() - habTime > 0.2) {
                        doClimb = true;
//                    System.out.println("2");
                    }
                } else {
                    habTime = Time.get();
                }
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
//System.out.println("Final: " + RCW);

        driveTrain.drive(new Vector(-STR, FWD), RCW);



//        RRLogger.addData("Counter Encoder FR:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getCounterEncoder());
//        RRLogger.addData("Clockwise Encoder FR:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getClockwiseEncoder());
        SmartDashboard.putNumber("FR Angle", SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getAngle());
        SmartDashboard.putNumber("FL Angle", SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getAngle());
        SmartDashboard.putNumber("BL Angle ", SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getAngle());
        SmartDashboard.putNumber("BR Angle ", SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getAngle());

        SmartDashboard.putNumber("FR Angle Command", SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getAngleCommand());
        SmartDashboard.putNumber("FL Angle Command", SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getAngleCommand());
        SmartDashboard.putNumber("BL Angle Command", SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getAngleCommand());
        SmartDashboard.putNumber("BR Angle Command", SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getAngleCommand());



        RRLogger.addData("Angle of FR:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getAngle());
        RRLogger.addData("Angle Command of  FR:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getAngleCommand());

//        RRLogger.addData("Counter Encoder FL:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getCounterEncoder());
//        RRLogger.addData("Clockwise Encoder FL:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getClockwiseEncoder());
        RRLogger.addData("Angle of FL:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getAngle());
        RRLogger.addData("Angle Command of FL:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getAngleCommand());

//        RRLogger.addData("Counter Encoder BL:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getCounterEncoder());
//        RRLogger.addData("Clockwise Encoder BL:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getClockwiseEncoder());
        RRLogger.addData("Angle of BL:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getAngle());
        RRLogger.addData("Angle Command of  BL:  ", 	 SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getAngleCommand());

//        RRLogger.addData("Counter Encoder BR:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getCounterEncoder());
//        RRLogger.addData("Clockwise Encoder BR:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getClockwiseEncoder());
        RRLogger.addData("Angle of BR:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getAngle());
        RRLogger.addData("Angle Command of BR:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getAngleCommand());

        RRLogger.newLine();
        RRLogger.writeFromQueue();
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

        if (xbox1.Start()) {
            driveTrain.resetWheels();
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

        Lift.reset();

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
            RRLogger.close();
            SwerveDrivetrain.resetDeltas();
        }
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        // LABEL test
//        double speed = (xbox1.RStickX());
//
//        System.out.println(speed);
//
//        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setRotationCommand(speed);
//		SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setRotationCommand(speed);
//		SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setRotationCommand(speed);
//		SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setRotationCommand(speed);
    }
}
