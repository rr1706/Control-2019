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

    private final double ACCEL_SPEED= 0.05/*0.031*/;
    private final double DECEL_SPEED= 0.08; //0.035

    private SendableChooser<Integer> autoChooser;

    private Compressor compressor;

    public static XboxController xbox1 = new XboxController(0);
    public static XboxController xbox2 = new XboxController(1);

    //	private JetsonServer jet;
//	private Thread t;
    private SwerveDrivetrain driveTrain;
    private IMU imu;

//	private boolean robotBackwards;

    private double robotOffset;

    private int disabled = 0;
    private int wheelStopCase = 0;
    private double habTime = 0.0;

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
    private boolean override;
    private boolean driveDone;
    private boolean turnDone;
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

    private double lead;

    private boolean autonomousRunning = false;
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

    private boolean autoOverride = false;

    private double prevWallAlignTime = 0.0;


    //	private int arcResolution = 100;
//	private int[] arcLengths = new int[arcResolution+1];
    private double[][] arcPoints = new double[3][101];
    private int alignCase = 0;
    private int cargoCase = 0;

    private void acceleration() {
        if (FWD + STR != 0.0) {
            if (Math.abs(RCW) > Math.abs(prevRCW[cmdCounter])) {
                if (Math.abs(RCW - prevRCW[cmdCounter]) > ACCEL_SPEED) {
                    if (RCW - prevRCW[cmdCounter] > 0.0) {
                        RCW = prevRCW[cmdCounter] + ACCEL_SPEED;
                    } else {
                        RCW = prevRCW[cmdCounter] - ACCEL_SPEED;
                    }
                }
            } else {
                if (Math.abs(RCW - prevRCW[cmdCounter]) > DECEL_SPEED) {
                    if (RCW - prevRCW[cmdCounter] > 0.0) {
                        RCW = prevRCW[cmdCounter] + DECEL_SPEED;
                    } else {
                        RCW = prevRCW[cmdCounter] - DECEL_SPEED;
                    }
                }
            }
        }

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
    private boolean wallAlign(double angle,  double moveAngle, double sideDistance, double frontDistance) {
        double robotAngle = imu.getAngle();
        int done = 0;
        double[] distances = {Lidar.getFRRightSensor(), Lidar.getFRFrontSensor(), Lidar.getFLFrontSensor(), Lidar.getFLLeftSensor()};

        double side = (Lidar.getFRRightSensor() < Lidar.getFLLeftSensor()) ? Lidar.getFRRightSensor() : Lidar.getFLLeftSensor();
        double front = (Lidar.getFRFrontSensor() < Lidar.getFLFrontSensor()) ? Lidar.getFRFrontSensor() : Lidar.getFLFrontSensor();

//        System.out.println(alignCase);

        switch (alignCase) {
            case 0:
                if (!xbox1.LB() && Time.get() - prevWallAlignTime < 2.0 && (front < 15.0 || side < 15.0)) {
                    if (front < 16.0) {
                        FWD = -0.3/Math.cos(Math.toRadians(robotAngle));
                    }
                    if (side < 16.0) {
                        STR = -0.2*Math.signum(Lidar.getFLLeftSensor() - Lidar.getFRRightSensor()) / Math.cos(Math.toRadians(robotAngle)); //Should be robot oriented and based off side that is closer
                    }
                    RCW = 0.0;
                } else {
                    alignCase = 1;
                }
                break;
            case 1: //Start at case == 1 in auto

                useKeepAngle = false;
                FWD = 0.0;
                STR = 0.0;

                if (Math.abs(MathUtils.calculateContinuousError(angle, robotAngle, 360.0, 0.0)) >= 5.0) {
                    RCW = MathUtils.calculateContinuousError(angle, robotAngle, 360.0, 0.0) *3e-3; //0.016
                    System.out.println(RCW);
                    if (Math.abs(RCW) > 1.8) { //0.3
                        RCW = 1.8 * Math.signum(RCW); //0.3
                    } else if (Math.abs(RCW) < 0.3) {
                        RCW = 0.3 * Math.signum(RCW);
                    }
//                    System.out.println(RCW);

//                    keepAngle = angle;
                } else {
                    alignCase = 2;
                }

                break;
            case 2:

                if (Math.abs(MathUtils.calculateContinuousError(angle, robotAngle, 360.0, 0.0)) >= 2.8) {
                    RCW = MathUtils.calculateContinuousError(angle, robotAngle, 360.0, 0.0) *1.0e-3;
                    if (Math.abs(RCW) > 0.4) {
                        RCW = 0.4 * Math.signum(RCW);
                    } else if (Math.abs(RCW) < 0.1) {
                        RCW = 0.1 * Math.signum(RCW);
                    }
                } else {
                    keepAngle = angle;
                    useKeepAngle = true;
                    RCW = 0.0;
                    System.out.println("RCW Done");

                    done++;
                }

                if (robotAngle < angle-15.0 || robotAngle > angle + 15.0) {
                    side = (front < side) ? front: side;
                    front = MathUtils.instertionSort(distances)[2];
                }

                //        min dist to wall        max dist to wall
                System.out.println(side + "| |" + sideDistance);
                if ((side <= sideDistance-1.1 || side >= sideDistance+1.1)) { //Decrease allowed error as much as possible. Log lidar values in comp. to get experimental mean and standard deviation
                    //                                                                                     setpoint                     P
                    STR = Math.sin(Math.toRadians(moveAngle)) * (side - sideDistance) * 0.0325;
//            System.out.println("STR BAD");
                } else {
                    System.out.println("STR Done");
                    STR = 0.0;
                    done++;
                }
                // max speed
                if (Math.abs(STR) > Math.abs(Math.sin(Math.toRadians(moveAngle))  * 0.4)) {
                    STR = Math.sin(Math.toRadians(moveAngle)) * Math.signum(side-sideDistance) * (0.2 + sideDistance/50.0); //Scale speed to distance from wall
                }

//                if (side < 40.0) {
//                    STR *= 0.8;
//                }

                if (autonomousRunning && side < 20.0) {
                    STR *= 0.6;
                }

                if  (Math.abs(STR) < 0.05) {
                    STR = Math.signum(STR)*0.05;
                }

                //  min dist to wall                            max dist to wall
                if ((front <= frontDistance-1.5 || front >= frontDistance+2.2)) {
                    //                                                                                              setpoint                  P
                    FWD = Math.cos(Math.toRadians(moveAngle)) * (front - frontDistance) * 0.025;
//                    System.out.println("FWD: " + FWD);

//            System.out.println("FWD BAD");
                    // max speed
                    if (Math.abs(FWD) > Math.abs(Math.sin(Math.toRadians(moveAngle))  * 0.4)) {
                        FWD = Math.cos(Math.toRadians(moveAngle)) * Math.signum(front-frontDistance) * (0.7 + frontDistance/15.0);
                    }
//                    if (front < 40) {
//                        FWD *= 0.8;
//                    }

                    if (autonomousRunning && front < 20.0) {
                        FWD *= 0.6;
                    }
                    // Min speed
                    if  (Math.abs(FWD) < 0.1) {
                        FWD = Math.signum(FWD)*0.1;
                    }
                } else {
                    System.out.println("FWD Done");
                    FWD = 0.0;
                    done++;
                }

                break;
        }


        if (done == 3) {
            xbox2.rumbleLeft(1.0);
        } else {
            xbox2.stopLeftRumble();
        }

        return (done == 3);
    }

    private boolean HABAlign(){
        boolean rotateDone = false;
        boolean fwdDone = false;
        double robotAngle = imu.getAngle();
        double front = (Lidar.getFRFrontSensor() < Lidar.getFLFrontSensor() ? Lidar.getFRFrontSensor(): Lidar.getFLFrontSensor());

        if(Math.abs(MathUtils.calculateContinuousError(180.0, robotAngle, 360.0, 0.0)) >= 2.8){
            RCW = MathUtils.calculateContinuousError(180.0, robotAngle, 360.0, 0.0) * 0.011;

            if(Math.abs(RCW) > 0.16){
                RCW = Math.signum(RCW) * 0.16;
            }

            keepAngle = 180.0;

        } else{
            RCW = 0.0;
            rotateDone = true;
        }

        if (rotateDone && Time.get() - habLidarTimeout > 3.0) {
            fwdDone = true;
        } else {
            habLidarTimeout = Time.get();
        }

        if (front > 7.75){
            FWD = -0.15;
        } else if(front < 7.25){
            FWD = 0.15;
        } else {
            fwdDone = true;
        }
        System.out.println(fwdDone + " " + rotateDone);

        return  (fwdDone && rotateDone);
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
                if (frontLeft < 10.0 || frontRight < 10.0) {
                    cargoCase = 2;
                } else {
                    FWD = -0.4; //Scale to distance from
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

                if (frontLeft < 11.0 && frontRight < 11.0) {
                    done++;
                } else if (frontLeft >= 11.0) { //If only BL edge is off cargo, move robot right
                    STR = 0.08 + (frontLeft-frontRight) * 0.004;
                } else if (frontRight >= 11.0) {
                    STR = -0.08 - (frontRight-frontLeft) * 0.004;
                }

                if (frontLeft > 10.0 && frontRight > 10.0) {
                    FWD = -0.4;
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

            SmartDashboard.putNumber("Robot Rotation 1", robotRotation);
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
        // LABEL robot init
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

        autoChooser = new SendableChooser<>();
        autoChooser.addOption("Left", 1);
        autoChooser.addOption("Right", 2);
//        autoChooser.addOption("Forward", 3);
        SmartDashboard.putData("Autonomous Mode Chooser", autoChooser);

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

    public void autonomousInit() {
//        autonomousRunning = true;
        compensateToggle = true;
        // LABEL autonomous init
//		jet.setAuto(); // this line is important because it does clock synchronization

        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).disableSpeed(2);
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).disableSpeed(2);
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).disableSpeed(2);
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).disableSpeed(2);

        timeCheck = true;
        imu.reset(0);
        setpoint = 1;

        String choice;

        choice = "/home/lvuser/deploy/RightHatch.csv";

        SmartDashboard.putString("Autonomous File", choice);

        imu.reset(0);
        arrayIndex = 0;
        initialAngle = imu.getAngle();
        turnDone = false;
        driveDone = false;
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
                     * 17 = use wall align (0 = none, 1 = use)
                     * 18 = side distance for wall align set-point
                     * 19 = front distance for wall align set-point
                     */

                    //Only use translation for RCW and Arc Speed
                    tSpeed = commands[arrayIndex][0];
                    rSpeed = commands[arrayIndex][1];

                    ArcXs[0] = commands[arrayIndex][6];
                    ArcXs[1] = commands[arrayIndex][7];
                    ArcXs[2] = commands[arrayIndex][8];

                    ArcYs[0] = commands[arrayIndex][9];
                    ArcYs[1] = commands[arrayIndex][10];
                    ArcYs[2] = commands[arrayIndex][11];


//				System.out.println(Hatch.get());

                    //Place hatch
//                    if (commands[arrayIndex][15] != 0.0) {
                        if (commands[arrayIndex][15] == 1.0 && elevatorCase == 1 && Hatch.hasHatch() && Hatch.set(false, true, false, true)) { //If we want to put the hatch and the elevator is in position and still has the hatch
                            hatchDone = false;
                        } else if (commands[arrayIndex][15] == 2.0 && elevatorCase == 1 && !Hatch.hasHatch() && Hatch.set(true, false, false, true)) { //If we want to get the hatch and we don't have one
                            hatchDone = false;
                        } else {
                        Hatch.set(false, false, false, true);
                        hatchDone = true;
                }


                    if (commands[arrayIndex][16] == 1.0 && elevatorCase == 1) { //If we want to push cargo and the elevator is in position
                        Cargo.set(false, false, true, Lidar.hasCargo(), Lidar.getBLLeftSensor(), Lidar.getBRRightSensor());
                        cargoDone = Lidar.hasCargo() > 30.0;
                    } else if (commands[arrayIndex][16] == 2.0 && elevatorCase == 1) {
                        Cargo.set(true, false, false, Lidar.hasCargo(), Lidar.getBLLeftSensor(), Lidar.getBRRightSensor());
                        cargoDone = Lidar.hasCargo() < 5.0;
                    } else {
                        cargoDone = true;
                    }

                    if (commands[arrayIndex][14] != 0.0) {
                        switch (elevatorCase) {
                            case 0: //Going up
                                setpoint = (int) commands[arrayIndex][14];
                                if (setpoint > 3) { //Moving for cargo
                                    Elevator.setPosition(0.0, 1.0, 0.0, false, false, setpoint - 3, false);
                                    cargoDone = false;
                                } else { //Moving for hatch
                                    Elevator.setPosition(0.0, 0.0, 0.0, false, false, setpoint, false);
                                    hatchDone = false;
                                }
                                if (Elevator.atPosition()) {
                                    elevatorCase = 1;
                                }
                                break;
                            case 1: //Done placing
                                if (hatchDone && cargoDone) {
                                    Elevator.setPosition(0.0, 0.0, 0.0, false, false, 1, false);
                                    elevatorCase = 2;
                                }
                                break;
                            case 2: //Done going down
                                if (Elevator.atPosition()) {
                                    override = true;
                                    elevatorCase = 0;
                                }
                                break;
                        }
                    } else {
                        Elevator.setPosition(0.0, 0.0, 0.0, false, false, 1, false);
                    }

//                setpoint = (int) commands[arrayIndex][14];

                    if (commands[arrayIndex][2] != -1) {
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
                        curveVelocity = calculatePath(SmartDashboard.getNumber("Distance", 0) - previousDistance);
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
                        smoothAccelerateNum = (MathUtils.convertRange(previousDistance, previousDistance + commands[arrayIndex][4], commands[arrayIndex][0], commands[arrayIndex + 1][0], SmartDashboard.getNumber("Distance", 0)));
                        smoothAccelerate = smoothAccelerateNum;
                        FWD *= smoothAccelerate;
                        STR *= smoothAccelerate;
                    } else if (commands[arrayIndex][5] == 2.0) {
                        smoothAccelerateNum = (MathUtils.convertRange(previousDistance, previousDistance + commands[arrayIndex][4], minSpeed, commands[arrayIndex][0], SmartDashboard.getNumber("Distance", 0)));
                        smoothAccelerate = smoothAccelerateNum;
                        FWD *= smoothAccelerate;
                        STR *= smoothAccelerate;
                    } else if (commands[arrayIndex][5] == 3.0) {
                        smoothAccelerateNum = (MathUtils.convertRange(previousDistance, previousDistance + commands[arrayIndex][4], commands[arrayIndex][0], minSpeed, SmartDashboard.getNumber("Distance", 0)));
                        smoothAccelerate = smoothAccelerateNum;
                        FWD *= smoothAccelerate;
                        STR *= smoothAccelerate;
                    } else {
                        FWD *= tSpeed;
                        STR *= tSpeed;
                    }


                    SmartDashboard.putNumber("Previous Distance", previousDistance);

                    if ((Math.abs(SmartDashboard.getNumber("Distance", 0) - previousDistance) >= commands[arrayIndex][4])) {
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

                    SmartDashboard.putNumber("FWD", FWD);
                    SmartDashboard.putNumber("STR", STR);
                    SmartDashboard.putNumber("RCW", RCW);

//				if (robotBackwards) {
//					driveTrain.drive(new Vector(-STR, FWD), RCW);
//				} else {

                    if (commands[arrayIndex][17] != 0.0) {
                        if (commands[arrayIndex][17] == 2.0 && alignCase == 0) {
                            autonomousRunning = true;
                            alignCase = 1;
                        } else {
                            autonomousRunning = false;
                        }

                        if (wallAlign(commands[arrayIndex][3], commands[arrayIndex][2], commands[arrayIndex][18], commands[arrayIndex][19])) {
                            override = true;
                        }

                        FWD *= commands[arrayIndex][0];
                        STR *= commands[arrayIndex][0];
                        RCW *= commands[arrayIndex][1];
                    }

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

//                    if (Math.abs(FWD) > Math.abs(prevFWD[cmdCounter])) {
//                        if (Math.abs(FWD - prevFWD[cmdCounter]) > ACCEL_SPEED) {
//                            if (FWD - prevFWD[cmdCounter] > 0.0) {
//                                FWD = prevFWD[cmdCounter] + ACCEL_SPEED;
//                            } else {
//                                FWD = prevFWD[cmdCounter] - ACCEL_SPEED;
//                            }
//                        }
//                    } else {
//                        if (Math.abs(FWD - prevFWD[cmdCounter]) > DECEL_SPEED) {
//                            if (FWD - prevFWD[cmdCounter] > 0.0) {
//                                FWD = prevFWD[cmdCounter] + DECEL_SPEED;
//                            } else {
//                                FWD = prevFWD[cmdCounter] - DECEL_SPEED;
//                            }
//                        }
//                    }
//
//                    if (Math.abs(STR) > Math.abs(prevSTR[cmdCounter])) {
//                        if (Math.abs(STR - prevSTR[cmdCounter]) > ACCEL_SPEED) {
//                            if (STR - prevSTR[cmdCounter] > 0.0) {
//                                STR = prevSTR[cmdCounter] + ACCEL_SPEED;
//                            } else {
//                                STR = prevSTR[cmdCounter] - ACCEL_SPEED;
//                            }
//                        }
//                    } else {
//                        if (Math.abs(STR - prevSTR[cmdCounter]) > DECEL_SPEED) {
//                            if (STR - prevSTR[cmdCounter] > 0.0) {
//                                STR = prevSTR[cmdCounter] + DECEL_SPEED;
//                            } else {
//                                STR = prevSTR[cmdCounter] - DECEL_SPEED;
//                            }
//                        }
//                    }

//                    prevFWD[cmdCounter] = FWD;
//                    prevSTR[cmdCounter] = STR;
                    acceleration();

                    Vector driveCommands;
                    driveCommands = MathUtils.convertOrientation(Math.toRadians(imu.getAngle()), FWD, STR);
                    FWD = driveCommands.getY();
                    STR = driveCommands.getX();
                    RCW *= rSpeed;

//                    System.out.println(driveDone + "  " + autoOverride + "  " + FWD + "  " + STR);

                    driveTrain.drive(new Vector(-STR, FWD), RCW);

                    if (override) {
                        driveDone = true;
                        turnDone = true;
                        hatchDone = true;
                        cargoDone = true;
                    }

//				System.out.println("Drive: " + driveDone);
//				System.out.println("Turn: " + turnDone);
//				System.out.println("Coll: " + collisionDone);
//				System.out.println("Time: " + timeDone);
//				System.out.println("TimeNum: " + Time.get() + " | " + (timeBase + commands[arrayIndex][10]));

                    SmartDashboard.putNumber("Array", arrayIndex);

				System.out.println(driveDone + "||" + hatchDone + "||" + cargoDone);

                    if (driveDone && hatchDone && cargoDone) {
//                    robotAligned = false;
                        arcCalculated = false;
                        arrayIndex++;
                        driveDone = false;
                        initialAngle = imu.getAngle();
                        previousDistance = SmartDashboard.getNumber("Distance", 0);//currentDistance;
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
        autonomousRunning = false;
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

//		RRLogger.start();
        Elevator.init();
        Elevator.setPosition(1.0, 0, 0, false, false, 0, false);
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
//        System.out.println(FWD + "| |" + STR + "| |" + RCW);
        // LABEL teleop periodic
        autonomous = false;

        System.out.println(xbox2.RStickY());
        habStart = xbox2.Start();

        SmartDashboard.putNumber("Lift Encoder", Lift.getDistance());
        if (habStart) {
            System.out.println("1");
            if (HABAlign()) {
                if (Time.get()-habTime > 1.0) {
                    doClimb = true;
                    System.out.println("2");
                }
            } else {
                habTime = Time.get();
            }
        }
        if (doClimb) {
            System.out.println("3");
            Lift.climb(xbox2.RStickY());
            if (xbox2.Back()) {
                Lift.setManual();
            }
        }

        SmartDashboard.putNumber("IMU Angle", imu.getAngle());
        SmartDashboard.putNumber("Elevator Setpoint", position);
        SmartDashboard.putNumber("L Trig", xbox2.LTrig());
        SmartDashboard.putNumber("R Trig", xbox2.RTrig());


        xbox1.setDeadband(0.2);
        xbox2.setDeadband(0.2);

        SmartDashboard.putNumber("DPad", xbox2.DPad());

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
            xbox2.rumbleRight(0.25);
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
            RCW = xbox1.RStickX() * 0.2;
        }

        if (!xbox1.LStickButton()) {
            FWD = -xbox1.LStickY();
            STR = xbox1.LStickX();
        } else {
            FWD = -xbox1.LStickY() * 1.9;
            STR = xbox1.LStickX() * 1.9;
        }

        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).turbo(xbox1.LStickButton());
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).turbo(xbox1.LStickButton());
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).turbo(xbox1.LStickButton());
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).turbo(xbox1.LStickButton());

        SmartDashboard.putNumber("PrevFWD", prevFWD[cmdCounter]);
        SmartDashboard.putNumber("Counter", cmdCounter);

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


        if (xbox1.buttonPad() != -1) {
            FWD = 0.0;
            STR = 0.0;
            RCW = 0.0;
        }

        if (xbox1.buttonPad() == 45) {
            wallAlign(25.0, 45.0, 13.5, 5.4);
            STR += xbox1.LStickX() * placeholderName;
            FWD -= xbox1.LStickY() * placeholderName;

        } else if (xbox1.buttonPad() == 135) {
            wallAlign(152.0, 132.0, 13.5, 5.4);
            STR += xbox1.LStickX() * placeholderName;
            FWD -= xbox1.LStickY() * placeholderName;

        } else if (xbox1.buttonPad() == 180 && xbox1.LTrig() > 0.25) {
            wallAlign(180.0,  225.0, 12.0, 2.5);
//            STR += xbox1.LStickX() * placeholderName;
            FWD -= xbox1.LStickY() * placeholderName;

        } else if (xbox1.buttonPad() == 180 && xbox1.RTrig() > 0.25) {
            wallAlign(180.0, 135.0, 12.0, 2.5);
//            STR += xbox1.LStickX() * placeholderName;
            FWD -= xbox1.LStickY() * placeholderName;

        } else if (xbox1.buttonPad() == 225) {
            wallAlign(216.0, 216.0, 14.5, 5.2);
//            STR += xbox1.LStickX() * placeholderName;
            FWD -= xbox1.LStickY() * placeholderName;

        } else if (xbox1.buttonPad() == 315) {
            wallAlign(333.0, 333.0, 14.5, 5.2);
//            STR += xbox1.LStickX() * placeholderName;
            FWD -= xbox1.LStickY() * placeholderName;
        }


        if (xbox1.buttonPad() != -1) {
            Vector commands;
            commands = MathUtils.convertOrientation(headingRad, FWD, STR);
            FWD = commands.getY();
            STR = commands.getX();
        } else {
            useKeepAngle = true;
            prevWallAlignTime = Time.get();
            alignCase = 0;
            xbox2.stopLeftRumble();
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
        if (compensateToggle && (Math.abs(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getSpeedCommand()) +
                Math.abs(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getSpeedCommand()) +
                Math.abs(SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getSpeedCommand()) +
                Math.abs(SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getSpeedCommand()) < 0.1) &&  //If we're coming from 0 and wheels aren't aligned yet, stop wheels till they're aligned
                (SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getAngleOff() ||
                        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getAngleOff() ||
                        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getAngleOff() ||
                        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getAngleOff())) {
            wheelStopCase = 0;
            //FIXME, current auto: add once lidar works
            //0.0,0.0,0.0,0.0,999.0,0.0,999.0,0.0,0.0,0.0,0.0,0.0,0.01,0.0,0.0,0.0,0.0,0.0,0.0,0.0
            //0.4,0.0,87.0,0.0,50.0,0,999,0,0,0,0,0,0.0,0,0,0,0,0,0,0
            //0.8,0.1,8.0,25.0,105.0,0,999,0,0,0,0,0,0,0,0,0,0,0,0,0
            //1.1,0.8,30.0,25.0,999.0,0,999,0,0,0,0,0,0.0,0,0,0,0,1.0,13.2,5.2
            //0.0,0.0,0.0,25.0,999.0,0.0,999.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,1.0,0.0,0.0,0.0,0.0
            //0.8,0.0,180.0,180.0,140.0,0,999,0,0,0,0,0,0,0.0,0,0,0,0,0,0
            //0.0,0.3,180.0,180.0,0.0,0,999,0,0,0,0,0,0,0.0,0,0,0,0,0,0
            //1.1,0.9,135.0,180.0,999.0,0,999,0,0,0,0,0,0.0,180.0,0,0,0,1.0,11.0,2.5
            //0.0,0.0,0.0,180.0,999.0,0.0,999.0,0.0,0.0,0.0,0.0,0.0,0.0,180.0,1.0,2.0,0.0,0.0,0.0,0.0
            //0.6,0.2,358,25.0,25.0,0,999,0,0,0,0,0,0,0,0,0,0,0,0,0
            //0.6,0.25,20.0,25.0,120.0,0,999,0,0,0,0,0,0,0,0,0,0,0,0,0
            //0.6,0.6,35.0,25.0,999.0,0,999,0,0,0,0,0,0.0,0,0,0,0,1.0,14.0,5.4
            //0.0,0.0,0.0,25.0,999.0,0.0,999.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,2.0,1.0,0.0,0.0,0.0,0.0
            //0,0,0,0,999,0,999,0,0,0,0,0,0,0.0,0,0,0,0,0,0
        } else if (compensateToggle && ((Math.abs(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getSpeedCommand()) +
                Math.abs(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getSpeedCommand()) +
                Math.abs(SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getSpeedCommand()) +
                Math.abs(SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getSpeedCommand()) > 0.1) ||
                (Math.abs(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getRotationCommand()) +
                        Math.abs(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getRotationCommand()) +
                        Math.abs(SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getRotationCommand()) +
                        Math.abs(SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getRotationCommand()) > 0.05)) &&  //If the robot is moving and it can't get there, decelerate to stop
                (SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getAngleOff() ||
                        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getAngleOff() ||
                        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getAngleOff() ||
                        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getAngleOff())) {
            wheelStopCase = 0;
        } else {
            wheelStopCase = 1;
        }

        SmartDashboard.putBoolean("Toggle Disable Wheels", compensateToggle);
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).disableSpeed(wheelStopCase);
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).disableSpeed(wheelStopCase);
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).disableSpeed(wheelStopCase);
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).disableSpeed(wheelStopCase);

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

        acceleration();

//        prevRCW[cmdCounter] = RCW;
//        prevFWD[cmdCounter] = FWD;
//        prevSTR[cmdCounter] = STR;



        LineSensor.get();

//        Hatch.ground(xbox2.LB());


//        if (xbox2.B()) {
//            RRLogger.addData("Placing Hatch", (Lidar.getFRRightSensor() + "| |" + Lidar.getFRFrontSensor() + Lidar.getFLFrontSensor() + "| |" + Lidar.getFLLeftSensor()));
//            RRLogger.newLine();
//        } else if (xbox2.A()) {
//            RRLogger.addData("Grabbing Hatch", (Lidar.getFRRightSensor() + "| |" + Lidar.getFRFrontSensor() + Lidar.getFLFrontSensor() + "| |" + Lidar.getFLLeftSensor()));
//            RRLogger.newLine();
//        }

        if (Elevator.atPosition()) {
            Cargo.set(false, xbox2.X(), xbox2.Y(), Lidar.hasCargo(), Lidar.getBLLeftSensor(), Lidar.getBRRightSensor());

//            System.out.println(xbox2.B());
            if (Hatch.set(xbox2.A(), xbox2.B(), false, safeToPutHatch) || xbox2.Y()) {
                FWD = 0.0;
                STR = 0.0;
                RCW = 0.0;
                Elevator.setPosition(0, 0, 0, true, false, 0, false);

            }
//            else {
//                if (xbox2.LB()) {
//                    Elevator.setPosition(0.0, 0.0, 0.0, false, false, 0, true);
//                } else {
//                    if (Elevator.getPosition() < 105.0 && Elevator.getPosition() > 1.0) {
//                        if (xbox2.X()) {
//                            Elevator.setPosition(xbox2.LStickY(), xbox2.LTrig(), xbox2.LStickX(), xbox2.LStickButton(), true, 0, false);
//                        } else {
//                            Elevator.setPosition(xbox2.LStickY(), xbox2.LTrig(), xbox2.LStickX(), xbox2.LStickButton(), false, 0, false);
//                        }
//                    } else {
//                        Elevator.setPower(-Math.signum(Elevator.getPosition() - 13.5) * 0.4);
//                    }
//                }
//            }
        } else {
//            System.out.println("Not here");

//            Hatch.set(false, false, false, false);
//            Hatch.set(xbox2.A(), xbox2.B(), xbox2.LB(), true);
//            Cargo.set(xbox2.Start(), xbox2.X(), xbox2.Y(), Lidar.hasCargo(), Lidar.getBLLeftSensor(), Lidar.getBRRightSensor());
          Cargo.set(false, false, false, 0.0, 0.0, 0.0);
        }

//        if (xbox1.Back()) {
            driveTrain.resetAngle(xbox1.Back());
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

//        if ((Math.abs(RCW) > 0.1/(Math.abs(FWD) +Math.abs(STR))) && (Math.abs(FWD) + Math.abs(STR) != 0.0)) {
//            RCW *= 0.1/(Math.abs(FWD) + Math.abs(STR));
//        }

        if ((Math.abs(RCW) > 0.2 && (Math.abs(FWD) + Math.abs(STR) > 0.1))) {
            RCW *= 0.2;
        }

        keepAngle();
        SmartDashboard.putBoolean("Wheels Are Off", wheelsGood);


        SmartDashboard.putNumber("FWD", FWD);
        SmartDashboard.putNumber("STR", STR);
        SmartDashboard.putNumber("RCW", RCW);
        SmartDashboard.putNumber("IMU Angle", imu.getAngle());

        driveTrain.drive(new Vector(-STR, FWD), RCW);



//        RRLogger.addData("Counter Encoder FR:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getCounterEncoder());
//        RRLogger.addData("Clockwise Encoder FR:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getClockwiseEncoder());
//        RRLogger.addData("Angle Command of  FR:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getAngleCommand());
//
//        RRLogger.addData("Counter Encoder FL:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getCounterEncoder());
//        RRLogger.addData("Clockwise Encoder FL:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getClockwiseEncoder());
//        RRLogger.addData("Angle Command of FL:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getAngleCommand());
//
//        RRLogger.addData("Counter Encoder BL:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getCounterEncoder());
//        RRLogger.addData("Clockwise Encoder BL:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getClockwiseEncoder());
//        RRLogger.addData("Angle Command of  BL:  ", 	 SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getAngleCommand());
//
//        RRLogger.addData("Counter Encoder BR:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getCounterEncoder());
//        RRLogger.addData("Clockwise Encoder BR:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getClockwiseEncoder());
//        RRLogger.addData("Angle Command of BR:  ", 	SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getAngleCommand());
//
//        RRLogger.newLine();
//        RRLogger.writeFromQueue();
    }

    public void robotPeriodic() {
        currentDistance += SwerveDrivetrain.getRobotDistance();
//		System.out.println("Robot Dist: " + SwerveDrivetrain.getRobotDistance());
        SmartDashboard.putNumber("Distance", currentDistance);

        if (xbox1.Start()) {
            driveTrain.resetWheels();
        }

        Lidar.read();
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
//            RRLogger.writeFromQueue();
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
