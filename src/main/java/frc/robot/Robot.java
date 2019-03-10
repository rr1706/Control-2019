package frc.robot;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.channels.CancelledKeyException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;
import java.util.Properties;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;

//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SwerveDrivetrain.WheelType;
import frc.robot.utilities.*;
import frc.robot.RRLogger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.opencv.core.Mat;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update the manifest file in the
 * resource directory.
 */
public class Robot extends TimedRobot {

    private final double ACCEL_SPEED= 0.03/*0.045*/;
    private final double DECEL_SPEED= 0.03;

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

    private double[][] commands;
    private int arrayIndex = -1;
    private int autoMove = 0;
    private int translateType;
    private double autonomousAngle;
    private double tSpeed;
    private double rSpeed;
    private double previousDistance = 0.0;
    private double currentDistance = 0.0;
    private boolean override;
    private boolean driveDone;
    private boolean turnDone;
    private boolean hatchDone;
    private boolean cargoDone;
    private boolean timeDone;
    private double offsetDeg;
    private double prevOffset = 0;
    private double timeBase;
    private boolean timeCheck;
    private double smoothArc;
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

    private double wheelRamp = 0;
    private double rampRate = 0;
    private double currentRampTime = 0;
    private double prevRampTime = 0;

    private double keepAngle;

    private boolean autonomous;

    private boolean fieldOriented = true; // start on field orientation
    private boolean previousOrientedButton = false;
    private boolean currentOrientedButton = false;

    private PIDController SwerveCompensate;

    private double lead;

    private double robotRotation;

    private double imuOffset = 0;

    private Properties application = new Properties();
    private File offsets = new File("/home/lvuser/deploy/SWERVE_OFFSET.txt");

    private boolean rumble = false;
    private int rumbleTime = 0;

    private Acceleration accel;
    private Acceleration rcwAccel;

    private Acceleration decelFWD;
    private Acceleration decelSTR;
    private Acceleration decelRCW;
    private int cmdCounter = 0;
    private boolean decel = false;
    private int accelState = -1;
    private int gameState = 0;


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

    private double placeholderName = 0.45;
    private boolean robotAligned = false;
    private boolean elevatorDescending = false;
    private boolean toggleCompresor = false;
    private boolean prevRB = false;
    private int elevatorCase = 0;



    //	private int arcResolution = 100;
//	private int[] arcLengths = new int[arcResolution+1];
    private double[][] arcPoints = new double[3][101];

    private boolean wallAlign(double angle,  double moveAngle, double sideDistance, double frontDistance) {
        int done = 0;
        double side = (Lidar.getFRRightSensor() < Lidar.getFLLeftSensor()) ? Lidar.getFRRightSensor() : Lidar.getFLLeftSensor();
        double front = (Lidar.getFRFrontSensor() < Lidar.getFLFrontSensor()) ? Lidar.getFRFrontSensor() : Lidar.getFLFrontSensor();

        if (Math.abs(MathUtils.calculateContinuousError(angle, imu.getAngle(), 360.0, 0.0)) >= 2.8) {
            RCW = MathUtils.calculateContinuousError(angle, imu.getAngle(), 360.0, 0.0) *0.011;
            if (RCW > 0.16) { //0.16
                RCW = 0.16;
            }
            System.out.println("RCW BAD");
            keepAngle = angle;
        } else {
            RCW = 0.0;
            done++;
        }

        //        min dist to wall        max dist to wall
        if (side <= sideDistance-1.1 || side >= sideDistance+1.1) {
            //                                                                                     setpoint                     P
            STR = Math.sin(Math.toRadians(moveAngle)) * (side - sideDistance) * 0.0325;
            System.out.println("STR BAD");
        } else {
            done++;
        }
        // max speed
        if (Math.abs(STR) > Math.abs(Math.sin(Math.toRadians(moveAngle))  * 0.4)) {
            STR = Math.sin(Math.toRadians(moveAngle)) * Math.signum(side - sideDistance) * 0.4;
        }

        //  min dist to wall                            max dist to wall
        if (front <= frontDistance-1.8 || front >= frontDistance+3.1) {
            //                                                                                              setpoint                  P
            FWD = Math.cos(Math.toRadians(moveAngle)) * (front - frontDistance) * 0.025;
            System.out.println("FWD BAD");
            // max speed
            if (Math.abs(FWD) > Math.abs(Math.sin(Math.toRadians(moveAngle))  * 0.4)) {
                FWD = Math.cos(Math.toRadians(moveAngle)) * Math.signum(front - frontDistance) * 0.4;
            }
            // Min speed
            if  (Math.abs(FWD) < 0.04) {
                FWD = Math.signum(FWD)*0.04;
            }
        } else {
            done++;
        }

        System.out.println("-----------");

        if (done == 3) {
            xbox2.rumbleLeft(1.0);
        } else {
            xbox2.stopLeftRumble();
        }

        return (done == 3);

//        if (RCW == 0.0 && Math.abs(STR) < 0.05 && Math.abs(FWD) < 0.05) {
//            robotAligned = true;
//        } else {
//            robotAligned = false;
//        }
    }

    private void keepAngle() {
        //This causes the point rotation when wheels are flipped over y=x
        // LABEL keepAngle
        SwerveCompensate.enable();

        double leadNum = SmartDashboard.getNumber("leadNum", 0);
        lead = RCW * leadNum;


        // This will update the angle to keep the robot's orientation
        if (Math.abs(xbox1.RStickX()) > 0.05 || // If right stick is pressed
                (Math.abs(FWD) < 0.05 && Math.abs(STR) < 0.05) && // If left stick is not pressed
                        (xbox1.buttonPad() == -1) && // If dpad is not pressed
                        (!autonomous)) { // If teleop

            SwerveCompensate.setPID(0.015/*0.015*/, 0.0, 0.0);
            keepAngle = imu.getAngle();
        } else {
            SwerveCompensate.setPID(0.008/*0.008*/, 0.0, 0.0);

            SwerveCompensate.setInput(imu.getAngle());
            SwerveCompensate.setSetpoint(keepAngle);

            if (!SwerveCompensate.onTarget()) {
                SwerveCompensate.setPID(0.007/*0.005*/, SmartDashboard.getNumber("CompensateI", 0.0), SmartDashboard.getNumber("CompensateD", 0.0));
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

//		SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDrift(application.getProperty("front_right_drift", "0.0,0.0").split(","));
//		SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDrift(application.getProperty("front_left_drift", "0.0,0.0").split(","));
//		SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDrift(application.getProperty("back_left_drift", "0.0,0.0").split(","));
//		SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDrift(application.getProperty("back_right_drift", "0.0,0.0").split(","));

//		robotBackwards = Boolean.parseBoolean(application.getProperty("robot_backwards", "false"));
    }

    private void autonomousAngle(double angle) {
        // LABEL autonomous angle

        SwerveCompensate.setInput(imu.getAngle());
        SwerveCompensate.setSetpoint(angle);

        SwerveCompensate.setTolerance(7);
        if (!SwerveCompensate.onTarget()) {
            SwerveCompensate.setPID(0.013, SmartDashboard.getNumber("CompensateI", 0.0), SmartDashboard.getNumber("CompensateD", 0.0));
        } else {
            SwerveCompensate.setPID(0.015, SmartDashboard.getNumber("CompensateI", 0.0), SmartDashboard.getNumber("CompensateD", 0.0));
        }

        robotRotation = SwerveCompensate.performPID();

        RCW = (robotRotation);
    }

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
        // LABEL autonomous init
//		jet.setAuto(); // this line is important because it does clock synchronization

        timeCheck = true;
        imu.reset(0);
        setpoint = 1;

        String choice;

        choice = "/home/lvuser/deploy/Test.csv";

        SmartDashboard.putString("Autonomous File", choice);

        imu.reset(0);
        arrayIndex = 0;
        initialAngle = imu.getAngle();
        turnDone = false;
        driveDone = false;
        hatchDone = false;
        cargoDone = false;

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
            System.err.println("Error in configuration!");
        }
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
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



				System.out.println(Hatch.get());
				//Place hatch
                if (commands[arrayIndex][15] == 1.0 && elevatorCase == 1) { //If we want to put the hatch and the elevator is in position
                    Hatch.set(false, true, false); //Move on to next command once finished with intaking. Cargo will have a sensor
                    hatchDone = !Hatch.get();
                } else if (commands[arrayIndex][15] == 2.0 && elevatorCase == 1) {
                    Hatch.set(true, false, false);
                    hatchDone = Hatch.get();
                } else {
                    hatchDone = true;
                }


                if (commands[arrayIndex][16] == 1.0 && elevatorCase == 1) { //If we want to put the hatch and the elevator is in position
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
                                Elevator.setPosition(0.0, 1.0, 0.0, false, false, setpoint-3, false);
                                cargoDone = false;
                            } else { //Moving for ball
                                Elevator.setPosition(0.0, 0.0, 0.0, false, false, setpoint, false);
                                hatchDone = false;
                            }
                            if (Elevator.atPosition()) {
                                elevatorCase = 1;
                            }
                            break;
                        case 1: //Placing
                            if (hatchDone && cargoDone) {
//                                Hatch.set(false, false);
                                setpoint = 1;
                                elevatorCase = 2;
                            }
                            break;
                        case 2: //Going down
                            override = true;
                            setpoint = 1;
                            elevatorCase = 0;
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
                    smoothAccelerateNum = (MathUtils.convertRange(previousDistance, previousDistance + commands[arrayIndex][4], commands[arrayIndex][0], commands[arrayIndex+1][0], SmartDashboard.getNumber("Distance", 0)));
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

                if (commands[arrayIndex][17] == 1.0) {
                    if (wallAlign(commands[arrayIndex][3], commands[arrayIndex][2], commands[arrayIndex][18], commands[arrayIndex][19])) {
                        override = true;
                    }

//                    else if (commands[arrayIndex][17] == 1.0 && commands[arrayIndex+1][16] != 0.0) {
                        //Do the placement things here
//                    }
//                    if (robotAligned) {
//                        override = true;
//                    }
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

                Vector driveCommands;
                driveCommands = MathUtils.convertOrientation(Math.toRadians(imu.getAngle()), FWD, STR);
                FWD = driveCommands.getY();
                STR = driveCommands.getX();
                RCW *= rSpeed;

//                System.out.println(FWD + "| |" + STR);
                driveTrain.drive(new Vector(-STR, FWD), -RCW);
                //FIXME, if point rotation happens, switch FL with BR and L with R
//				}

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

//				System.out.println(driveDone + "||" + arcCalculated + "||" + commands[arrayIndex][4] + "||" + (SmartDashboard.getNumber("Distance", 0) - previousDistance));


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
    }

    public void teleopInit() {
//		jet.startTeleop();

        imu.setOffset(imuOffset);
//Sets ids
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setID(1);
        SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setID(2);
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setID(4);
        SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setID(3);

//		RRLogger.start();
        Elevator.init();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        // LABEL teleop periodic
        autonomous = false;

//        Lift.climb(xbox1.RB(), xbox1.LB());
        SmartDashboard.putNumber("IMU Angle", imu.getAngle());
        SmartDashboard.putNumber("Elevator Setpoint", position);
        SmartDashboard.putNumber("L Trig", xbox2.LTrig());
        SmartDashboard.putNumber("R Trig", xbox2.RTrig());


        xbox1.setDeadband(0.2);
        xbox2.setDeadband(0.2);

        if (xbox2.LB()) {
            Elevator.setPosition(0.0, 0.0, 0.0, false, false, 0, true);
        } else {
            if (Elevator.getPosition() < 105.0 && Elevator.getPosition() > 1.0) {
                if (xbox2.X() || xbox2.Start()) {
                    Elevator.setPosition(xbox2.LStickY(), xbox2.LTrig(), xbox2.LStickX(), xbox2.LStickButton(), true, 0, false);
                } else {
                    Elevator.setPosition(xbox2.LStickY(), xbox2.LTrig(), xbox2.LStickX(), xbox2.LStickButton(), false, 0, false);
                }
            } else {
                Elevator.setPower(-Math.signum(Elevator.getPosition() - 13.5) * 0.4);
            }
        }

        SmartDashboard.putNumber("DPad", xbox2.DPad());

		Hatch.set(xbox2.A(), xbox2.B(), xbox2.LB());
        Hatch.ground(xbox2.LB());

		Cargo.set(xbox2.Start(), xbox2.X(), xbox2.Y(), Lidar.hasCargo(), Lidar.getBLLeftSensor(), Lidar.getBRRightSensor());


		SmartDashboard.putBoolean("Compressor", compressor.enabled());

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
        FWD = -xbox1.LStickY()/* / 10.5 * Ds.getBatteryVoltage() * 1.0*/;
//
//		}

        // strafe command (-1.0 to 1.0)
//		if (Math.abs(xbox1.LStickX()) >= 0.05) {
        STR = xbox1.LStickX() /*/ 10.5 * Ds.getBatteryVoltage() * 1.0*/;
//		}

        RCW =  xbox1.RStickX() * 0.3;

        // Increase the time it takes for the robot to accelerate

        //Get pressure sensor to disable once above 100 psi
        //Base it off actuations


//		System.out.println(FWD + "| |" + prevFWD[cmdCounter] + "| |" + STR + "| |" + prevSTR[cmdCounter] + "| |" + Math.toDegrees(Math.atan2(prevFWD[cmdCounter], prevSTR[cmdCounter])) + "| |" + Math.toDegrees(Math.abs(Math.atan2(FWD, STR) - Math.atan2(prevFWD[cmdCounter], prevSTR[cmdCounter]))));
//				Math.toDegrees(Math.atan2(FWD, STR)) + "| |"  + Math.toDegrees(Math.atan2(prevFWD[cmdCounter], prevSTR[cmdCounter])) + "| |"  + prevSTR[cmdCounter] + "| |"  + prevFWD[cmdCounter]); //Change cmdCounter to actually get old values
//		if (Math.abs(FWD- prevFWD[cmdCounter]) > 1.0) {
//			decelFWD.set(prevFWD[cmdCounter], 0.0, 0.5);
//			FWD = decelFWD.calculate();
//		}





        SmartDashboard.putNumber("PrevFWD", prevFWD[cmdCounter]);
        SmartDashboard.putNumber("Counter", cmdCounter);

//		if (cmdCounter > 4) {
//			cmdCounter = 0;
//		}

//		System.out.println(STR);


        if (imu.collisionDetected()) {
            xbox1.rumbleRight(1.0);
            xbox1.rumbleLeft(1.0);
        } else {
            xbox1.stopLeftRumble();
            xbox1.stopRightRumble();
        }

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

        double headingDeg = imu.getAngle();
        double headingRad = Math.toRadians(headingDeg);

        currentOrientedButton = xbox1.RB();
        if (currentOrientedButton && !previousOrientedButton) {
            fieldOriented = !fieldOriented;

        }
        previousOrientedButton = currentOrientedButton;

        SmartDashboard.putBoolean("Field Oriented", fieldOriented);

        if (fieldOriented) {
            Vector commands;
            commands = MathUtils.convertOrientation(headingRad, FWD, STR);
            FWD = commands.getY();
            STR = commands.getX();
        } else {
//			if (!robotBackwards) {
            FWD *= 0.5;
            STR *= 0.5;
            RCW *= 0.5;
//			}
        }

        keepAngle();

//        System.out.println(xbox1.DPad());
//        System.out.println(MathUtils.calculateContinuousError(45.0, imu.getAngle(), 360.0, 0.0));
//        System.out.println(Math.abs(MathUtils.calculateContinuousError(45.0, imu.getAngle(), 360.0, 0.0)) >= 3);


        if (xbox1.buttonPad() != -1) {
            FWD = 0.0;
            STR = 0.0;
            RCW = 0.0;
        }

        if (xbox1.buttonPad() == 45) {
            wallAlign(25.0, 25.0, 15.0, 6.3);
            STR += xbox1.LStickX() * placeholderName;
            FWD -= xbox1.LStickY() * placeholderName;

        } else if (xbox1.buttonPad() == 135) {
            wallAlign(152.0, 152.0, 14.0, 6.3);
            STR += xbox1.LStickX() * placeholderName;
            FWD -= xbox1.LStickY() * placeholderName;

        } else if (xbox1.buttonPad() == 180 && xbox1.LTrig() > 0.25) {
            wallAlign(180.0,  225.0, 12.0, 4.0);
            STR += xbox1.LStickX() * placeholderName;
            FWD -= xbox1.LStickY() * placeholderName;

        } else if (xbox1.buttonPad() == 180 && xbox1.RTrig() > 0.25) {
            wallAlign(180.0, 135.0, 12.0, 4.0);
            STR += xbox1.LStickX() * placeholderName;
            FWD -= xbox1.LStickY() * placeholderName;

        } else if (xbox1.buttonPad() == 225) {
            wallAlign(216.0, 216.0, 13.5, 6.3);
            STR += xbox1.LStickX() * placeholderName;
            FWD -= xbox1.LStickY() * placeholderName;

        } else if (xbox1.buttonPad() == 315) {
            wallAlign(333.0, 333.0, 13.5, 6.3);
            STR += xbox1.LStickX() * placeholderName;
            FWD -= xbox1.LStickY() * placeholderName;
        } else if (xbox1.buttonPad() == 90 || xbox1.buttonPad() == 0 || xbox1.buttonPad() == 270) {
//            keepAngle = 90;
//            if (LineSensor.get() == 1) {
//                STR += 0.1;
//            } else if (LineSensor.get() == 2) {
//                STR -= 0.1;
//            }
//        } else if (xbox1.buttonPad() == 270.0) {
//            keepAngle = 270;
//        } else if (xbox1.buttonPad() == 0) {

        }
        if (xbox1.buttonPad() != -1) {
            Vector commands;
            commands = MathUtils.convertOrientation(headingRad, FWD, STR);
            FWD = commands.getY();
            STR = commands.getX();
        } else {
            xbox2.stopLeftRumble();
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

        if (Math.abs(RCW) > 0.5) {
            RCW = Math.signum(RCW)*0.5;
        }

        //System.out.println(FWD + "|     |"  + STR + "|     |"  + prevFWD[cmdCounter] + "|     |"  +prevSTR[cmdCounter]);

        prevRCW[cmdCounter] = RCW;
        prevFWD[cmdCounter] = FWD;
        prevSTR[cmdCounter] = STR;


        SmartDashboard.putNumber("FWD", FWD);
        SmartDashboard.putNumber("STR", STR);
        SmartDashboard.putNumber("RCW", RCW);
        SmartDashboard.putNumber("IMU Angle", imu.getAngle());

        LineSensor.get();

        driveTrain.drive(new Vector(-STR, FWD), -RCW); // x = str, y = fwd, rotation = rcw
//		}

//		RRLogger.writeFromQueue();

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

        autoMove = 0;
        Elevator.setPower(0.0);
        driveTrain.drive(new Vector(0.0,0.0),  0.0);
        // When robot is turned on, disabledInit is called once
        if (disabled < 1) {
            System.out.println("Hello, this is the Ratchet Rockers' Robot, Rocket Rocker");
            disabled++;
        }

    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        // LABEL test
        double speed = (xbox1.RStickX() * 0.3);

        if (xbox1.DPad() != -1) {
            dx = xbox1.DPad();
        }

//		System.out.println("FL Angle: " + MathUtils.resolveDeg(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getAngle()));
//		System.out.println("BL Angle: " + MathUtils.resolveDeg(SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getAngle()));
//		System.out.println("BR Angle: " + MathUtils.resolveDeg(SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getAngle()));
//		System.out.println("FR Angle: " + MathUtils.resolveDeg(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getAngle()));

        // Move a single motor from the drivetrain depending on Dpad and right stick
//		if (dx == 0) {
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(speed);
//
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
//		}
//
//		if (dx == 45) {
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(speed);
//
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
//		}
//
//		if (dx == 90) {
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(speed);
//
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
//		}
//
//		if (dx == 135) {
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(speed);
//
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
//		}
//
//		if (dx == 180) {
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(speed);
//
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
//		}
//
//		if (dx == 225) {
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(speed);
//
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
//		}
//
//		if (dx == 270) {
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(speed);
//
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
//		}
//
//		if (dx == 315) {
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(speed);
//
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
//			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
//		}
    }
}
