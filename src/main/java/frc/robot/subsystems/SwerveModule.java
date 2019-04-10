package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.Acceleration;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.PIDController;
import frc.robot.utilities.Vector;
import edu.wpi.first.wpilibj.DigitalInput;

import javax.naming.ldap.Control;
import java.awt.geom.Rectangle2D;
import java.awt.geom.RoundRectangle2D;

/**
 * A swerve module.
 */
public class SwerveModule {
    private static final double TICKS_PER_REVOLUTION = 360.0;
    private static final double TRANSLATION_DISTANCE_PER_PULSE = 1.0; //Multiply by 1.7 later
    private static final double ROTATION_DISTANCE_PER_PULSE = 6.0;
    private  static double SPEED_RATIO = 0.47;

    private static final int CAN_TIMEOUT = 20;
    private static final double SMALL_NUMBER = 0.015; //Was 0.05
    private static final double MAX_RPM = 5555; // 3500 // 5555

    private CANSparkMax translationMotor;
    private boolean wheelOverheat = false;
    private CANEncoder translationEncoder;
    private CANPIDController translationPID;

    private CANSparkMax rotationMotor;
    private CANEncoder rotationEncoder;
    private CANPIDController rotationPID;

    private final double THEORETICAL_MAX = 5676;
    private double kMaxOutput = 0.9;
    private double kMinOutput = -0.9;

    private double motorP = 1.651e-4;
    private double tempScalar = 0.0;
    private double motorI = 0.9e-6;
    private double motorD = 0.9e-6;
    private double motorF = 0.93/THEORETICAL_MAX;


    private Vector position;
    private double prevAngleCommand = 0.0;
    private  double maxP = 1.2e-3;
    private double frictionCompensationRatio = 0.0;
    private boolean useFrictionCompensate = false;
    private double speedScale = 0.0;
    private boolean prevReversed = false;
    private double prevRotationCommand = 0.0;
    private final double ACCEL_SPEED= 0.05;
    private final double DECEL_SPEED= 0.08;
    private boolean turbo = false;
    private int speedCase = 0;
    private double decelSpeed = 0.035;
    private double prevSpeedCommand = 0.0;
    private double speedCommand;
    private double angleCommand;
    private double distance = 0.0;
    private double previousDistance = 0.0;
    private double previousRobotDistance = 0.0;
    private double trueError;
    private double rawError;
    private double delta = 0.0;
    private double rightDelta = 0.0;
    private double forwardDelta = 0.0;
    private double rightSum = 0;
    private double forwardSum = 0;
//    private SwerveMotor swerveMotor;
//    private PIDController anglePID;
    private double keepPIDAngle;
    private double angle = 0.0;
    private boolean wheelReversed;
    private double reversePoint;
//    private DigitalInput wheelSensor;
    private double wheelOffsetFirst = 0.0;
    private double wheelOffset = 0.0;
    private double sensorCounter = 0.0;
    private  double testing = 0.0;

    public static XboxController xbox1 = new XboxController(0);

    private double angleOld;
    private double angleFiltered = 0.0;
    private double angleFilteredOld = 0.0;

    private boolean resettingAngle = false;
    private boolean foundSensor = false;
    private boolean foundFlag = false;

//Good base values:
//    private double translationP = 1.45e-4;
//    private double translationI = 1.6e-6;
//    private double translationD = 8.0e-5;

    private double translationP = 1.5e-4;//8.0e-4; //1.8, 1.3e-3
    private double translationI = 0.0;
    private double translationD = 0.0;

    private double rotationCommand = 0.0;

    private double rotationP = 1.0e-1;//8.0e-4; //1.8, 1.3e-3, lower P
    private double rotationI = 0.0;
    private double rotationD = 0.0;

    private double i;
    private double j;
    private double k;
    private double z;


//    double spin = 0;

    private int id;


    /**
     * @param canPortT Port of translation motor
     * @param canPortR Port of the rotation motor
     * @param sensorPort Roborio port of the allignment sensor
     */
    SwerveModule(int canPortT, int canPortR, int sensorPort) {
        super();


        translationMotor = new CANSparkMax(canPortT, CANSparkMaxLowLevel.MotorType.kBrushless);

        translationMotor.setCANTimeout(CAN_TIMEOUT);

        translationEncoder = new CANEncoder(translationMotor);

        translationMotor.setInverted(false);

        translationPID = translationMotor.getPIDController();
//        translationPID = new CANPIDController(translationMotor);
        translationPID.setP(translationP);
        translationPID.setI(translationI);
        translationPID.setD(translationD);
        translationPID.setIZone(0.0);
//        translationPID.setFF(translationF);

        translationPID.setOutputRange(kMinOutput, kMaxOutput);

        translationMotor.setSmartCurrentLimit(20, 10);

        translationMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 5);
        translationMotor.burnFlash();



        rotationMotor = new CANSparkMax(canPortR, CANSparkMaxLowLevel.MotorType.kBrushless);

        rotationMotor.setCANTimeout(CAN_TIMEOUT);

        rotationEncoder = new CANEncoder(rotationMotor);

        rotationMotor.setInverted(false);

        rotationPID = rotationMotor.getPIDController();
        rotationPID.setP(rotationP);
        rotationPID.setI(rotationI);
        rotationPID.setD(rotationD);
        rotationPID.setIZone(0.0);
//        rotationPID.setFF(rotationF);

        rotationPID.setOutputRange(kMinOutput, kMaxOutput);

//        translationEncoder.setPosition(0.0);
//        rotationEncoder.setPosition(0.0);
        rotationMotor.setSmartCurrentLimit(20, 10);

        rotationMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 5);
        rotationMotor.burnFlash();

//        anglePID = new PIDController(0.0, 0.0, 0.0);
//        anglePID.setContinuous();
//        anglePID.setInputRange(0.0, 360.0);
//        anglePID.setOutputRange(-1.0, 1.0);
//        anglePID.enable();

        SmartDashboard.putNumber("translationP", translationP);
        SmartDashboard.putNumber("translationI", translationI);
        SmartDashboard.putNumber("translationD", translationD);
//        wheelSensor = new DigitalInput(sensorPort);
    }

    void drive() {
        //This allows us to reset the wheels with the pulleys on the inside
        if (id == 1 || id == 4) { //id 4 was inverted with wires
            translationMotor.setInverted(true);
        }

        distance = translationEncoder.getPosition() * TRANSLATION_DISTANCE_PER_PULSE;

        angle = rotationEncoder.getPosition()/* * ROTATION_DISTANCE_PER_PULSE*/;

        wheelOffset = 0.0;

        angleOld = angle;

        //Count rotation cycles of wheel
        i = Math.floor(rotationEncoder.getPosition() * ROTATION_DISTANCE_PER_PULSE/360.0);
//        i = Math.floor(rotationMotor.getSensorCollection().getAnalogIn() / 1024);


        //Set command + rotations in encoder units (wrap command)
        j = angleCommand + i * 360.0/ROTATION_DISTANCE_PER_PULSE;
        //        j = this.angleCommand + i * 1024;

        //Set wrapped command - current position (error)
        k = j - rotationEncoder.getPosition();
//        k = j - rotationMotor.getSensorCollection().getAnalogIn();

        /*
         * If the error is greater than 30 units (180 degrees), have wheel go to next
         * cycle so it doesn't jump back to beginning of current cycle
         */
        if (Math.abs(k) > 180.0/ROTATION_DISTANCE_PER_PULSE) {
            angleCommand = -(j - Math.signum(k) * 360.0/ROTATION_DISTANCE_PER_PULSE);
        } else {
            angleCommand =  -j;
        }

        //        if (Math.abs(k) > 512) {
//            z = -(j - Math.signum(k) * 1024);
//        } else {
//            z = -j;
//        }


        //        angle = MathUtils.resolveDeg(/*wheelOffset*/swerveMotor.getAngle()); //FIXME!!! This will cause error unless it is initiated by the button that rotates them all in the same direction!

//        angleFilteredOld = angleFiltered;
//
//        if (angle-angleOld >= 45/ROTATION_DISTANCE_PER_PULSE) {
//            angleOld += 360/ROTATION_DISTANCE_PER_PULSE;
//        } else if (angleOld-angle >= 45/ROTATION_DISTANCE_PER_PULSE) {
//            angleOld -= 360/ROTATION_DISTANCE_PER_PULSE;
//        }

//        angleFiltered = MathUtils.resolveDeg((angle+angleOld+9*angleFilteredOld)/11);

//        trueError = MathUtils.calculateContinuousError(-angleCommand, angle, 360.0/ROTATION_DISTANCE_PER_PULSE, 0.0);

//        if (!wheelReversed) {
            delta = previousDistance - distance;
//        } else {
//            delta = distance-previousDistance;
//        }

        /*
         * If the wheel has to move over 90 degrees
         * go opposite to command and reverse translation
         */

        if (Math.abs(MathUtils.getDelta(-angleCommand, angle)) > (90.0/ROTATION_DISTANCE_PER_PULSE) ) {
            angleCommand += Math.signum(MathUtils.getDelta(-angleCommand, angle)) * 180.0/ROTATION_DISTANCE_PER_PULSE;
            speedCommand *= -1;
            wheelReversed = true;
//            System.out.println("Reversing!");
        } else {
            wheelReversed = false;
        }

        /*
         * If wheel direction has to change more than 22.5 degrees
         * then set wheel speed command to 0 while wheel is turning.
         */
//        if (Math.abs(anglePID.getError()) > TICKS_PER_REVOLUTION / 16) {
//            speedCommand = 0.0;
//        }

        if (id == 1) {
//            System.out.println(i + " | |" + j +  "| |" + k);

//            SmartDashboard.putNumber("Wheel rotations", i);
//            SmartDashboard.putNumber("Wheel rotations", i);

//            SmartDashboard.putNumber("I", i);
//            SmartDashboard.putNumber("J", j);
//            SmartDashboard.putNumber("K", k);


            SmartDashboard.putNumber("FR Speed Command", speedCommand);
            SmartDashboard.putNumber("FR Angle Command", angleCommand);//MathUtils.resolveDeg(angleCommand*ROTATION_DISTANCE_PER_PULSE));

//            SmartDashboard.putNumber("FR Speed", translationEncoder.getVelocity());
//            SmartDashboard.putNumber("FR Scalar", tempScalar);

//            SmartDashboard.putNumber("FR Angular Speed", rotationEncoder.getVelocity());

            SmartDashboard.putNumber("FR Offset", wheelOffset);
            SmartDashboard.putNumber("FR Dist", distance);
            SmartDashboard.putNumber("Error", Math.abs(MathUtils.getDelta(-angleCommand, angle)));

            SmartDashboard.putNumber("FR Angle", rotationEncoder.getPosition());

            SmartDashboard.putNumber("FR Calculated Angle", angle);//MathUtils.resolveDeg(angle*ROTATION_DISTANCE_PER_PULSE));

        } else if (id == 2) {
            SmartDashboard.putNumber("FL Angle Command", angleCommand*ROTATION_DISTANCE_PER_PULSE);
            SmartDashboard.putNumber("FL Speed Command", speedCommand);
            SmartDashboard.putNumber("FL Calculated Angle", angle);

//            SmartDashboard.putBoolean("FL Sensor", !wheelSensor.get());
            SmartDashboard.putNumber("FL Offset", wheelOffset);
            SmartDashboard.putNumber("FL Dist", distance);
//            if (!wheelSensor.get()) {
//                SmartDashboard.putNumber("FL Sensor Angle ",  angle);
//            }

        } else if (id == 3) {
            SmartDashboard.putNumber("BR Angle Command", angleCommand*ROTATION_DISTANCE_PER_PULSE);
            SmartDashboard.putNumber("BR Speed Command", speedCommand);
//            SmartDashboard.putBoolean("BR Sensor", !wheelSensor.get());
            SmartDashboard.putNumber("BR Offset", wheelOffset);
            SmartDashboard.putNumber("BR Calculated Angle", angle);
            SmartDashboard.putNumber("BR Dist", distance);
//            if (!wheelSensor.get()) {
//                SmartDashboard.putNumber("BL Sensor Angle ",  angle);
//            }

        } else {
            SmartDashboard.putNumber("BL Angle Command", angleCommand*ROTATION_DISTANCE_PER_PULSE);
            SmartDashboard.putNumber("BL Speed Command", speedCommand);
//            SmartDashboard.putBoolean("BL Sensor", !wheelSensor.get());
            SmartDashboard.putNumber("BL Offset", wheelOffset);
            SmartDashboard.putNumber("BL Dist", distance);
            SmartDashboard.putNumber("FL Calculated Angle", angle);

//            if (!wheelSensor.get()) {
//                SmartDashboard.putNumber("BR Sensor Angle ",  angle);
//            }
        }

        //Keepangle for the wheels
        if (Math.abs(speedCommand ) < 0.04) {
            angleCommand = prevAngleCommand;
        }


        if (translationMotor.getMotorTemperature() > 90.0 || rotationMotor.getMotorTemperature() > 90.0) {
            wheelOverheat = true;
        }
        if (wheelOverheat && translationMotor.getMotorTemperature() < 80.0 && translationMotor.getMotorTemperature() < 80.0) {
            wheelOverheat = false;
        }
//        if (id == 2) {
//            translationMotor.set(speedCommand);
//            rotationMotor.set(rotationCommand);
        if (!wheelOverheat) {
            translationPID.setReference(speedCommand * THEORETICAL_MAX, ControlType.kVelocity);
//            translationMotor.set(2*speedCommand);
//            SmartDashboard.putNumber("Rotation Command", rotationCommand);

//            System.out.println(speedCommand + "||" + angleCommand);
//            SmartDashboard.putNumber("Angle Command", MathUtils.resolveDeg(tempScalar));
            rotationPID.setReference(-angleCommand, ControlType.kPosition);
        }
//        }

//        if (id == 1) {
//            translationPID.setReference(speedCommand*THEORETICAL_MAX, ControlType.kVelocity);
////            translationMotor.set(2*speedCommand);
////            SmartDashboard.putNumber("Rotation Command", rotationCommand)
////            System.out.println(speedCommand + "||" + angleCommand);
////            SmartDashboard.putNumber("Angle Command", MathUtils.resolveDeg(tempScalar));
//            rotationPID.setReference(-angleCommand, ControlType.kPosition);
//        }

        prevRotationCommand = rotationCommand;
        prevAngleCommand = angleCommand;
        prevSpeedCommand = speedCommand;
        prevReversed = wheelReversed;



        rightDelta = delta * Math.sin(Math.toRadians(angle * ROTATION_DISTANCE_PER_PULSE));
        forwardDelta = delta * Math.cos(Math.toRadians(angle * ROTATION_DISTANCE_PER_PULSE));


        previousDistance = distance;
        previousRobotDistance = SwerveDrivetrain.getRobotDistance();
    }

    public void setID(int id) {
        this.id = id;
    }


    public void setOffset(double set) {
        wheelOffsetFirst = set;
    }
    public double getAngle() {
        return angle;
    }

    public void setSpeedCommand(double speedCommand) {
        this.speedCommand = speedCommand;
    }

    public void scaleSpeedCommand(double scaler) {
        this.speedScale = scaler;
    }
    public double getRotationCommand() {
        return rotationCommand;
    }
    public void disableSpeed(int speedCase) {
        this.speedCase = speedCase;
    }

    public void setRotationCommand(double rotationCommand) {
        this.rotationCommand = rotationCommand;
        if (Math.signum(prevRotationCommand) == Math.signum(this.rotationCommand)) {
            tempScalar += 5*Math.signum(this.rotationCommand);
        }
//        else {
//            tempScalar = 0.0;
//        }
    }
    void setAngleCommand(double angleCommand) {
        this.angleCommand = angleCommand/ROTATION_DISTANCE_PER_PULSE;
    }

    void resetAngle (boolean set) {
        resettingAngle = set;

        if (!foundFlag) {
            foundSensor = false;
            foundFlag = true;
            SmartDashboard.putBoolean("Wheel 1 Aligned", false);
            SmartDashboard.putBoolean("Wheel 2 Aligned", false);
            SmartDashboard.putBoolean("Wheel 3 Aligned", false);
            SmartDashboard.putBoolean("Wheel 4 Aligned", false);
        }
    }

    public void setFoundFlag (/*boolean startButton*/) {
        foundFlag= false;
//        buttonDown = startButton;
    }

    public double getSpeedCommand() {
        return speedCommand;
    }

    public double getAngleCommand() {
        return angleCommand;
    }

    Vector getPosition() {
        return position;
    }

    public void setPosition(Vector position) {
        this.position = position;
    }

    public double getAngleError() {
        return Math.abs(trueError);
    }

    public void checkFrictionCompensation(double averageError) {
//        useFrictionCompensate = (getAngleError() > averageError);
        if (getAngleError() > averageError) {
            frictionCompensationRatio = maxP * (getAngleError() - averageError) / 180.0;
        } else {
            frictionCompensationRatio = 0.0;
        }
        //Angle Error should only reasonably reach 180, so increase P to 1.0e-3 when at 180, but start at 5.0e-3 normally
        //If a wheel is 90 off more than the rest, its P will double
    }

    public boolean getAngleOff() {
        return (Math.abs(trueError) > 22.5);
    }

    public double getDistance() {
        return distance;
    }

    void resetDistance() {
//        swerveMotor.reset();
    }
    public void resetWheel() {
        this.prevSpeedCommand = 0.0;
        this.speedCommand = 0.0;
    }
    public boolean getReversed() {
        return this.wheelReversed;
    }

    public void turbo(boolean turboCase) {
        this.turbo = turboCase;
    }
    public double getRightSum() {
        return rightSum;
    }

    public double getForwardSum() {
        return forwardSum;
    }

    public static void setSpeedRatio(double ratio) {
        SPEED_RATIO = ratio;
    }

    public void resetDelta() {
        rightSum = 0.0;
        forwardSum = 0.0;
        delta = 0.0;

    }

    public double getTranslationSpeed() {
        return translationEncoder.getVelocity();
    }

    public double[] getXYDist() {
        if (wheelReversed) {
            forwardSum *= -1.0;
            rightSum *= -1.0;
//            rightDelta *= -1.0;
//            forwardDelta *= -1.0;
        }
        double[] i = {rightDelta, forwardDelta};
        return i;
    }
}
