package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.*;
//import com.kauailabs.navx.*;
//import java.util.*;
import com.revrobotics.*;
import frc.robot.*;

public class Elevator{

    private static CANSparkMax ElMotor;
    private static CANEncoder ElEncoder;
    private static CANPIDController ElPID;

    private static double ElP = 0.001;
    private static double ElI = 0.0;
    private static double ElD = 0.0;
    private static double ElF = 0.0;
    private static double ElMax = 1;
    private static double ElMin = -1;
    public static int ElCase = 0;

//    private static double ElSet = 0.0;

    private static boolean manual = false;
//    private static int manualToggle = 0;
//    private static boolean manualToggled = false;
    
    public static void init(int port){
        ElMotor = new CANSparkMax(port, CANSparkMaxLowLevel.MotorType.kBrushless);

        ElEncoder = new CANEncoder(ElMotor);

        ElMotor.setInverted(false);

        ElPID = ElMotor.getPIDController();
        ElPID.setP(ElP);
        ElPID.setI(ElI);
        ElPID.setD(ElD);
        ElPID.setIZone(0.0);
        ElPID.setFF(ElF);
        ElPID.setOutputRange(ElMin, ElMax);
    }

//    public static void input(){
        
//    }

    public static void set(double speed){
        manual = false;
        // int manualToggle = 0;
        // switch(manualToggle){
        //     case 0 :
        //         System.out.println("manual T");
        //         if((Robot.xboxEl.LStickButton() == true) && (Robot.xboxEl.RStickButton() == true)){
        //             manual = true;
        //             manualToggle = 1;
        //         }
        //         break;
        //     case 1 :
        //         System.out.println("manual F");
        //         if((Robot.xboxEl.LStickButton() == true) && (Robot.xboxEl.RStickButton() == true)){
        //             manual = false;
        //             manualToggle = 0;
        //         }
        //     break;
        //     }
        if(manual == false){
            if(Robot.xboxEl.LStickY() == 1){
                if(Robot.xboxEl.LB()==true){
                    ElCase = 1;
                }
            if(Robot.xboxEl.LStickX() == 1){
                if(Robot.xboxEl.LB()==true){
                    ElCase = 5;
            }
            else if(Robot.xboxEl.LStickY() == -1){
                if(Robot.xboxEl.LB()==true){
                    ElCase = 3;
                }
            if(Robot.xboxEl.LStickY() == 1){
                if(Robot.xboxEl.RB()==true){
                    ElCase = 4;
                }
            if(Robot.xboxEl.LStickY() == -1){
                if(Robot.xboxEl.RB()==true){
                    ElCase = 2;
                }
            if(Robot.xboxEl.LStickX() == 1){
                if(Robot.xboxEl.RB()==true){
                    ElCase = 6;
                }    

            }
            switch(ElCase){
                case 0 :
                    ElPID.setReference(0, ControlType.kPosition);
                    if(ElEncoder.getPosition() == 0){
                        ElMotor.stopMotor();
                        break;
                    }
                    break;
                case 1 :
                System.out.println("Case1");
                    ElPID.setReference(6, ControlType.kPosition);
                    if(ElEncoder.getPosition() == 6.3809)
                    {
                        System.out.println("Case1in");
                        ElMotor.stopMotor();
                        break;
                    }
                    else if(ElEncoder.getPosition() == 105)
                    {
                        ElMotor.stopMotor();
                        break;
                    }
                    break;
                    
                case 2 :
                    ElPID.setReference(14, ControlType.kPosition);
                    if(ElEncoder.getPosition() == 14.101)
                    {
                        ElMotor.stopMotor();
                        break;
                    }
                    else if(ElEncoder.getPosition() == 205)
                    {
                        ElMotor.stopMotor();
                        break;
                    }
                    break;
                case 3 :
                    ElPID.setReference(32, ControlType.kPosition);
                    if(ElEncoder.getPosition() == 295)
                    {
                        ElMotor.stopMotor();
                        break;
                    }
                    else if(ElEncoder.getPosition() ==305)
                    {
                
                        break;
                    }
                    break;
                case 4:
                   ElPID.setReference(40, ControlType.kPosition);
                   if(ElEncoder.getPosition() == 145)
                   {
                       ElMotor.stopMotor();
                       break;
                   }
                
                       else if (ElEncoder.getPosition() ==155)
                   {

                    break;
                }
                   
                   case 5:
                   ElPID.setReference(57, ControlType.kPosition);
                   if(ElEncoder.getPosition() == 145)
                   {
                       ElMotor.stopMotor();
                       break;
                   }
                
                       else if (ElEncoder.getPosition() ==155)
                   {

                    break;
                   }
                   
                   
                   case 6:
                   ElPID.setReference(65, ControlType.kPosition);
                   if(ElEncoder.getPosition() == 70)
                   {
                       ElMotor.stopMotor();
                       break;
                   }
                
                       else if (ElEncoder.getPosition() ==155)
                   {

                    break;
            }  
        }
    }
}
            }
        }
    }
    
          
        {
         if(manual == true){
            ElMotor.set(speed);
        }
         ElPID.setReference(300, ControlType.kPosition);
        System.out.println(ElEncoder.getPosition());
    }
        }

    
            }
    public static void control() {
    
    }
    public static void printOut(String printOut){
        System.out.println(printOut);
    }
}
