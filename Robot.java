/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST Robotics Team # 3571                                   */
/*                                                                            */
/* Authors :                                                                  */
/* Evan C. Richinson                                                          */
/* Tomas Rakusan                                                              */
/*----------------------------------------------------------------------------*/
 
package edu.wpi.first.wpilibj.templates;
 
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.DriverStationLCD.Line;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.HSLImage;
import edu.wpi.first.wpilibj.image.NIVision.MeasurementType;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
 
public class RobotTemplate extends IterativeRobot {
     
    //Driver Station Setup
    DriverStationLCD dlg = DriverStationLCD.getInstance();
     
    //Drive Control Setup
    Joystick joy_LeftStick = new Joystick(1);   //Driver Controller
    Joystick joy_RightStick = new Joystick(2);  //Operator Controller
     
    // left Joysstick buttons
    int btn_Reverse=1;             // 'a' Green Button
    int btn_Stop_Base=2;           // 'b' Red Button
    int btn_Shift_Speed=3;         // 'x' Blue Button
     
    // Right joystick buttons
    int btn_r_Hang=10;             // Right Controller Click
    int btn_r_Fire=5;              // Left Bumper Button
    int btn_r_Run_Shooter=6;       // Right Bumper Button
     
    //button read states
    boolean btnLastRd_r_Fire = false;
    boolean btnLastRd_r_Hang = false;
    boolean btnLastRd_r_Run_Shooter = false;
    boolean btnLastRd_r_ChangeSpeed = false;
     
    //Axis Variables
    double driveX=0.0;
    double driveY=0.0;
    double dbl_Shooter_Axis=0.0;
    boolean speedChanging=false;
     
    //Victor Setup
    RobotDrive drive = new RobotDrive(1,2,3,4);
    Victor mtr_Shooter_Angle=new Victor(5);
    Victor mtr_Shooter=new Victor(6);
     
    //Victor Max Speed
    double dbl_Max_Shooter_Angle_Speed_Up=-0.5;
    double dbl_Max_Shooter_Angle_Speed_Down=-0.3;
             
    //Compressor Setup
    Compressor compressor1=new Compressor(7,1);                 //Compressor Relay
    DoubleSolenoid sol_Drive_Shifter=new DoubleSolenoid(1,2);   //Shifter on Outputs 1 & 2
    Solenoid sol_fire=new Solenoid(3);                          //Fire on Output 3
    Solenoid sol_hang=new Solenoid(4);                          //Hang on Output 4
     
    //Encoder Setup
    Encoder leftMotorEncoder=new Encoder(1,2,true);
    Encoder rightMotorEncoder=new Encoder(3,4,false);
    Encoder shooterAngle=new Encoder(5,6);
     
    //Timing & Shooting
    Timer shooter_Timer=new Timer();
    Timer fire_Timer=new Timer();
     
    //Display Variables
    String[] disp={"","","","","",""};
    String str_Shooter_Before_Ready=new String("Shooter Ready in : ");
    String str_Shooter_Ready=new String("Shooter READY !            ");
    String str_Shooter_Not_Ready=new String("Shooter Off            ");
    String str_Left_Encoder=new String("leftMotorEncoder=");
    String str_Right_Encoder=new String("rightMotorEncoder=");
     
    //Auto Variables
    boolean bool_Shooter_Run=false;
    boolean bool_Auto_Run=false;
    boolean AutoShoot=true;
    boolean CameraON=false;
    boolean CameraMove=false;
    int int_Shoots_Taken=0;
    int CameraImageSizeY;
    int CameraTargetY;
     
    //Fire Piston & Hang Piston
    boolean bool_Fire=false;
    boolean bool_Hang=false;
     
    //Camera Setup
    AxisCamera camera;
    HSLImage image;
    CriteriaCollection cc;                      // Camera Vision Collection
    String CameraIP=new String("10.35.71.11");
    Relay lights=new Relay(3);                  // Light Relay
     
    //Encoder Variables
    double dEncoderDpP=0.0095162569;
    boolean TestLefMotortEncoder=false;
    boolean testEncoderDpP=false;
     
    //Error Variables
    int ErrorLine=0;
     
    public void robotInit()
    {
        leftMotorEncoder.setDistancePerPulse(dEncoderDpP);
        rightMotorEncoder.setDistancePerPulse(dEncoderDpP);
        shooterAngle.setDistancePerPulse(1);
         
        if (CameraON) {
            camera = AxisCamera.getInstance(CameraIP);
            camera.writeResolution(AxisCamera.ResolutionT.k320x240);
            camera.writeBrightness(30);
            CameraImageSizeY = 240;
             
            cc = new CriteriaCollection();
            cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH, 30, 400, false);
            cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_HEIGHT, 40, 400, false);
        }
         
        sol_Drive_Shifter.set(DoubleSolenoid.Value.kReverse);
        compressor1.start();
        ErrorLine=0;
    }
     
    public void autonomousInit()
    {
        bool_Auto_Run=true;
        int_Shoots_Taken=0;
         
        //Start Up Encoders and Timers
        leftMotorEncoder.start();
        rightMotorEncoder.start();
         
        //Set Init Values
        lights.set(Relay.Value.kOn);
        sol_fire.set(false);
         
        compressor1.start();
    }
    public void autonomousPeriodic() {
            try {
                if (CameraON) {
                    //Grab Image and Process
                    ColorImage Image = camera.getImage();
                    BinaryImage thresholdImage = Image.thresholdRGB(140, 160, 140, 160, 140, 160);   // keep only red objects
                    BinaryImage bigObjectsImage = thresholdImage.removeSmallObjects(false, 2);  // remove small artifacts
                    BinaryImage convexHullImage = bigObjectsImage.convexHull(false);          // fill in occluded rectangles
                    BinaryImage filteredImage = convexHullImage.particleFilter(cc);           // find filled in rectangles
                     
                    ErrorLine=1;
                     
                    //Tell Us What You Found
                    ParticleAnalysisReport[] reports = filteredImage.getOrderedParticleAnalysisReports();  // get list of results
                     
                    CameraTargetY = 0;
                     
                    for (int i = 0; i < reports.length; i++) {                                // print results
                        ParticleAnalysisReport r = reports[i];
                        System.out.println("Particle: " + i + " center x= " + r.center_mass_x +"center y="+r.center_mass_y + " x="+r.imageWidth+" y="+r.imageHeight);
                         
                        if (CameraMove) {
                            if (CameraTargetY < r.center_mass_y) {
                                CameraTargetY = r.center_mass_y;
                            }
                            if (CameraImageSizeY != r.imageHeight) {
                                System.out.println("Image Size Difference " + CameraImageSizeY + " , " + r.imageHeight);
                                //CameraImageSizeY = r.imageHeight;
                            }
                        }
                    }
                     
                    if (CameraMove){
                        //Angle to the Target
                        if (CameraTargetY < (CameraImageSizeY / 2)) {
                            mtr_Shooter_Angle.set(1 * dbl_Max_Shooter_Angle_Speed_Up);
                        }
                        if (CameraTargetY > (CameraImageSizeY / 2)) {
                            mtr_Shooter_Angle.set(1 * dbl_Max_Shooter_Angle_Speed_Down);
                        }
                    }
                    ErrorLine=2;
                     
                    System.out.println(filteredImage.getNumberParticles() + "  " + Timer.getFPGATimestamp());
                     
                    // Clear 'Floating' Objects
                    filteredImage.free();
                    convexHullImage.free();
                    bigObjectsImage.free();
                    thresholdImage.free();
                    Image.free();
                     
                    ErrorLine=3;
                }
                 
                if (AutoShoot) {
                    if (!bool_Shooter_Run && bool_Auto_Run) {
                        shooter_Timer.start();
                        bool_Shooter_Run=true;
                        mtr_Shooter.set(1);
                    }
                    if (bool_Shooter_Run && bool_Auto_Run && (shooter_Timer.get()>6) && int_Shoots_Taken==0) {
                        int_Shoots_Taken++;
                        fire_Timer.start();
                        sol_fire.set(true);
                    }
                    if (bool_Shooter_Run && bool_Auto_Run && (shooter_Timer.get()>9) && int_Shoots_Taken==1) {
                        int_Shoots_Taken++;
                        fire_Timer.start();
                        sol_fire.set(true);
                    }
                    if (bool_Shooter_Run && bool_Auto_Run && (shooter_Timer.get()>12) && int_Shoots_Taken==2) {
                        int_Shoots_Taken++;
                        fire_Timer.start();
                        sol_fire.set(true);
                    }
                    if (bool_Shooter_Run && bool_Auto_Run && (shooter_Timer.get()>14) && int_Shoots_Taken==3) {
                        int_Shoots_Taken++;
                        fire_Timer.start();
                        sol_fire.set(true);
                    }
                    if (bool_Shooter_Run && bool_Auto_Run && (shooter_Timer.get()>14.5)) {
                        mtr_Shooter.stopMotor();
                        sol_fire.set(false);
                        fire_Timer.reset();
                        shooter_Timer.reset();
                        bool_Shooter_Run=false;
                        bool_Auto_Run=false;
                        int_Shoots_Taken=0;
                    }
                    if (bool_Shooter_Run && bool_Auto_Run && (fire_Timer.get()>1)) {
                        fire_Timer.reset();
                        sol_fire.set(false);
                    }
                }
            }
            catch (Exception ex) {
                System.out.println(ex);
                dlg.println(Line.kUser1, 1, "a"+ErrorLine+" "+ex.toString());
                dlg.updateLCD();
            }
    }
     
    public void teleopInit()
    {
        ErrorLine=0;
         
        disp[2]="Motor Speed: LOW ";
        driveY=0;
        driveX=0;
        speedChanging=false;
        leftMotorEncoder.reset();
        rightMotorEncoder.reset();
        leftMotorEncoder.start();
        rightMotorEncoder.start();
        lights.set(Relay.Value.kOn);
        sol_Drive_Shifter.set(DoubleSolenoid.Value.kReverse);
        sol_fire.set(false);
        sol_hang.set(false);
         
        compressor1.start();
    }
     
    public void teleopPeriodic()
    {
        try {
                //Joystick Get Axis
                driveY = -joy_LeftStick.getAxis(Joystick.AxisType.kY);
                driveX = joy_LeftStick.getAxis(Joystick.AxisType.kX);
                 
                if (joy_LeftStick.getRawButton(btn_Reverse)) {
                    driveY = -driveY;
                }
                 
                //Drive Stop and Drive Under 5% For Axis Trim Cleanup
                if(joy_LeftStick.getRawButton(btn_Stop_Base) || Math.floor(Math.abs((driveX+driveY)*100))<10) {
                    drive.stopMotor();
                }
                else {
                    drive.arcadeDrive(driveY,driveX);
                }
                 
                ErrorLine=1;
                 
                //Shooter Angle
                dbl_Shooter_Axis = joy_RightStick.getAxis(Joystick.AxisType.kY);
                //dbl_Shooter_Axis = joy_RightStick.getRawAxis(3);
                 
                if ((dbl_Shooter_Axis > 0.05) || (dbl_Shooter_Axis < -0.05)) {
                    if (dbl_Shooter_Axis>0) {
                        mtr_Shooter_Angle.set(dbl_Shooter_Axis * dbl_Max_Shooter_Angle_Speed_Down);
                    }
                    else if (dbl_Shooter_Axis<0) {
                        mtr_Shooter_Angle.set(dbl_Shooter_Axis * dbl_Max_Shooter_Angle_Speed_Up);
                    }
                }
                else {
                    mtr_Shooter_Angle.stopMotor();
                }
                 
                // Frisbee Firing
                if (joy_RightStick.getRawButton(btn_r_Fire) && !btnLastRd_r_Fire) {
                    sol_fire.set(true);
                }
                else if (!joy_RightStick.getRawButton(btn_r_Fire) && btnLastRd_r_Fire) {
                    sol_fire.set(false);
                }
                btnLastRd_r_Fire = joy_RightStick.getRawButton(btn_r_Fire);
                 
                // Robot Hang
                if (joy_RightStick.getRawButton(btn_r_Hang) && !btnLastRd_r_Hang) {
                    sol_hang.set(!sol_hang.get());
                }
                btnLastRd_r_Hang = joy_RightStick.getRawButton(btn_r_Hang);
                 
                // Shooter Full Speed
                if (joy_RightStick.getRawButton(btn_r_Run_Shooter) &&  !btnLastRd_r_Run_Shooter) {
                    mtr_Shooter.set(1);
                }
                else if (!joy_RightStick.getRawButton(btn_r_Run_Shooter) &&  btnLastRd_r_Run_Shooter) {
                    mtr_Shooter.stopMotor();
                }
                btnLastRd_r_Run_Shooter = joy_RightStick.getRawButton(btn_r_Run_Shooter);
                 
                ErrorLine=2;
                 
                // Change Gear Nicely...
                if (joy_LeftStick.getRawButton(btn_Shift_Speed) && !btnLastRd_r_ChangeSpeed) {
                    if (!speedChanging  ) {
                        disp[2]="Motor Speed: HIGH";
                        driveX=driveX/2.27;
                        driveY=driveY/2.27;
                        sol_Drive_Shifter.set(DoubleSolenoid.Value.kForward);
                        speedChanging=true;
                    }
                    else {
                        disp[2]="Motor Speed: LOW ";
                        if (Math.abs(driveX*2.27)>1) {
                            drive.arcadeDrive(driveX/2.27, driveY/2.27);
                        }
                        else {
                            driveX=Math.min(Math.max(driveX*2.27, -1.0), 1.0);
                            driveX=Math.min(Math.max(driveX*2.27, -1.0), 1.0);
                        }
                        sol_Drive_Shifter.set(DoubleSolenoid.Value.kReverse);
                        speedChanging=false;
                    }
                }
                btnLastRd_r_ChangeSpeed = joy_LeftStick.getRawButton(btn_Shift_Speed);
                 
                ErrorLine=3;
                 
                disp[3]=str_Left_Encoder+String.valueOf(Math.floor(leftMotorEncoder.getDistance()*1000)/1000)+"    ";
                disp[4]=str_Right_Encoder+String.valueOf(Math.floor(rightMotorEncoder.getDistance()*1000)/1000)+"    ";
                 
                ErrorLine=4;
                 
                if(TestLefMotortEncoder && testEncoderDpP) {
                    if (leftMotorEncoder.getDistance()!=0) {
                    disp[0]=str_Left_Encoder+String.valueOf(60/leftMotorEncoder.getDistance())+"      ";
                    }
                    else {
                        disp[0]=str_Left_Encoder+"0        ";
                    }
                }
                else if(testEncoderDpP && !TestLefMotortEncoder) {
                    if (leftMotorEncoder.getDistance()!=0) {
                    disp[0]=str_Right_Encoder+String.valueOf(60/rightMotorEncoder.getDistance())+"      ";
                    }
                    else {
                        disp[0]=str_Right_Encoder+"0          ";
                    }
                }
                 
                // Display For Shooter Ready or Not
                else if((bool_Shooter_Run && shooter_Timer.get()>2.0)||(int_Shoots_Taken>0 && shooter_Timer.get()>1)) {
                    disp[0]=str_Shooter_Ready;
                }
                else if(bool_Shooter_Run)
                {
                    if (int_Shoots_Taken>0 && shooter_Timer.get()<1) {
                        disp[0]=str_Shooter_Before_Ready+(Math.floor((1-shooter_Timer.get())*100)/100);
                    }
                    else if(int_Shoots_Taken==0 && shooter_Timer.get()<8) {
                        disp[0]=str_Shooter_Before_Ready+(Math.floor((8-shooter_Timer.get())*100)/100);
                    }
                }
                else {
                    disp[0]=str_Shooter_Not_Ready;
                }
                ErrorLine=5;
                 
                disp[1]="Advance Control";
                disp[5]="sAngleEncoder="+(2/shooterAngle.getDistance());
            }
            catch(Exception e) {
                disp[0]=("T"+ErrorLine+" "+e.toString());
                System.out.println(e);
            }
         
        dlg.println(Line.kUser1, 1, disp[0]);
        dlg.println(Line.kUser2, 1, disp[1]);
        dlg.println(Line.kUser3, 1, disp[2]);
        dlg.println(Line.kUser4, 1, disp[3]);
        dlg.println(Line.kUser5, 1, disp[4]);
        dlg.println(Line.kUser6, 1, disp[5]);
        dlg.updateLCD();
    }
    public void disabledInit() {
        try {
            drive.arcadeDrive(0.0,0.0);
            sol_Drive_Shifter.set(DoubleSolenoid.Value.kOff);
             
            compressor1.stop();
             
            mtr_Shooter.stopMotor();
            sol_fire.set(false);
             
            fire_Timer.reset();
            shooter_Timer.reset();
             
            bool_Shooter_Run=false;
            bool_Auto_Run=false;
            int_Shoots_Taken=0;
        }
        catch(Exception e) {
            System.out.println(e);
        }
        leftMotorEncoder.stop();
        rightMotorEncoder.stop();
        lights.set(Relay.Value.kOff);
    }
}