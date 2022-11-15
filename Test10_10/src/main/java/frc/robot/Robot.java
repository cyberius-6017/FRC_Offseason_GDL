// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.can.CANInvalidBufferException;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
//dimport edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Victor;

import edu.wpi.first.wpilibj.Timer; 

import javax.xml.namespace.QName;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/** This is a demo program showing how to use Mecanum control with the MecanumDrive class. */
public class Robot extends TimedRobot {
  private static final int kFrontLeftChannel = 40;
  private static final int kRearLeftChannel = 30;
  private static final int kFrontRightChannel = 20;
  private static final int kRearRightChannel = 10;

  private static final int kIndexer = 50;
  //private static final int kEscaladorChannel = 4;

  private static final int kIntakeChannel = 27;

  private static final int kRearShooterChannel = 28;
  private static final int kFrontShooterChannel =26;

  //private static final int kLimitSwitchEscalador = 7;

  private static final int kXbocControllerChannel = 0;

  private MecanumDrive m_robotDrive;
  private XboxController m_stick;

  //private MotorController escalador;
  private VictorSPX shooterRear;
  private VictorSPX shooterFront;

  //private boolean canDownElevator = true;
  private boolean longShot = true;

  
  private boolean eneableShooter = false;

  //private DigitalInput switchEscalador;

  private ADXRS450_Gyro gyroscope;

  private VictorSPX intake; 
  private CANSparkMax indexer;
  
  private Compressor pcmCompressor; 
  private Solenoid intakeSolenoid1;
  private Solenoid intakeSolenoid2;

  private boolean intakeFlag;
  Timer timer_nTimer = new Timer();

  @Override
  public void robotInit() {

    CameraServer.startAutomaticCapture();
  
    CANSparkMax frontLeft = new CANSparkMax(kFrontLeftChannel, MotorType.kBrushless);
    CANSparkMax rearLeft = new CANSparkMax(kRearLeftChannel, MotorType.kBrushless);
    CANSparkMax frontRight = new CANSparkMax(kFrontRightChannel, MotorType.kBrushless);
    CANSparkMax rearRight = new CANSparkMax(kRearRightChannel, MotorType.kBrushless);
    indexer = new CANSparkMax(kIndexer, MotorType.kBrushless);
    
    //escalador = new CANSparkMax(kEscaladorChannel, MotorType.kBrushless);

    shooterRear = new VictorSPX(kRearShooterChannel);
    shooterFront = new VictorSPX(kFrontShooterChannel);

    //switchEscalador = new DigitalInput(kLimitSwitchEscalador);

    gyroscope = new ADXRS450_Gyro();
    
    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    frontRight.setInverted(true);
    rearRight.setInverted(true);

    intake = new VictorSPX(kIntakeChannel);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    m_stick = new XboxController(kXbocControllerChannel);

    pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    intakeSolenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    intakeSolenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

    shooterRear.set(ControlMode.PercentOutput,0.0);
    shooterFront.set(ControlMode.PercentOutput,0.0);
  }

  
  @Override
  public void autonomousInit(){
    timer_nTimer.reset();
    timer_nTimer.start();
    m_robotDrive.drivePolar(0.0, 0.0, 0.0);
    shooterRear.set(ControlMode.PercentOutput,0.0);    
    shooterFront.set(ControlMode.PercentOutput,0.0);
    indexer.set(0.0);  
  }
  @Override
  public void autonomousPeriodic(){
    pcmCompressor.enableDigital();
    System.out.println(timer_nTimer.get());
    if(timer_nTimer.get() < 1.7){
      m_robotDrive.drivePolar(0.3, 0.0, 0.0);
    }
    else if(timer_nTimer.get() < 3.0) { 
      shooterRear.set(ControlMode.PercentOutput,-0.7);    
      shooterFront.set(ControlMode.PercentOutput,-0.7);
    }
    else if(timer_nTimer.get() < 6.0){
      indexer.set(0.6);
      intake.set(ControlMode.PercentOutput, -0.9);
    }
    else if(timer_nTimer.get() < 7.5){
      indexer.set(0.0);
      intake.set(ControlMode.PercentOutput, 0.0);
    }
    else if(timer_nTimer.get() < 9.0){
      indexer.set(0.6);
      intake.set(ControlMode.PercentOutput, -0.9);
    }
    else{
      m_robotDrive.drivePolar(0.0, 0.0, 0.0);
      shooterRear.set(ControlMode.PercentOutput,0.0);    
      shooterFront.set(ControlMode.PercentOutput,0.0);
      intake.set(ControlMode.PercentOutput, 0.0);
      indexer.set(0.0);  
    }
  }

  @Override
  
  public void teleopInit(){
   intakeFlag = false;
   intakeSolenoid1.set(true);
   intakeSolenoid2.set(true);
  }

  @Override
  public void teleopPeriodic() {
    // Use the left X axisS for lateral movement, Y axis for forward
    // movement, and right X axis for rotation.
    //CameraServer.startAutomaticCapture();
    
    pcmCompressor.enableDigital();
    //pcmCompressor.disable();


    m_robotDrive.driveCartesian(-0.7 * m_stick.getLeftY(), 0.7 * m_stick.getLeftX(),0.5 * m_stick.getRightX(), gyroscope.getAngle());
    

    /*
    GYROSCOPIO
    Reiniciar el gyroscopio a 0 con el botton "Start"
    */
    if(m_stick.getStartButton())
    {
      gyroscope.reset();
    }
    //TODO: Read Gyro Angle, for debug onlyl!!!!!
    //System.out.println(String.valueOf(gyroscope.getAngle()));
    //_______________________________________________________________

     
    //TODO: Read Trigger, for debug only!!!!!
    //System.out.println(String.valueOf(m_stick.getLeftTriggerAxis()));

      if(m_stick.getRightTriggerAxis() > 0.15  && intakeFlag){

        //TODO: Change power constant (battery sensitive)
       
        indexer.set(1 * m_stick.getRightTriggerAxis());
        
      
      } 
      else if(m_stick.getRightTriggerAxis() > 0.15  && !intakeFlag){

        //TODO: Change power constant (battery sensitive)
        
        indexer.set(0.8 * m_stick.getRightTriggerAxis());
      } 
      else{
        indexer.set(0.0);
      }

    /*
    DISPARADOR
    Bottones: RB y LB
    Dos modos:
    - Disparo largo
    - Disparo corto
    Cuando uno deja de precionar RB entonces los motores se detienen
    Cambiar de modo con LB
     */

    //if para cambiar de modo
    if(m_stick.getXButtonPressed())
    {
      longShot = !longShot;
    }

    if(m_stick.getLeftBumperPressed()){
      eneableShooter = !eneableShooter;

    }
    //if para el disparo largo
    
    if(eneableShooter && longShot)
    {
      shooterRear.set(ControlMode.PercentOutput,-0.9);
      
      shooterFront.set(ControlMode.PercentOutput,-0.9);
    }
    //if para disparo corto
    else if(eneableShooter && !longShot)
    {
      shooterRear.set(ControlMode.PercentOutput,-0.5);
      shooterFront.set(ControlMode.PercentOutput,-0.5);
    }
    else if(!eneableShooter){
      shooterRear.set(ControlMode.PercentOutput,0.0);
      shooterFront.set(ControlMode.PercentOutput,0.0);
    }
    /*
    //if para parar los motores
    if(m_stick.getRightBumperReleased())
    {
      shooterRear.set(ControlMode.PercentOutput,0.0);
      shooterFront.set(ControlMode.PercentOutput,0.0);
    }
*/
    if (m_stick.getYButtonReleased()) {
      intakeSolenoid1.toggle();
      intakeSolenoid2.toggle();
      if (!intakeSolenoid1.get()){
        intake.set(ControlMode.PercentOutput,-0.9);
      }
      else{
        intake.set(ControlMode.PercentOutput,0.0);
    }
      intakeFlag = !intakeFlag;

   }
   if (m_stick.getBButtonReleased()) {
    intakeSolenoid1.toggle();
    intakeSolenoid2.toggle();
    if (!intakeSolenoid1.get()){
      intake.set(ControlMode.PercentOutput,0.9);
    }
    else{
      intake.set(ControlMode.PercentOutput,0.0);
  }
    intakeFlag = !intakeFlag;

 }
   
  }
}
