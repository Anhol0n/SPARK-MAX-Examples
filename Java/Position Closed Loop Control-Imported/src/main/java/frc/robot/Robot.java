/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;



public class Robot extends TimedRobot {
  private static final int deviceID = 1;
  private SparkMax m_motor;
  private SparkMaxConfig config = new SparkMaxConfig();
  private SparkClosedLoopController m_pidController;
  private RelativeEncoder m_encoder;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  @Override
  public void robotInit() {
    // initialize motor
    m_motor = new SparkMax(deviceID, MotorType.kBrushless);

    /**
     * The restoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
  

    /**
     * In order to use PID functionality for a controller, a SparkPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_motor.getClosedLoopController();

    // Encoder object created to display position values
    m_encoder = m_motor.getEncoder();

    // PID coefficients
    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pidf(kP, kI, kD, kFF)
    .iZone(kIz)
    .outputRange(kMinOutput, kMaxOutput);
              

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
  }

  @Override
  public void teleopPeriodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { config.closedLoop.p(p); kP = p; }
    if((i != kI)) { config.closedLoop.i(i); kI = i; }
    if((d != kD)) { config.closedLoop.d(d); kD = d; }
    if((iz != kIz)) { config.closedLoop.iZone(iz); kIz = iz; }
    if((ff != kFF)) { config.closedLoop.velocityFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      config.closedLoop.outputRange(min,max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    m_pidController.setReference(rotations, SparkMax.ControlType.kPosition);
    
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
  }
}
