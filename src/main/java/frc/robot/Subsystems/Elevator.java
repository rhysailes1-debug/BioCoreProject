// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private TalonFX rightElevatorMotor;
  private TalonFX leftElevatorMotor;

  private static final int RIGHT_ELEVATOR_MOTOR_ID = 9;
  private static final int LEFT_ELEVATOR_MOTOR_ID = 10;
  
  public boolean homed = false;

  public static final double MANUAL_SPEED = 0.5;
  public static final double CONVERSION_FACTOR = 1.0 / 3.0 * Math.PI * 2.0; 
  public static final double SCALING_FACTOR = 1.15;
  public static final double OFFSET = 0.05;
  public static final double GRAVITY_COMPENSATION = 0.1;

  //height
  public static final double INTAKE_HEIGHT = 0.1; //intaking and low goal
  public static final double HIGH_GOAL_HEIGHT = 0.5 + OFFSET; //high goal

  private final VoltageOut motorVolts = new VoltageOut(0.0);

  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  private static Elevator instance; 
  /**
   * Makes and/or returns the singleton instance of the {@link Elevator}. 
   * @return the {@link Elevator} instance. 
   */
  public static Elevator GetInstance() {
    // if instance doesn't exist, make a new one.
    if (instance == null) {
      try {
        // safety delay to make sure that the robot is fully up-and-running
        Thread.sleep(50);
      } catch (Exception e) {
        e.printStackTrace();
      }
      
      instance = new Elevator();
    }
    // return instance
    return instance;
  }

  /** Creates a new Elevator. */
  public Elevator() {
    leftElevatorMotor = new TalonFX(LEFT_ELEVATOR_MOTOR_ID);
    leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    rightElevatorMotor = new TalonFX(RIGHT_ELEVATOR_MOTOR_ID);
    rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    
    var cfgl = leftElevatorMotor.getConfigurator();
    var cfgr = rightElevatorMotor.getConfigurator();

    // PIDS
    var slot0 = new Slot0Configs();

    slot0.kS = 0.2;
    slot0.kV = 0.1;
    slot0.kA = 0.01;
    slot0.kP = 0.5;
    slot0.kI = 0.0;
    slot0.kD = 0.0;

    slot0.GravityType = GravityTypeValue.Elevator_Static;

    slot0.kG = 0.67676767676767676767;

    var motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 80;
    motionMagicConfigs.MotionMagicAcceleration = 120;
    motionMagicConfigs.MotionMagicJerk = 1600;

    cfgl.apply(slot0);
    cfgr.apply(slot0);
    cfgl.apply(motionMagicConfigs);
    cfgr.apply(motionMagicConfigs);

    MotorOutputConfigs outCfg = new MotorOutputConfigs();
    outCfg.NeutralMode = NeutralModeValue.Brake;
    outCfg.Inverted = InvertedValue.CounterClockwise_Positive;

    cfgl.apply(outCfg);

    CurrentLimitsConfigs currentCfg = new CurrentLimitsConfigs().withSupplyCurrentLimit(100).withSupplyCurrentLowerLimit(100);

    currentCfg.SupplyCurrentLimitEnable = false;

    cfgl.apply(currentCfg);
    cfgr.apply(currentCfg);

    rightElevatorMotor.setControl(new Follower(LEFT_ELEVATOR_MOTOR_ID, MotorAlignmentValue.Aligned));
  }

  public void setSpeed(double speed) {
    leftElevatorMotor.set(speed);
  }

  public void setHeight(double distance) {
    double target = (distance * SCALING_FACTOR);
    double rots = target / CONVERSION_FACTOR;

    leftElevatorMotor.setControl(m_request.withPosition(rots));
  }

  public double getHeight(){
    double pos = leftElevatorMotor.getPosition().getValueAsDouble() * CONVERSION_FACTOR;
    pos /= SCALING_FACTOR;
    return pos; 
  }

  public boolean isZeroed() {
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
