// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;
import java.time.chrono.ThaiBuddhistChronology;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.auto.DriveFeedforwards;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.LimelightHelpers.RawFiducial;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;




public class Drivetrain extends SubsystemBase {
  public SwerveDrive swerveDrive;
  private RobotConfig config;
  public static final double MAX_SPEED = Units.feetToMeters(14.5); //feet per second
  public static final double PERCISION_MODE = Units.feetToMeters(3.0);
  public static final double PERCISION_MODE_ANGLE = Units.degreesToRadians(90);
  public static final double TURN_MODIFIER = 1.35;

  public static final double wheelRadiusFromCenterRobot = 0.3958; //meters

  public static final double kS = 0.21817;
  public static final double kV = 2.6864;
  public static final double kA = 0.48559;

  public static final double kSr = 0.13917;
  public static final double kVr = 2.7296 * wheelRadiusFromCenterRobot;
  public static final double kAr = 0.49085 * wheelRadiusFromCenterRobot;


  public static final PIDConstants TRANSLATION_PID = new PIDConstants(2.881, 0.0);
  public static final PIDConstants ROTATION_PID = new PIDConstants(2.823, 0.0);

  public static final double LATENCY = 0.1; //bruh idk how to calculate this but it is the time between when the robot gets a command and when it actually executes it, so we will just guess 100 ms for now
  
  public static final double VELOCITY_CONSTANT = Math.exp(-LATENCY * kV/kA);
  public static final double DISPLACEMENT_CONSTANT = kA/kV * (1 - VELOCITY_CONSTANT);

  public static final double VELOCITY_CONSTANT_ROT = Math.exp(-LATENCY * kVr/kAr);
  public static final double DISPLACEMENT_CONSTANT_ROT = kAr/kVr * (1 - VELOCITY_CONSTANT_ROT);

  public static final double FIELD_X = 16.541;
  public static final double FIELD_Y = 8.0693;
  public static final double ROBOT_RADIUS = 0.4; //Variable number

    public static final boolean USE_AUTO_FEEDFORWARD = true;

  StructPublisher<Pose2d> publishercam1Pose = NetworkTableInstance.getDefault().getStructTopic("cam1pose", Pose2d.struct).publish();
  StructPublisher<Pose2d> publishercam2Pose = NetworkTableInstance.getDefault().getStructTopic("cam2pose", Pose2d.struct).publish();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    
    File swerveDir = new File(Filesystem.getDeployDirectory(), "swerve");

    try {
      swerveDrive = new SwerveParser(swerveDir).createSwerveDrive(MAX_SPEED);
    } catch (Exception e) {
      e.printStackTrace();
    }

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    AutoBuilder.configure(
      this::getPose, 
      this::resetPose, 
      this::getRobotVelocity,

        (speeds, feedforward) -> { 
          if (USE_AUTO_FEEDFORWARD) {
            swerveDrive.drive(
              speeds, 
              swerveDrive.kinematics.toSwerveModuleStates(speeds), 
              feedforward.linearForces()
            );
          } else {
            setChassisSpeeds(speeds);
          }
        },
        new PPHolonomicDriveController(
          TRANSLATION_PID, 
          ROTATION_PID
        ),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this
        );
       
        if(swerveDrive != null) {
      swerveDrive.stopOdometryThread();
    }




    if (swerveDrive != null) {
      swerveDrive.stopOdometryThread();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
