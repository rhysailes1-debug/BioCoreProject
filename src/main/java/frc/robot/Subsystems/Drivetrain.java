// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import static edu.wpi.first.units.Units.Volts;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
  public static final double PRECISION_MODE = Units.feetToMeters(3.0);
  public static final double PRECISION_MODE_ANGLE = Units.degreesToRadians(90);
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

  private boolean rejectCamera1 = false;
  private boolean rejectCamera2 = false;


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
      this::getRobotVelocity ,

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

    if (swerveDrive != null) {
      swerveDrive.stopOdometryThread();
    }

    SmartDashboard.putBoolean("MT2", false);
    SmartDashboard.putBoolean("FilterCamera", true);
  }
  private static Drivetrain instance;
  public static Drivetrain getInstance() {
    if (instance == null) {
      try {
        Thread.sleep(50);
      } catch (Exception e) {
        e.printStackTrace();
      }
      instance = new Drivetrain();
    }
    return instance;
  }

  public void drive(Translation2d t, double rot, boolean isFieldRel) {
    swerveDrive.drive(t, rot, isFieldRel, false);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    swerveDrive.drive(speeds);
  }
  public void createSysidCommands() {

    Command sysidDriveWithSpin = SwerveDriveTest.generateSysIdCommand(
      SwerveDriveTest.setDriveSysIdRoutine(
        new SysIdRoutine.Config(), 
        this, 
        swerveDrive, 
        6,
        true
      ), 
      2.0, 
      10.0, 
      15.0
    );

    Command sysidDriveWithoutSpin = SwerveDriveTest.generateSysIdCommand(
      SwerveDriveTest.setDriveSysIdRoutine(
        new SysIdRoutine.Config(), 
        this, 
        swerveDrive, 
        6,
        false
      ), 
      2.0, 
      5.0, 
      2.5
    );

    Command sysidAngle = SwerveDriveTest.generateSysIdCommand(
      SwerveDriveTest.setAngleSysIdRoutine(
        new SysIdRoutine.Config(), 
        this, 
        swerveDrive
      ),
      2.0, 
      15.0, 
      20.0
    );
    // objects to store logged values in. Prevents creating new instances each iteration of logging
    MutVoltage logVolts = new MutVoltage(0, 0, Volts);
    MutDistance logAngle = new MutDistance(0, 0, Meters);
    MutLinearVelocity logAngleVel = new MutLinearVelocity(0, 0, MetersPerSecond);
    
    // Assuming all 4 swerve modules are equidistant from the robot origin and just grab 1 of them here
    double radiusMeters = swerveDrive.getModules()[0].configuration.moduleLocation.getNorm();
    
    // SysID Mechanism takes a voltage and applies it to the motors and logs the state of the "mechanism" as a result of that applied voltage
    SysIdRoutine.Mechanism spinRobotMechanism = new SysIdRoutine.Mechanism(
      (Voltage voltage) -> {
        SwerveDriveTest.setModulesToRotaryPosition(swerveDrive); // Modules set to point tangent to robot center
        SwerveDriveTest.powerDriveMotorsVoltage(swerveDrive, voltage.in(Volts)); // Apply requested power along tangent vector
      },
      (SysIdRoutineLog log) -> {
        for (swervelib.SwerveModule module : swerveDrive.getModules())
        {
          // For each motor, we manually estimate the rotation of the robot using distance / radius = radians
          log.motor("spin-" + module.configuration.name)
          .voltage(logVolts.mut_replace(module.getDriveMotor().getVoltage(), Volts))
          .linearPosition(logAngle.mut_replace(module.getDriveMotor().getPosition(), Meters))
          .linearVelocity(logAngleVel.mut_replace(module.getDriveMotor().getVelocity(), MetersPerSecond));
        }
      }, Drivetrain.getInstance());

    // SysIdRoutine combines the config and mechanism
    SysIdRoutine spinRobotRoutine = new SysIdRoutine(new SysIdRoutine.Config(), spinRobotMechanism);

    // Make it a command we can assign to a button
    Command sysidSpinRobot = SwerveDriveTest.generateSysIdCommand(
      spinRobotRoutine, 
      2.0, 
      10.0, 
      15.0
    );

    sysidSpinRobot.addRequirements(this);
  }

  public void resetPose(Pose2d resetPose) {
    swerveDrive.resetOdometry(resetPose);
  }

  public void resetPose() {
    resetPose(
      new Pose2d(
        0, 
        0, 
        new Rotation2d(0))
    );
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void rejectCameraChange(boolean change){
    rejectCamera1 = change;
    rejectCamera2 = change;
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public static Pose2d getInputSpeeds(DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega, boolean precision) {

    double xVelo = vX.getAsDouble();
    double yVelo = vY.getAsDouble();
    double aVelo = omega.getAsDouble();

    xVelo = Math.pow(xVelo, 2) * Math.signum(xVelo) * Drivetrain.MAX_SPEED;
    yVelo = Math.pow(yVelo, 2) * Math.signum(yVelo) * Drivetrain.MAX_SPEED;
    aVelo = Math.pow(aVelo, 2) * Math.signum(aVelo) * Math.PI * Drivetrain.TURN_MODIFIER;

    if (precision) {
      if (xVelo > Drivetrain.PRECISION_MODE) {
        xVelo = Drivetrain.PRECISION_MODE;
      }
      else if (xVelo < -Drivetrain.PRECISION_MODE) {
        xVelo = -Drivetrain.PRECISION_MODE;
      }
      if (yVelo > Drivetrain.PRECISION_MODE) {
        yVelo = Drivetrain.PRECISION_MODE;
      }
      else if (yVelo < -Drivetrain.PRECISION_MODE) {
        yVelo = -Drivetrain.PRECISION_MODE;
      }
      if(aVelo > Drivetrain.PRECISION_MODE_ANGLE) {
        aVelo = Drivetrain.PRECISION_MODE_ANGLE;
      } else if (aVelo < -Drivetrain.PRECISION_MODE_ANGLE) {
        aVelo = -Drivetrain.PRECISION_MODE_ANGLE;
      }
    }
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false) {
      yVelo *= -1;
      xVelo *= -1;
    }
    return new Pose2d(xVelo, yVelo, new Rotation2d(aVelo));
  }

  public double getMatchTime() {
    double robotTime = 140 - Robot.robotTimer.get();
    double matchTime = DriverStation.getMatchTime();
    
    SmartDashboard.putNumber("MatchTime", Math.abs(matchTime - robotTime) < 2 ? matchTime : robotTime);
    return (Math.abs(matchTime - robotTime) < 2 ? matchTime : robotTime);
  }

  public void idlemode(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public int latestTagId = -1;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(4, 8,4));
    UpdateCamera1();
    UpdateCamera2();
    

    Pose2d robotPose = getPose();
      if(Math.min(robotPose.getX(), robotPose.getY()) < ROBOT_RADIUS || robotPose.getX() > FIELD_X - ROBOT_RADIUS || robotPose.getY() > FIELD_Y - ROBOT_RADIUS) {
        double x = MathUtil.clamp(robotPose.getX(), ROBOT_RADIUS, FIELD_X - ROBOT_RADIUS);
        double y = MathUtil.clamp(robotPose.getY(), ROBOT_RADIUS, FIELD_Y - ROBOT_RADIUS);
        resetPose(new Pose2d(x, y, robotPose.getRotation()));
      }
    }

    public void UpdateCamera1() {
      boolean hasTarget1 = LimelightHelpers.getTV("cam1");

      LimelightHelpers.PoseEstimate mt11 = LimelightHelpers.getBotPoseEstimate_wpiBlue("cam1");
      if(mt11 == null) {
        return;
      }
      boolean doRejectUpdate1 = false;
      publishercam1Pose.set(mt11.pose);
      double min_distance1 = Double.POSITIVE_INFINITY;
      RawFiducial bestFid1 = null;
      int tagCount = 0;
      for(RawFiducial fid1 : mt11.rawFiducials) {
        if (fid1.id > 0) {
          tagCount++;
        }
        if(fid1.distToCamera >= min_distance1) {
          continue;
        }
        bestFid1 = fid1;
        min_distance1 = fid1.distToCamera;
      }

      boolean filter = mt11.pose.getTranslation().getSquaredDistance(getPose().getTranslation()) > 1.0;
      
      if(bestFid1 != null && (filter || Math.min(tagCount, mt11.tagCount) == 1)) 
      { 
        if(bestFid1.ambiguity > 0.7)
        {
        doRejectUpdate1 = true;
        if(bestFid1.distToCamera > 3.0)
        doRejectUpdate1 = true;
      }
    }
    if(mt11.tagCount == 0) {
      doRejectUpdate1 = true;
    }
    if(!doRejectUpdate1 && !rejectCamera1) {
      swerveDrive.addVisionMeasurement(
        mt11.pose, 
        mt11.timestampSeconds
      );
    }
    if(doRejectUpdate1 || rejectCamera1) {
      rejectCamera1 = false;
    }
    SmartDashboard.putBoolean("ATVisibleCam1", hasTarget1);
  }
    public void UpdateCamera2() {

      boolean hasTarget2 = LimelightHelpers.getTV("cam2");

      LimelightHelpers.PoseEstimate mt12 = LimelightHelpers.getBotPoseEstimate_wpiBlue("cam2");
      if(mt12 == null) {
        return;
      }
      boolean doRejectUpdate2 = false;
      publishercam2Pose.set(mt12.pose);
      double min_distance2 = Double.POSITIVE_INFINITY;
      RawFiducial bestFid2 = null;
      int tagCount = 0;
      for(RawFiducial fid2 : mt12.rawFiducials) {
        if (fid2.id > 0) {
          tagCount++;
        }
        if(fid2.distToCamera >= min_distance2) {
          continue;
        }
        bestFid2 = fid2;
        min_distance2 = fid2.distToCamera;
      }

      boolean filter = mt12.pose.getTranslation().getSquaredDistance(getPose().getTranslation()) > 1.0;

      
      if(bestFid2 != null && (filter || Math.min(tagCount, mt12.tagCount) == 1)) 
      { 
        if(bestFid2.ambiguity > 0.7)
        {
        doRejectUpdate2 = true;
        if(bestFid2.distToCamera > 3.0)
        doRejectUpdate2 = true;
      }
    }
    if(mt12.tagCount == 0) {
      doRejectUpdate2 = true;
    }
  }
}
