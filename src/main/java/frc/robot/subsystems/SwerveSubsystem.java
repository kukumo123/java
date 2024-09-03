// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.DriveConstants;

/**
 * Basic simulation of a swerve subsystem with the methods needed by PathPlanner
 */
public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule[] modules = new SwerveModule[4];
  SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;

  private AHRS gyro;
  
  private Field2d field = new Field2d();

  private final CANcoder FLcancoder = new CANcoder(9, "rio");
  private final CANcoder BLcancoder = new CANcoder(10, "rio");
  private final CANcoder FRcancoder = new CANcoder(11, "rio");
  private final CANcoder BRcancoder = new CANcoder(12, "rio");
  
  public SwerveSubsystem() {
     gyro = new AHRS(SPI.Port.kMXP);
     new Thread(() -> {
      try {
          Thread.sleep(1000);
          zeroHeading();
      } catch (Exception e) {
      }
  }).start();
modules[0] = new SwerveModule(
                DriveConstants.kFrontLeftDriveMotorPort,
                DriveConstants.kFrontLeftTurningMotorPort,
                DriveConstants.kFrontLeftDriveReversed,
                DriveConstants.kFrontLeftTurningReversed,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
                DriveConstants.kFrontLeftAbsoluteEncoderOffsetRad,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

        modules[1] = new SwerveModule(
                DriveConstants.kFrontRightDriveMotorPort,
                DriveConstants.kFrontRightTurningMotorPort,
                DriveConstants.kFrontRightDriveReversed,
                DriveConstants.kFrontRightTurningReversed,
                DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
                DriveConstants.kFrontRightAbsoluteEncoderOffsetRad,
                DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

        modules[2] = new SwerveModule(
                DriveConstants.kBackLeftDriveMotorPort,
                DriveConstants.kBackLeftTurningMotorPort,
                DriveConstants.kBackLeftDriveReversed,
                DriveConstants.kBackLeftTurningReversed,
                DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
                DriveConstants.kBackLeftAbsoluteEncoderOffsetRad,
                DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

        modules[3] = new SwerveModule(
                DriveConstants.kBackRightDriveMotorPort,
                DriveConstants.kBackRightTurningMotorPort,
                DriveConstants.kBackRightDriveReversed,
                DriveConstants.kBackRightTurningReversed,
                DriveConstants.kBackRightDriveAbsoluteEncoderPort,
                DriveConstants.kBackRightAbsoluteEncoderOffsetRad,
                DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
    kinematics = new SwerveDriveKinematics(
      Constants.Swerve.flModuleOffset, 
      Constants.Swerve.frModuleOffset, 
      Constants.Swerve.blModuleOffset, 
      Constants.Swerve.brModuleOffset
    );
    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());

    // Configure AutoBuilder
    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetPose, 
      this::getSpeeds, 
      this::driveRobotRelative, 
      Constants.Swerve.pathFollowerConfig,
      () -> {

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      this
    );

    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);
  }

  public void zeroHeading() {
    if (gyro.isConnected()) {
        gyro.reset();
    }
}

public double getHeading() {
  return Math.IEEEremainder(gyro.getAngle(), 360);
}

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), getPositions());

    field.setRobotPose(getPose());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.Swerve.maxModuleSpeed);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setTargetState(targetStates[i]);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public void resetEncoder(){
    modules[0].resetEncoders();
    modules[1].resetEncoders();
    modules[2].resetEncoders();
    modules[3].resetEncoders();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public SwerveModuleState[] setModuleStates(SwerveModuleState[] desiredStates) {
    // 重新規範（標準化）萬向輪驅動模組速度
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
  
    // 設定馬達速度
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
        optimizedSetpointStates[i] = modules[i].setDesiredState(desiredStates[i]);
    }
    return optimizedSetpointStates;
  }

  public void getTurningEncoderPosition(){
    SmartDashboard.putNumber("FL", FLcancoder.getAbsolutePosition().getValue()*(Math.PI*2));
    SmartDashboard.putNumber("BL", BLcancoder.getAbsolutePosition().getValue()*(Math.PI*2));
    SmartDashboard.putNumber("FR", FRcancoder.getAbsolutePosition().getValue()*(Math.PI*2));
    SmartDashboard.putNumber("BR", BRcancoder.getAbsolutePosition().getValue()*(Math.PI*2));
}

public void stopModules() {
  for (int i = 0; i < 4; i++) {
      modules[i].stop();
  }
}
}
