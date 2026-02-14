// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModulePosition;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.parser.json.ModuleJson;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveSubsystem extends SubsystemBase {

    /** Creates a new ExampleSubsystem. */
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  // File fL = new File(Filesystem.getDeployDirectory(),"frontLeft");
  // File fR = new File(Filesystem.getDeployDirectory(),"frontRight");
  // File bL = new File(Filesystem.getDeployDirectory(),"backLeft");
  // File bR = new File(Filesystem.getDeployDirectory(),"backRight");
  SwerveDrive swerveDrive;
  RobotContainer m_robotContainer;
  
  // SwerveModule[] swerveModules;
    
  
  public SwerveSubsystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
      try
      {
        swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxSpeed, new Pose2d(new Translation2d(Meter.of(1),
                                                                                                                    Meter.of(4)),
                                                                                                                    Rotation2d.fromDegrees(0)));
          
        // Obtain the modules from the parsed SwerveDrive instead of calling a non-existent method on SwerveParser.
        // swerveModules = new SwerveModule[4];
        // Alternative method if you don't want to supply the conversion factor via JSON files.
        // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
      } catch (Exception e)
      {
        throw new RuntimeException(e);
      }
  }
  
    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
      // Inline construction of command goes here.
      // Subsystem::RunOnce implicitly requires `this` subsystem.
      return runOnce(
          () -> {
            /* one-time action goes here */
          });
    }
  
    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
      // Query some boolean state, such as a digital sensor.
      return false;
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putBoolean("Field Oriented", RobotContainer.FieldOriented);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }


  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
    return run(() ->{
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public Command lock() {
    return run( () -> {
      swerveDrive.lockPose();
    });
  }
  public Command strafeLeft() {
    return run( () -> {
      for (SwerveModule module : swerveDrive.getModules()) {
        module.setDesiredState(new SwerveModuleState(0.5, Rotation2d.fromDegrees(90)), false, true);
      }
    });
  }
  public Command strafeRight() {
    return run( () -> {
      for (SwerveModule module : swerveDrive.getModules()) {
        module.setDesiredState(new SwerveModuleState(-0.5, Rotation2d.fromDegrees(90)), false, true);
      }
    });
  }
}
