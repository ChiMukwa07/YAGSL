package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.util.function.DoubleSupplier;



import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
//import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

//import com.ctre.phoenix6.configs.CANcoderConfiguration;
//import com.ctre.phoenix6.hardware.CANcoder;
//import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
//import com.studica.frc.AHRS.NavXComType;




public class SwerveSubsystem extends SubsystemBase {
    
  
    // Attributes
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry   odometry;
    AHRS                  gyro; // NavX gyroscope instance
    SwerveModule[]        swerveModules; // Psuedo-class representing swerve modules.

  private final SwerveDrive swerveDrive;
  /** Creates a new ExampleSubsystem. */

  /**
   * Example command factory method.
   *
   * @return a command
   */
// public class SwerveModule {

//     private SparkMax driveMotor;
//     private SparkMax steerMotor;
//     private CANcoder    absoluteEncoder;
    
//     public SwerveModule(int driveMotorCANID, int steerMotorCANID, int cancoderCANID)
//     {
//         driveMotor = new SparkMax(driveMotorCANID, SparkMax.MotorType.kBrushless);
//         steerMotor = new SparkMax(steerMotorCANID, SparkMax.MotorType.kBrushless);
//         absoluteEncoder = new CANcoder(cancoderCANID);
        
//         // Reset everything to factory default
//         driveMotor.restoreFactoryDefaults();
//         steerMotor.restoreFactoryDefaults();
//         absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());
        
        // Continue configuration here..
        
//     }

// }


  public SwerveSubsystem() {
    


        swerveModules = new SwerveModule[4]; // Psuedo-code; Create swerve modules here.
        
        // Create SwerveDriveKinematics object
        // 12.5in from center of robot to center of wheel.
        // 12.5in is converted to meters to work with object.
        // Translation2d(x,y) == Translation2d(front, left)
        kinematics = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(12.5)), // Front Left
            new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(-12.5)), // Front Right
            new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(12.5)), // Back Left
            new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(-12.5)));  // Back Right

        AHRS NavX = new AHRS(null); // Psuedo-code for creating a NavX gyroscope.
        gyro = NavX; // use AHRS directly as the gyro

        // Create the SwerveDriveOdometry given the current angle, the robot is at x=0, r=0, and heading=0
        odometry = new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(gyro.getAngle()), // convert NavX yaw (degrees) to Rotation2d
            new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},
            // Front-Left, Front-Right, Back-Left, Back-Right
            new Pose2d(0,0,new Rotation2d())); // x=0, y=0, heading=0



double maximumSpeed = Units.feetToMeters(4.5);
File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
try {
    this.swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
} catch (java.io.IOException e) {
    throw new java.io.UncheckedIOException("Failed to parse swerve configuration", e);
}
}

    public void drive()
    {
        // Create test ChassisSpeeds going X = 14in, Y=4in, and spins at 30deg per second.
        ChassisSpeeds testSpeeds = new ChassisSpeeds(Units.inchesToMeters(14), Units.inchesToMeters(4), Units.degreesToRadians(30));
        
        // Drive the robot using the swerveDrive instance instead of directly setting module states
        swerveDrive.drive(new Translation2d(testSpeeds.vxMetersPerSecond, testSpeeds.vyMetersPerSecond),
                          testSpeeds.omegaRadiansPerSecond,
                          true,
                          false);
    }

    public SwerveModulePosition[] getCurrentSwerveModulePositions()
    {
        // If the SwerveModule class does not expose distance/angle accessors, return default positions.
        // Replace with actual module readings when appropriate accessors are available.
        return new SwerveModulePosition[]{
            new SwerveModulePosition(), // Front-Left
            new SwerveModulePosition(), // Front-Right
            new SwerveModulePosition(), // Back-Left
            new SwerveModulePosition()  // Back-Right
        };
    }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }

  private void driveFieldOriented(ChassisSpeeds targetSpeeds) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'driveFieldOriented'");
}
/**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
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
            // Update the odometry every run.
    odometry.update(Rotation2d.fromDegrees(gyro.getAngle()),  getCurrentSwerveModulePositions());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
public SwerveDrive getSwerveSubsystem() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getSwerveSubsystem'");
}
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }
}
