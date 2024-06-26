package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class DriveFair extends DriveCommandBase {

  private final DriveSubsystem driveSubsystem;

  private final DoubleSupplier rightJoystickX;
  private final BooleanSupplier isFieldRelative, isHighRotation;
  private double angularSpeed;

  // private SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.Y_RATE_LIMIT);
  // private SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.X_RATE_LIMIT);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.ROT_RATE_LIMIT);
  
  /**
   * The command for driving the robot using joystick inputs.
   * @param driveSubsystem The subsystem for the swerve drive
   * @param visionSubsystem The subsystem for vision measurements
   * @param leftJoystickY The joystick input for driving forward and backwards
   * @param leftJoystickX The joystick input for driving left and right
   * @param rightJoystickX The joystick input for turning
   * @param isFieldRelative The boolean supplier if the robot should drive
   * field relative
   * @param isHighRotation The boolean supplier for if the robot should drive with a higher rotation
   */
  public DriveFair(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,DoubleSupplier rightJoystickX, BooleanSupplier isFieldRelative, BooleanSupplier isHighRotation) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem, visionSubsystem);
    // this.leftJoystickY = leftJoystickY;
    // this.leftJoystickX = leftJoystickX;
    this.rightJoystickX = rightJoystickX;
    this.isFieldRelative = isFieldRelative;
    this.isHighRotation = isHighRotation;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Drives the robot
    if (isHighRotation.getAsBoolean()) {
      angularSpeed = DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
    } else {
      angularSpeed = DriveConstants.LOW_ANGULAR_SPEED_RADIANS_PER_SECOND;
    }
    // Rotation2d rot = driveSubsystem.getPose().getRotation();
    // if (angularSpeed < -0.1 && rot.getDegrees() > 90) {
    //   angularSpeed = 0;
    // } else if (angularSpeed > 0.1 && rot.getDegrees() < -90) {
    //   angularSpeed = 0;
    // }
    
    driveSubsystem.drive(
      0,//yLimiter.calculate(leftJoystickY.getAsDouble() * DriveConstants.MAX_SPEED_METERS_PER_SECOND / 4.0)
      0, //xLimiter.calculate(leftJoystickX.getAsDouble() * DriveConstants.MAX_SPEED_METERS_PER_SECOND / 4.0)
      rotLimiter.calculate(rightJoystickX.getAsDouble() * angularSpeed * 0.25),
      isFieldRelative.getAsBoolean()
    );

    // Runs all the code from DriveCommandBase that estimates pose
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    // When the command ends, it stops the robot
    driveSubsystem.drive(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
