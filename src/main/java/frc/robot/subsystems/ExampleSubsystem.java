// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;

public class ExampleSubsystem extends SubsystemBase {

  TalonFX m_frontRight = new TalonFX(8);
  TalonFX m_frontLeft = new TalonFX(6);
  TalonFX m_backRight = new TalonFX(9);
  TalonFX m_backLeft = new TalonFX(7);

  double rightEncoderDistance = m_frontRight.getPosition().getValueAsDouble()*(DriveTrainConstants.kLinearDistanceConversionFactor);
  double leftEncoderDistance = m_frontLeft.getPosition().getValueAsDouble()*(DriveTrainConstants.kLinearDistanceConversionFactor);
  double rightEncoderVelocity = m_frontRight.getPosition().getValueAsDouble()*(DriveTrainConstants.kLinearDistanceConversionFactor/60);
  double leftEncoderVelocity = m_frontLeft.getPosition().getValueAsDouble()*(DriveTrainConstants.kLinearDistanceConversionFactor/60);

  // Create a new SimpleMotorFeedforward with gains kS, kV, and kA
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(5, 1, 1);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.5842);

  public static final AHRS gyro = new AHRS();
  public DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d(),
  m_frontLeft.getPosition().getValueAsDouble(), m_frontRight.getPosition().getValueAsDouble());

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_frontLeft::set, m_frontRight::set);

  SlewRateLimiter slewRate = new SlewRateLimiter(0.5);

  public double getRightEncoderPosition() {
    return rightEncoderDistance;
  }

  public double getRightEncoderVelocity() {
    return rightEncoderVelocity;
  }

  public double getLeftEncoderPosition() {
    return leftEncoderDistance;
  }

  public double getLeftEncoderVelocity() {
    return leftEncoderVelocity;
  }

  public double getAverageEncoderVelocity() {
    return (getLeftEncoderVelocity() + getRightEncoderVelocity())/2;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(gyro.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition(), pose);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return new ChassisSpeeds(gyro.getVelocityX(), 0, Math.PI);
  }

  public ChassisSpeeds chassisSpeeds() {
    return new ChassisSpeeds();
  }

  public DifferentialDriveWheelSpeeds wheelSpeeds() {
    return kinematics.toWheelSpeeds(chassisSpeeds());
  } 

  public void DriveInit() {
    m_frontRight.setInverted(false);
    m_frontLeft.setInverted(true);
    m_backRight.setInverted(false);
    m_backLeft.setInverted(true);
  }

  public void DriveTeleOp() {
    double motorSpeed = RobotContainer.XCont.getLeftY();
    motorSpeed = Deadzone(motorSpeed);
    motorSpeed = slewRate.calculate(motorSpeed);

    double turnSpeed = RobotContainer.XCont.getRightX();
    turnSpeed = Deadzone(turnSpeed)*0.75;
  }

  public double Deadzone(double value) {
    if(Math.abs(value) > 0.075) {
      return value;
    }
    else{
      return 0;
    }
  }

  public void Drive(ChassisSpeeds desiredChassisSpeeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(desiredChassisSpeeds);
  }

  public ExampleSubsystem() {
    AutoBuilder.configureRamsete(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getCurrentSpeeds, // Current ChassisSpeeds supplier
      this::Drive, // Method that will drive the robot given ChassisSpeeds
      new ReplanningConfig(), // Default path replanning config. See the API for the options here
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
);
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
