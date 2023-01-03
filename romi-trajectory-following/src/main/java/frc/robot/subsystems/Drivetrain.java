// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.CreateTrajectory;
import frc.robot.Constants.DriveConstants;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  private final RomiGyro m_gyro = new RomiGyro();
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  private final Field2d m_field = new Field2d(); // to output robot's actual path to Glass


  // Odometry class to track the position of a differential drive robot on the field
  // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/intro-and-chassis-speeds.html#what-is-odometry
  // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-odometry.html
  private final DifferentialDriveOdometry m_odometry;

  public Drivetrain() {
    m_rightMotor.setInverted(true);

    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    resetEncoders();

    /**
     * could also include robot's (x, y) position (meters) on the field like this: 
     * m_odometry = new DifferentialDriveOdometry(getGyroHeading(), new Pose2d(5.0, 10.0, new Rotation2d()));
     */
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    
    SmartDashboard.putData("Field", m_field);
    m_field.getObject("traj").setTrajectory(CreateTrajectory.fromPathweaverFile(DriveConstants.PATHWEAVERFILE));
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(rightVolts);
    m_diffDrive.feed();
  }
  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   * 
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_diffDrive.setMaxOutput(maxOutput);
  }

  /* start encoder methods */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }
  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }
  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }
  /* end enconder methods */

  /* start gyro methods */
  public void resetGyro() {
    m_gyro.reset();
  }

  // acceleration of the Romi along the axes in Gs
  public double getAccelX() {
    return m_accelerometer.getX();
  }
  public double getAccelY() {
    return m_accelerometer.getY();
  }
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  // angle of the Romi in degrees
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }
  // Zeroes the heading of the robot.
  public void zeroHeading() {
    m_gyro.reset();
  }
  // Returns the heading of the robot in degrees, from -180 to 180
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }
  // the turn rate of the robot, in degrees per second
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
  /* end gyro methods */


  /* start odometry methods */
   // Returns the currently-estimated pose of the robot.
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  // Resets the odometry to the specified pose.
  public void resetOdometry(Pose2d pose) {
    m_gyro.reset();
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }
  @Override
  public void periodic() {
    // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-odometry.html#updating-the-robot-pose
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    m_field.setRobotPose(m_odometry.getPoseMeters());//to show the robot's actual path in Glass
    SmartDashboard.putNumber("robot's x", m_odometry.getPoseMeters().getX());//robot's x in field coordinates (for debugging)
    SmartDashboard.putNumber("robot's y", m_odometry.getPoseMeters().getY());//robot's y in field coordinates (for debugging)
  }
  /* end odometry methods */
}
