// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class RobotContainer {
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final PS4Controller m_controller = new PS4Controller(0);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());
  }

  public Command getAutonomousCommand() {
    /** 
     * PART ONE: create feedfoward controller
     * Feedforward controller handles known settings to make a system track a reference 
     * Feedback controller compensates unknown runtime behvaior
     * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html
     */
    SimpleMotorFeedforward feedforwardController = new SimpleMotorFeedforward(
      DriveConstants.ksVolts,
      DriveConstants.kvVoltSecondsPerMeter,
      DriveConstants.kaVoltSecondsSquaredPerMeter
    );//ks, kv, ka for feedforward as determined by charactrerizing robot w/ SysID tool
       
    /**
     * PART TWO: create and set constraints for trajectory following
     */
    // the config for trajectory stores max velocity, max acceleration, and custom constraints
    TrajectoryConfig config = new TrajectoryConfig(
      DriveConstants.kMaxSpeedMetersPerSecond, //i think this is chassis velocity
      DriveConstants.kMaxAccelerationMetersPerSecondSquared //i think this is chassis accel
    );
    // adding differential drive kinematics constraint to ensure that no wheel velocity goes above max velocity
    config.setKinematics(DriveConstants.kDriveKinematics);
    // adding a voltage constraint to the TrajectoryConfig in two steps
    // step one: create the voltage constraint to ensure we don't accelerate too fast
    DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      feedforwardController, 
      DriveConstants.kDriveKinematics, 
      6 //volts
    );
    // step two: add the constraint
    config.addConstraint(autoVoltageConstraint);

    /**
     * PART THREE: create a trajectory for robot to follow (all units in meters)
     * I created a separate class to do this but you could put code here
     */
    Trajectory pathToFollow = new CreateTrajectory(config, "zigzag.wpilib.json").getPath();

    /** PART FOUR: create a RamseteCommand 
     * 
     */
    // create PID Controllers for left and right side of robot that will be adjusted by Ramsete Command
    PIDController leftPIDcontroller = new PIDController(0.0, 0.0, 0.0);//kp, kd, ki will be adjusted by Ramsete Command
    PIDController rightPIDcontroller = new PIDController(0.0, 0.0, 0.0);//kp, kd, ki will be adjusted by Ramsete Command

    // create a ramsete feedBACK controller that uses global pose. PID controllers only deals with the local pose. 
    // https://file.tavsys.net/control/controls-engineering-in-frc.pdf
    RamseteController ramseteController = new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta);

    RamseteCommand ramseteCommand = new RamseteCommand(
        pathToFollow,
        m_drivetrain::getPose,
        ramseteController,
        feedforwardController,
        DriveConstants.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        leftPIDcontroller,
        rightPIDcontroller,
        m_drivetrain::tankDriveVolts,
        m_drivetrain
    );

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrain.resetOdometry(pathToFollow.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drivetrain.tankDriveVolts(0, 0));
  }

  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(m_drivetrain, () -> -m_controller.getLeftY(), () -> m_controller.getLeftX());
  }
}
