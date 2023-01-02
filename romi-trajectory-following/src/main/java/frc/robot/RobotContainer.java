// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
       
    /*
     * step one: create a trajectory for robot to follow 
     * I wrote a helper class called CreateTrajectory but 
     * also fine to just write the trajectory gen code here.
     * 
     * option 1: use pathweaver generated file (easier and visual)
     * option 2: write your own trajectory using x,y field coordinates (more precise, maybe smoother)
     */
    //option 1: from pathweaver gen file
    Trajectory pathToFollow = CreateTrajectory.fromPathweaverFile("circle clockwise.wpilib.json");

    // option 2: create a trajectory using x, y coordinates
    // Trajectory pathToFollow = CreateTrajectory.fromCoordinates(
    //   new Pose2d(0, 0, new Rotation2d(0)), // start pose in meters 
    //   List.of(
    //     new Translation2d(0.375, 0.125), //field x,y coordinates in meters
    //     new Translation2d(0.625, 0.875) //field x,y coordinates in meters
    //   ), // waypoints
    //   new Pose2d(1, 1, new Rotation2d(0)), // end pose in meters
    //   7.0 //max voltage
    // );

    /**
     * step two: create feedforward and feedback controllers
     */    
    // create feedforward controller for known robot settings gathered from robot characterization
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html
    SimpleMotorFeedforward feedforwardController = new SimpleMotorFeedforward(
      DriveConstants.ksVolts,
      DriveConstants.kvVoltSecondsPerMeter,
      DriveConstants.kaVoltSecondsSquaredPerMeter
    );//ks, kv, ka for feedforward as determined by charactrerizing robot w/ SysID tool
    
    //ramsete feedforward controller https://file.tavsys.net/control/controls-engineering-in-frc.pdf
    RamseteController ramseteController = new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta);

    // create PID Controllers for feedback gathered from runtime
    PIDController leftPIDcontroller = new PIDController(DriveConstants.kPDriveVel, 0.0, 0.0);// using kp from SysID
    PIDController rightPIDcontroller = new PIDController(DriveConstants.kPDriveVel, 0.0, 0.0);// using kp from SysID
    
    /**
     * step three: create a ramsete command
     */
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
