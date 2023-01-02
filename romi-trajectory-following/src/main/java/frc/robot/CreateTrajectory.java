package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.DriveConstants;

public class CreateTrajectory {
    private static Trajectory m_path;

    /**
     * creates a trajectory from a pathweaver generated file
     * @param pathweaverFilename the filename of the pathweaver generated "build path" file
     */
    public static Trajectory fromPathweaverFile(String pathweaverFilename) {
        try {

            m_path = TrajectoryUtil.fromPathweaverJson(
                Filesystem.getDeployDirectory().toPath().resolve(pathweaverFilename)
            );//TrajectoryUtil.fromPathweaverJson returns a new trajectory

        } catch (IOException ex) {
            // DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            System.err.println("Unable to open trajectory: " + pathweaverFilename);//romi doesn't use driverstation
        }
        return m_path;
    }

    /** 
     * creates a trajectory from user inputs (not a pathweaver file)
     * @param startingPose the starting position, e.g., new Pose2d(0, 0, new Rotation2d(0)) for origin, facing pos x
     * @param waypoints a List of internal (x,y) waypoints, e.g., List.of(new Translation2d(0.5, 0.25), new Translation2d(0.75, 0.5)),
     * @param endingPose the ending position, e.g., new Pose2d(1, 1, new Rotation2d(0)) for x = 1, y = 1, facing pos x
     */
    public static Trajectory fromCoordinates(
        Pose2d startingPose, 
        List<Translation2d> waypoints, 
        Pose2d endingPose, 
        double maxVoltage) {

        //create a config for the trajectory which stores max velocity, max acceleration, and custom constraints
        TrajectoryConfig config = new TrajectoryConfig(
            DriveConstants.kMaxSpeedMetersPerSecond, // i think this is chassis velocity
            DriveConstants.kMaxAccelerationMetersPerSecondSquared // i think this is chassis accel
        );
        // ensuring that no wheel velocity goes above max chassis velocity
        config.setKinematics(DriveConstants.kDriveKinematics);
        // adding a voltage constraint to the TrajectoryConfig 
        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter), // ks, kv, ka for feedforward as determined by SysID tool
            DriveConstants.kDriveKinematics,
            maxVoltage
        );
        config.addConstraint(autoVoltageConstraint);

        m_path = TrajectoryGenerator.generateTrajectory(
            startingPose,
            waypoints,
            endingPose,            
            config);

        return m_path;
    }
}

/* EXAMPLES for manually creating trajectories with start, waypoints, end */

/* 90 degree turn toward increasing y-axis (Romi left-hand turn) */
// this.path = TrajectoryGenerator.generateTrajectory(
// new Pose2d(0, 0, new Rotation2d(0)),
// List.of(
// new Translation2d(0.5, 0.25),
// new Translation2d(0.75, 0.5)
// ),
// new Pose2d(1, 1, new Rotation2d(90)),
// config);

/* 90 degree turn toward decreasing y-axis (Romi right-hand turn) */
// this.path = TrajectoryGenerator.generateTrajectory(
// new Pose2d(0, 0, new Rotation2d(0)),
// List.of(
// new Translation2d(0.5, -0.25),
// new Translation2d(0.75, -0.5)
// ),
// new Pose2d(1, -1, new Rotation2d(-90)),
// config);

/* s-shape toward increasing y-axis (Romi's left) */
// this.path = TrajectoryGenerator.generateTrajectory(
// new Pose2d(0, 0, new Rotation2d(0)),
// List.of(new Translation2d(0.5, 0.5)),
// new Pose2d(1, 1, new Rotation2d(0)),
// config);
