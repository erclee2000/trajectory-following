package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

public class CreateTrajectory {
    private static Trajectory m_path = new Trajectory();

    public static Trajectory fromPathweaverFile(String pathweaverFilename) {
        try {
            m_path = TrajectoryUtil.fromPathweaverJson(
                Filesystem.getDeployDirectory().toPath().resolve(pathweaverFilename)
            );
        } catch (IOException ex) {
            // DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            System.err.println("Unable to open trajectory: " + pathweaverFilename);//romi doesn't use driverstation
        }
        return m_path;
    }

    public static Trajectory fromCoordinates(
        Pose2d startingPose, 
        List<Translation2d> waypoints, 
        Pose2d endingPose, 
        double maxVoltage) {
            return new Trajectory();
    }
}

        /* 90 degree turn toward increasing y-axis (Romi left-hand turn) */
        // this.path = TrajectoryGenerator.generateTrajectory(
        //     // start at the origin facing the +X direction
        //     new Pose2d(0, 0, new Rotation2d(0)),
        //     // interior waypoints
        //     List.of(
        //         new Translation2d(0.5, 0.25),
        //         new Translation2d(0.75, 0.5)
        //     ),
        //     // end 2 meters straight ahead of where we started, facing forward
        //     new Pose2d(1, 1, new Rotation2d(90)),
        //     // provide our constraints
        //     config);

        /* 90 degree turn toward decreasing y-axis (Romi right-hand turn) */
        // this.path = TrajectoryGenerator.generateTrajectory(
        //     // start at the origin facing the +X direction
        //     new Pose2d(0, 0, new Rotation2d(0)),
        //     // interior waypoints
        //     List.of(
        //         new Translation2d(0.5, -0.25),
        //         new Translation2d(0.75, -0.5)
        //     ),
        //     // end 2 meters straight ahead of where we started, facing forward
        //     new Pose2d(1, -1, new Rotation2d(-90)),
        //     // provide our constraints
        //     config);

        /* s-shape toward increasing y-axis (Romi's left) */
        // this.path = TrajectoryGenerator.generateTrajectory(
        //     // start at the origin facing the +X direction
        //     new Pose2d(0, 0, new Rotation2d(0)),
        //     // interior waypoints
        //     List.of(new Translation2d(0.5, 0.5)),
        //     // end +1 meter on x and +1 meter on y, facing forward
        //     new Pose2d(1, 1, new Rotation2d(0)),
        //     // provide our constraints
        //     config);
