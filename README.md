## Trajectory Following/PathWeaver with Romi

Caveats
1. I am not an expert programmer and do not code for a living
2. I relied on other projects.
- overall I looked at Daltz3's project here https://www.chiefdelphi.com/t/from-romi-path-following-to-frc-robot/393857
- To get the Romi to work with SysID (for characterizing the robot) I used this project https://github.com/bb-frc-workshops/romi-examples/tree/main/romi-characterization-sysid
- there was another project I took a quick look at just to see if my characterization constants were reasonable and it also gave me the idea of using the Romi Reference example project and converting that project to the Ramsete Command example code.
3. I read (and sometimes reread) the FRC Docs--they are quite helpful.

Step One. "Characterized" the Romi
-----------------------
1. Forked https://github.com/bb-frc-workshops/romi-examples/tree/main/romi-characterization-sysid
2. Used romi-characterization-sysid project with WPILib's SysID tool ctl+shift+p -> start tool -> sysid
3. In SysID changed 
    - "Analysis Type" to Romi
    - "Unit Type" to meters 
    - wheel circumference  0.2199  
    - dynamic velocity -- reduced from 7.00 to 4.00
4. Ran each of the four test by starting test, using "auton" from the romi-characterization-sysid project 
5. Saved data to .json file (sysid_data_meters_and_velo_4.0.json)
6. Opened .json in SysID and verified that data looked good https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/analyzing-data.html


Step Two. Created "Bare Bones" Romi Reference project in VSCode 
-----------------------------------------
1. ctl+shift+p -> Create New Project -> Romi Reference
2. Deleted everything except Main.java, Robot.java, RobotContainer.java, Constants.java, Drivetrain.java, RomiGyro.java  (can keep ArcadeDrive.java for the convience of using the controller to reset the Romi during testing)

Step Three. Added Values from SysID to Constants.java
------------------------------------------
1. for general instructions see: https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/entering-constants.html

2. here are my values:
```java
public final class Constants {
    public static final class DriveConstants{
        public static final double kTrackwidthMeters = 0.14; //distance between wheels in meters
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters); //for converting chassis velocity to wheel velocity
    
        public static final int kEncoderCPR = 1440; //romi encoder is 12 per rev but gear ratio is 1:120
        public static final double kWheelDiameterMeters = 0.07;
        public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR; //meters/per pulse
    
        // copy/paste values from SysID tool using the .json created by characterizing romi
        public static final double ksVolts = 0.46435; //Ks (feedforward)
        public static final double kvVoltSecondsPerMeter = 10.134; //Kv (feedforward)
        public static final double kaVoltSecondsSquaredPerMeter = 0.94359; //Ka (feedforward)
        // make sure you are using "WPILib (2020-)" in sysid
        public static final double kPDriveVel = 12.278; //Kp (feedback)

        public static final double kMaxSpeedMetersPerSecond = 0.6;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.4;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
```
3. NOTE: for `kPDriveVel` make sure you are using "WPILib (2020-)" setting in the SysID tool (the Ramsete Command project (see below) uses WPILib PIDController for the velocity loop)

Step Four. Modified Bare Bones Romi to Have Functionality of the Ramsete Command Example Project
--------------------------------------------------------------
1. Modified RomiGyro.java to include methods of WPILib's Gyro.java class
- added a `getAngle()` (which just returned `getAngleZ()`)
    - angle is expected to increase as the gyro turns clockwise when looked at from the top. NED axis convention.
- added `getRotation2d()` (see WPILib Gyro class)
    - angle is expected to increase as the gyro turns counterclockwise when looked at from the top. NWU axis convention.
- my final version of RomiGyro.java is in the project

2. Modified DriveTrain.java so it had all methods of the DriveSubsytem.java of the Ramsete Command example project
- basically this involves adding some odometry methods (`getPose()` and `resetOdometry()`) and some encoder getter methods and gryo getter methods
- also involves adding `tankDriveVolts()` which drives the Romi by voltage instead of the usual 0.0 to 1.0 speed values
- override `periodic()` so that it updates the odometry
    - note you may want to output the x, y coordinates of the Romi onto SmartDashboard for debugging. Like this: 

```java
    SmartDashboard.putNumber("robot's x", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("robot's y", m_odometry.getPoseMeters().getY());
```

- my final version of Drivetrain.java is in the project

3. Modified Romi's RobotContainer.java to Match Ramsete Controller's RobotContainer.java
- I basically copied the Ramsete Controller's `getAutonomousCommand()` 
- I detailed the steps in my final version of RobotContainer.java in the project
- NOTE: I rordered a lot of the code in Ramsete Controller's `getAutonomousCommand()` for my `getAutonomousCommand()` to make it more readable (imo). This included creating a helper class for trajectory generation (so you aren't writing the trajectory creation code in the command which is much better (imo) if you are manually creating a trajectory and not using pathweaver file)

Step Five. Created Paths using PathWeaver
--------------------------------------------------------------
1. I basically followed the FRC tutorial on PathWeaver here https://docs.wpilib.org/en/stable/docs/software/pathplanning/pathweaver/index.html
    NOTE: path should use small distances for Romi--pathweaver shows the whole field which is huge for Romi
2. I set max acceleration to 0.4 and max velocity to 0.4
3. After creating a reasonable (simple) path, and then doing "build paths" in pathweaver, I copied the generated json to java/frc/robot/deploy (e.g., "circle clockwise.wpilib.json")
4. I updated Constants file to include a constant for the file name of the generated json (I used the filename in two places in the code)
5. You can also output the planned trajectory and the actual robot position into the Sim GUI or Glass. You can do this by adding code to Drivetrain.java. Basically I followed the instructions here with little to no changes: https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/field2d-widget.html

```java
  private final Field2d m_field = new Field2d(); // to output robot's actual path to Glass

  public Drivetrain() {
      . . .
    SmartDashboard.putData("Field", m_field);
    m_field.getObject("traj").setTrajectory(CreateTrajectory.fromPathweaverFile(DriveConstants.PATHWEAVERFILE));
  }
    . . .
  @Override
  public void periodic() {
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    m_field.setRobotPose(m_odometry.getPoseMeters());//to show the robot's actual path in Glass
```
6. My paths are in the project in java/frc/robot/deploy