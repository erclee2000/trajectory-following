A. "Characterize" the Romi
-----------------------
1. Forked https://github.com/bb-frc-workshops/romi-examples/tree/main/romi-characterization-sysid
2. Used romi-characterization-sysid project with WPILib's SysID tool ctl+shift+p -> start tool -> sysid
3. In SysID changed 
    - "Analysis Type" to Romi
    - "Unit Type" to meters 
    - wheel circumference  0.2199  
    - dynamic velocity -- reduce from 7.00 to 4.00
4. Ran each of the four test by starting test, using "auton" from the romi-characterization-sysid project 
5. Saved data to .json file (sysid_data_meters_and_velo_4.0.json)
6. Opened .json in SysID and verified that data looked good https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/analyzing-data.html


B. Create "Bare Bones" Romi Reference project in VSCode 
-----------------------------------------
1. ctl+shift+p -> Create New Project -> Romi Reference
2. Delete everything except Constants.java, Drivetrain.java, RomiGyro.java, RobotContainer.java, Robot.java, Main.java (can keep ArcadeDrive.java for convience of using the controller)

C. Add Values from SysID to Constants.java
1. for general instructions see: https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/entering-constants.html

2. here are my values:
```java
public final class Constants {
    public static final class DriveConstants{
        public static final double kTrackwidthMeters = 0.14; //distance between wheels in meters
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
    
        public static final int kEncoderCPR = 1440; //romi encoder is 12 per rev but gear ratio is 1:120
        public static final double kWheelDiameterMeters = 0.07;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
    
        // copy/paste values from SysID tool using the .json created by characterizing romi
        public static final double ksVolts = 0.46435;
        public static final double kvVoltSecondsPerMeter = 10.134;
        public static final double kaVoltSecondsSquaredPerMeter = 0.94359;
        // to get proper velocity, make sure you are using "WPILib (2020-)" in sysid
        public static final double kPDriveVel = 12.278;
    }
}
```
3. NOTE: for `kPDriveVel` make sure you are using "WPILib (2020-)" setting in the SysID tool (the Ramsete Command project (see below) uses WPILib PIDController for the velocity loop)

D. Modify Bare Bones Romi to Have Functionality of the Ramsete Command Example Project
--------------------------------------------------------------
1. Modify RomiGyro.java to include methods of WPILib's Gyro.java class
- add a `getAngle()` which just returns `getAngleZ()`
    - angle is expected to increase as the gyro turns clockwise when looked at from the top. NED axis convention.
- add `getRotation2d()` (see WPILib Gyro class)
    - angle is expected to increase as the gyro turns counterclockwise when looked at from the top. NWU axis convention.
- add `getRate()` which just returns `getRateZ()`
- my final version of RomiGyro.java is in the project

2. Modify DriveTrain.java so it has all methods of the DriveSubsytem.java of the Ramsete Command example project
- basically this involves adding some odometry methods (`getPose()` and `resetOdometry`) and some encoder getter methods and gryo getter methods
- also involves adding `tankDriveVolts()` which drives the Romi by voltage instead of the usual 0.0 to 1.0 speed values
- override `periodic()` so that it updates the odometry 
- my final version of Drivetrain.java is in the project

3. Modify RobotContainer.java
