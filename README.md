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
2. Delete everything except Drivetrain.java, RomiGyro.java, RobotContainer.java, Robot.java, Main.java (can keep ArcadeDrive.java for convience of using the controller)

C. Modify Bare Bones Romi to Track Ramsete Command Example Project
--------------------------------------------------------------
1. Modify RomiGyro.java to include methods of WPILib's Gyro.java class
- add a `getAngle()` method which just returns `getAngleZ()`
- direction should be correct but might want to verify on robot (The angle is expected to increase as the gyro turns clockwise when looked at from the top. It needs to follow the NED axis convention.)
    - add a `public Rotation2d getRotation2d()` :
```  
    public Rotation2d getRotation2d() {
       return Rotation2d.fromDegrees(-getAngle());
    }
```
- direction should be correct but might want to verify on robot (The angle is expected to increase as the gyro turns counterclockwise when looked at from the top. It needs to follow the NWU axis convention.)

2. Modify DriveTrain.java so it has comparable methods as the DriveSubsytem.java of the Ramsete Command project

3. Modify RobotContainer.java
