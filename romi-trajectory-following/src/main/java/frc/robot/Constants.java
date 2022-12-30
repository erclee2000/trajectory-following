package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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
