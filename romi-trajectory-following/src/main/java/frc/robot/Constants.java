package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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

        public static final double kMaxSpeedMetersPerSecond = 0.4;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.4;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
