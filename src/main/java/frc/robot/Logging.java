package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import org.rivierarobotics.lib.shuffleboard.RobotShuffleboard;

public class Logging {
    public static NetworkTableInstance networkTableInstance;
    public static RobotShuffleboard robotShuffleboard;

    public static void initialize() {
        networkTableInstance = NetworkTableInstance.getDefault();
        robotShuffleboard = new RobotShuffleboard();
    }
}
