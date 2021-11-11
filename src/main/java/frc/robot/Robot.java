package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import subsystems.DriveTrain;


public class Robot extends TimedRobot
{
    @Override
    public void robotInit() {
        initializeAllSubsystems();
        new ButtonConfiguration().initTeleop();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic()
    {
    }

    private void initializeAllSubsystems() {
        DriveTrain.getInstance();
    }
}
