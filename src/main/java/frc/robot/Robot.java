package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import subsystems.DriveTrain;
import util.AIFieldDisplay;
import util.FieldMesh;

import java.io.IOException;


public class Robot extends TimedRobot
{
    @Override
    public void robotInit() {
        //initializeAllSubsystems();
        //new ButtonConfiguration().initTeleop();
        Thread th = null;
        try {
            FieldMesh m = new FieldMesh();
            m.addWeightToArea(1000, 600, 500, 1000, 1100);
            var imgTst = new AIFieldDisplay(m, 100);
            th = new Thread(imgTst);
        } catch (IOException e) {
            e.printStackTrace();
        }
        th.start();
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
