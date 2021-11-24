package frc.robot;

import commands.drive.SwerveControl;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.rivierarobotics.lib.shuffleboard.RSTable;
import org.rivierarobotics.lib.shuffleboard.RSTileOptions;
import subsystems.DriveTrain;
import util.AIFieldDisplay;
import util.FieldMesh;
import util.ML.MLCore;

import java.io.IOException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;


public class Robot extends TimedRobot {

    private RSTable table;

    @Override
    public void robotInit() {
        Logging.initialize();
        if (Robot.isReal()) {
            initializeAllSubsystems();
            setDefaultCommands();
        }
    }

    @Override
    public void robotPeriodic() {
        //DISABLE WHEN NOT IN USE - forces NWT to update at the same loop rate as the rio, extremely laggy but useful for
        //tuning state-space
        NetworkTableInstance.getDefault().flush();

        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        new ButtonConfiguration().initTeleop();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {
    }

    private void setDefaultCommands() {
        CommandScheduler.getInstance().setDefaultCommand(DriveTrain.getInstance(), new SwerveControl());
    }

    private void initializeAllSubsystems() {
        DriveTrain.getInstance();
        try {
            FieldMesh.getInstance();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    FieldMesh m;
    AIFieldDisplay display;
    ExecutorService executor;

    @Override
    public void simulationInit() {
        executor = Executors.newFixedThreadPool(1);
        var tab = Logging.robotShuffleboard.getTab("ML");
        table = new RSTable("Objects", tab, new RSTileOptions(2, 4, 2, 2));
        try {
            m = FieldMesh.getInstance();
            display = new AIFieldDisplay(m, 300);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    double time = Timer.getFPGATimestamp();
    double ttPath = Timer.getFPGATimestamp();
    public static Trajectory trajectory;

    @Override
    public void simulationPeriodic() {
        if (Timer.getFPGATimestamp() - ttPath > 0.2) {
            Thread th = new Thread(() -> {
                double startx = 0;
                double endx = m.fieldWidth * Math.random();
                double starty = 0;
                double endy = m.fieldHeight * Math.random();
                var generatedTrajectory = m.getTrajectory(startx / 100.0, starty / 100.0, endx / 100.0, endy / 100.0, true, 0);
                if (generatedTrajectory != null) display.updatePath(generatedTrajectory);
            });
            executor.execute(th);
            ttPath = Timer.getFPGATimestamp();
        }

        if (Timer.getFPGATimestamp() - time > 1) {
            var obj = MLCore.getInstance().getDetectedObjects();
            int ndex = 1;
            for (var key : obj.keySet()) {
                for (var ob : obj.get(key)) {
                    table.setEntry(ob.label + " " + ndex, "confidence: " + ob.confidence + ", tx: " + ob.tx + ", ty: " + ob.ty);
                    ndex++;
                }
            }

            Logging.robotShuffleboard.getTab("ML").setEntry("Num Objects", ndex);
            time = Timer.getFPGATimestamp();
        }
    }
}
