package frc.robot;

import commands.auto.SimpleAuto;
import commands.drive.SetWheelbaseAngle;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ButtonConfiguration {
    public void initTeleop() {
        new JoystickButton(JoystickConfiguration.driverLeft, 1).whenPressed(new SetWheelbaseAngle(90));
        new JoystickButton(JoystickConfiguration.driverLeft, 2).whenPressed(new SetWheelbaseAngle(-90));
        new JoystickButton(JoystickConfiguration.driverRight, 1).whenPressed(new SetWheelbaseAngle(0));
    }
}
