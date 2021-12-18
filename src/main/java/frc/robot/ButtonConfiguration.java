package frc.robot;

import commands.drive.SetVoltage;
import commands.drive.SetWheelbaseAngle;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ButtonConfiguration {
    public void initTeleop() {
        new JoystickButton(ControlMap.driverLeft, 1).whenPressed(new SetVoltage(7));
        new JoystickButton(ControlMap.driverLeft, 2).whenPressed(new SetWheelbaseAngle(-90).withTimeout(4));
        new JoystickButton(ControlMap.driverRight, 1).whenPressed(new SetWheelbaseAngle(90).withTimeout(4));
        new JoystickButton(ControlMap.driverRight, 2).whenPressed(new SetWheelbaseAngle(180).withTimeout(4));
    }
}
