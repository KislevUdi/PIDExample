package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import static frc.robot.Constants.ChassisConstants.*;


public class VelocityPIDCommand extends CommandBase {
    Chassis chassis;

    public VelocityPIDCommand(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        double v = SmartDashboard.getNumber(AutoVelocityID, 1);
        chassis.setPID();
        chassis.setVelocity(v);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }
    
}
