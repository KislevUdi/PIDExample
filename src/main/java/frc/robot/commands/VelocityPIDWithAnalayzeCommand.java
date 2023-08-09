package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.VelocityAnalyzer;
import frc.robot.subsystems.Chassis;
import static frc.robot.Constants.ChassisConstants.*;


public class VelocityPIDWithAnalayzeCommand extends CommandBase {
    Chassis chassis;
    VelocityAnalyzer va = null;

    public VelocityPIDWithAnalayzeCommand(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        double v = SmartDashboard.getNumber(AutoVelocityID, 1);
        chassis.setVelocity(v, v);
        va = new VelocityAnalyzer(v);
    }

    @Override
    public void execute() {
        va.update(chassis.getVelocity());
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
        va.publishResult(VelocityKP, VelocityKI, VelocityKD);
    }
    
}
