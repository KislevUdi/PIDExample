package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.DistanceTrapezoid;
import frc.robot.subsystems.Chassis;

public class DistVelDirExerciseCommand extends CommandBase {

    private Chassis chassis;
    DistanceTrapezoid trapezoid;
    private double distance;
    private double direction;
    private double heading = 0;
    private double startDistance = 0;
    private double remainingDistance = 0;

    private final double headingKP = 0.1;

    public DistVelDirExerciseCommand(double distance, double velocity, double maxAcceleration, Chassis chassis) {
        super();
        this.chassis = chassis;
        this.distance = distance;
        direction = Math.signum(distance);
        trapezoid = new DistanceTrapezoid(velocity, maxAcceleration,0.1);
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        heading = chassis.heading();
        startDistance = chassis.getDistance();
        SmartDashboard.putNumber("Start Heading", heading);
    }

    @Override
    public void execute() {
        double curentVelocity = chassis.getVelocity();
        double headingError = chassis.heading() - heading;
        remainingDistance = remainingDistance();
        double tgtVel = trapezoid.calculate(remainingDistance, curentVelocity, 0) * direction;
        chassis.setVelocity(tgtVel + headingError * headingKP, tgtVel - headingError * headingKP);
    }

    private double remainingDistance() {
        return direction*(distance + startDistance - chassis.getDistance());
    }

    @Override
    public boolean isFinished() {
        return remainingDistance < 0.02;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
        SmartDashboard.putNumber("End Heading", chassis.heading());
        SmartDashboard.putNumber("Error Heading", chassis.heading() - heading);
        SmartDashboard.putNumber("End Distance", distance - remainingDistance);

    }
}
