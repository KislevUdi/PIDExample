package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.DistanceTrapezoid;
import frc.robot.subsystems.Chassis;

public class DriveToPosition extends CommandBase {

    static final double AngleKP = -0.5;
    Pose2d tgtPose;
    Chassis chassis;
    double remainingDistance = 0;
    DistanceTrapezoid trapeziod;

    public DriveToPosition(Pose2d pose, double maxVelcotiy, double maxAccelration, Chassis chassis) {
        super();
        tgtPose = new Pose2d(pose.getTranslation(),pose.getRotation()); // copy so data will not change
        this.chassis = chassis;
        trapeziod = new DistanceTrapezoid(maxVelcotiy, maxAccelration, 0.01);
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        Pose2d pose = chassis.getPose();
        Translation2d vec = tgtPose.getTranslation().minus(pose.getTranslation());
        remainingDistance = vec.getNorm();
        double velocity = trapeziod.calculate(remainingDistance, chassis.getVelocity(), 0);
        double angleError = vec.getAngle().minus(pose.getRotation()).getDegrees();
        ChassisSpeeds chassisSpeed = new ChassisSpeeds(velocity, 0, angleError*AngleKP);
        chassis.setVelocity(chassisSpeed);
    }

    @Override
    public boolean isFinished() {
        return remainingDistance < 0.05;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }


    
}
