package frc.robot.Util;

import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;

public class ExtendedDifferentialDriveFeedForward extends DifferentialDriveFeedforward {

    private double ks;
    private double minPower;
    private double zeroVelocity = 0.04;
    public ExtendedDifferentialDriveFeedForward(double ks, double kv, double ka, double kav, double kaa, double minPower, double TrackWidth) {
        super(kv, ka, kav, kaa, TrackWidth);
        this.minPower = minPower;
        this.ks = ks;
    }

    @Override
    public DifferentialDriveWheelVoltages calculate(double currentLeftVelocity, double nextLeftVelocity,
            double currentRightVelocity, double nextRightVelocity, double dtSeconds) {
        DifferentialDriveWheelVoltages result = super.calculate(currentLeftVelocity, nextLeftVelocity, currentRightVelocity, nextRightVelocity, dtSeconds);
        result.left += ks*Math.signum(nextLeftVelocity);
        result.right += ks*Math.signum(nextRightVelocity);
        // check min power - next velocity is not zero, the calculated power is too small and we are very slow now
        if(nextLeftVelocity != 0 && Math.abs(result.left) < minPower && Math.abs(currentLeftVelocity) < zeroVelocity) {
            result.left = minPower*Math.signum(nextLeftVelocity);
        }
        if(nextRightVelocity != 0 && Math.abs(result.right) < minPower && Math.abs(currentRightVelocity) < zeroVelocity) {
            result.right = minPower*Math.signum(nextRightVelocity);
        }
        return result;
    }
    
}
