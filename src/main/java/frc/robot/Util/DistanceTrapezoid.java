package frc.robot.Util;

import frc.robot.Constants;

/*
 * Calculate required velocity based on remaining distance current velocity and target velocity
 */
public class DistanceTrapezoid {

    double maxVelocity;
    double maxAcceleration;
    private double accelDist; // 1/2 * a * t * t
    private double deltaV; // max velocity change in 1 cycle at max acceleration

    public DistanceTrapezoid(double maxVelocity, double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        deltaV = maxAcceleration * Constants.CycleTime;
        accelDist = maxAcceleration * Constants.CycleTime * Constants.CycleTime / 2;

    }

    public double calculate(double remainingDistance, double curentVelocity, double tgtVelocity) {
        if(curentVelocity < maxVelocity && distanceToVel(curentVelocity+deltaV, tgtVelocity, maxAcceleration) > remainingDistance - cycleDistanceWithAccel(curentVelocity)) {
            // can accelerate - velocity not at max and remaining distance allow deacceleration
            return Math.min(curentVelocity + deltaV, maxVelocity);

        } else if(distanceToVel(curentVelocity, tgtVelocity, maxAcceleration) > remainingDistance - cycleDistanceNoAccel(curentVelocity)) {
            // mainintining velocity
            return curentVelocity;
        } else {
            // deccelerate
            return curentVelocity - deltaV;
        }
        
    }

    private double cycleDistanceWithAccel(double currentVelocity) {
        return currentVelocity * Constants.CycleTime + accelDist;
    }
    private double cycleDistanceNoAccel(double currentVelocity) {
        return currentVelocity * Constants.CycleTime;
    }

    private double distanceToVel(double currentVel, double tgtVel, double accel) {
        double deltaVel = currentVel - tgtVel;
        return (currentVel - deltaVel/2)*deltaVel/accel;
    }

}
