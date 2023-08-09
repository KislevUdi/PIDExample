package frc.robot.Util;

public class VelocityAnalyzer {

    public static final double goodPercent = 5;
    double signum;
    double targetVelocity;
    double minGoodVelocity;
    double maxGoodVelocity;
    int nCycles;
    int firstGoodVelocityCycle = -1;
    int sumInvalidCycles = 0;
    int nOscilation = 0;
    double maxError = 0;
    double sumErrors = 0;
    boolean lastGood = false;
    double maxVelocity = 0;
    double rank = 0;

    public VelocityAnalyzer(double target) {
        signum = Math.signum(target);
        targetVelocity = target;
        double d = target * goodPercent / 100;
        minGoodVelocity = target*signum - d;
        maxGoodVelocity = target*signum + d;
        nCycles = 0;
    }

    public void update(double vel) {
        double v = vel*signum;
        nCycles ++;
        if(v > maxVelocity) {
            maxVelocity = v;
        }
        if(firstGoodVelocityCycle < 0) {
            if(v > minGoodVelocity) {
                firstGoodVelocityCycle = nCycles;
                lastGood = true;
            }
        }
        if(firstGoodVelocityCycle > 0) {
            if(v > maxGoodVelocity || v < minGoodVelocity) {
                double error = Math.abs(v - targetVelocity*signum);
                sumErrors += error;
                if(error > maxError) {
                    maxError = error;
                }
                sumInvalidCycles++;
                if(lastGood) {
                    lastGood = false;
                    nOscilation ++;
                }
            } else {
                lastGood = true;
            }
        }
    }

    public void analyze() {
        double t = targetVelocity * signum;
        if(firstGoodVelocityCycle < 0) {
            // never reached
            rank = (t - maxVelocity)/t;
        } else {
            rank = (nCycles - firstGoodVelocityCycle)/nCycles;
            rank += 1-nOscilation/10;
            rank += (t/maxVelocity);
            rank += 1 - (sumErrors/sumInvalidCycles)/t;
            rank += 1 - (sumInvalidCycles + firstGoodVelocityCycle)/nCycles;
        }

    }

    @Override
    public String toString() {
        if(rank == 0) {
            analyze();
        }
        return String.format("rank=%f target=%f maxv=%f first=%d osc=%d maxE=%f sumE=%f",rank,targetVelocity,maxVelocity,firstGoodVelocityCycle,nOscilation,maxError,sumErrors);
    }

    public void publishResult(double kp, double ki, double kd) {
        if(rank == 0) {
            analyze();
        }
        System.out.printf("Result for kp=%f, ki=%f, kd=%f, targhet=%f\n", kp, ki, kd, targetVelocity);
        if(firstGoodVelocityCycle > 0) {
            System.out.printf("   rank=%f\n",rank);
            System.out.printf("   first good after %f seconds\n",firstGoodVelocityCycle/50);
            System.out.printf("   total error time %f seconds\n",sumInvalidCycles/50);
            System.out.printf("   %d oscilations\n",nOscilation);
            System.out.printf("   maxError = %f\n",maxError);
            System.out.printf("   sumError = %f\n",sumErrors);
        } else {
            System.out.printf("   Never Reached Target - max was %f\n",maxVelocity);
        }
    }

    
}
