package frc.robot.Util;

public class UdiFeedForward {

    double kv;
    double ks;
    double ka;

    public UdiFeedForward(double ks, double kv, double ka) {
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
    }

    public double calculate(double v, double a) {
        return ks + v*kv + a*ka;
    }
    
}
