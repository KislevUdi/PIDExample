package frc.robot.Util;

import java.util.LinkedList;

import Jama.Matrix;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class ChassisDataCollector {

    class data {
        int cycle;
        double[] velocity;
        double[] power;
        double[] acceleration;
    }

    Chassis chassis;
    data[] collect = new data[250];
    int n = 0;

    public ChassisDataCollector(Chassis chassis) {
        this.chassis = chassis;
    }

    public void update() {
        data d = new data();
        d.velocity = new double[]{chassis.getLeftVelocity(),chassis.getRightVelocity()};
        d.power = new double[]{chassis.getLeftPower(),chassis.getRightPower()};
        d.cycle = n;
        collect[n++] = d;
        if(n > 1) {
            d.acceleration = new double[]{(d.velocity[0] - prev(d).velocity[0])/Constants.CycleTime, (d.velocity[1] - prev(d).velocity[1])/Constants.CycleTime};
        } else {
            d.acceleration = new double[]{(d.velocity[0])/Constants.CycleTime, (d.velocity[1])/Constants.CycleTime};
        }
    }

    data prev(data d) {
        return collect[d.cycle-1];
    }
    data prev(data d, int n) {
        return collect[d.cycle-n];
    }
    data next(data d) {
        return collect[d.cycle+1];
    }
    data next(data d, int n) {
        return collect[d.cycle+n];
    }
    data last() {
        if(n > 0) {
            return collect[n-1];
        } else {
            return null;
        }
    }
    data first() {
        if(n > 0) {
            return collect[0];
        } else {
            return null;
        }
    }

    class result {
        double ks;
        double kv;
        double ka;
        double sumErrorrs;
        int nCycles;
    }

    public result calculate() {
        if(n < 50) {
            return null;
        }
        Matrix m = eqMatrix().solve(rMatrix());
        result r = new result();
        r.ks = m.get(0, 0);
        r.kv = m.get(0, 0);
        r.ka = m.get(0, 0);
        r.sumErrorrs = sumErrors(r);
        r.nCycles = n;
        return r;
    }

    double error(int i, result r) {
        data d = collect[i];
        double e0 = Math.abs((r.ks + r.kv * d.velocity[0]  + r.ka*d.acceleration[0])-d.power[0]);
        double e1 = Math.abs((r.ks + r.kv * d.velocity[1]  + r.ka*d.acceleration[1])-d.power[1]);
        return e0 + e1;
    }

    double sumErrors(result r) {
        double s = 0;
        for(int i = 0; i < n; i++) {
            s += error(i, r);
        }
        return s;
    }

    Matrix eqMatrix() {
        // the equation - ks + v*kv + a*ka = p
        // cooef - 1, v, a
        // result p
        Matrix m = new Matrix(2*n-2, 3);
        for(int i = 0; i < n; i++) {
            data d = collect[i];
            m.set(i, 0, 1);
            m.set(i, 1, d.velocity[0]);
            m.set(i, 2, d.acceleration[0]);
            m.set(i+n, 0, 1);
            m.set(i+n, 1, d.velocity[1]);
            m.set(i+n, 2, d.acceleration[1]);
        }
        return m;
    }
    Matrix rMatrix() {
        // the equation - ks + v*kv + a*ka = p
        // cooef - 1, v, a
        // result p
        Matrix m = new Matrix(2*n-2, 1);
        for(int i = 0; i < n; i++) {
            data d = collect[i];
            m.set(i, 0, d.power[0]);
            m.set(i+n, 0, d.power[1]);
        }
        return m;
    }

}
