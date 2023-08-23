package frc.robot.Util;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class TalonFXGroup extends TalonFX {
    TalonFX[] followers;

    public TalonFXGroup(int ...id) {
        super(id[0]);
        followers = new TalonFX[id.length - 1];
        for(int i = 1; i < id.length; i++) {
            followers[i-1] = new TalonFX(id[i]);
            followers[i-1].follow(this);
        }
    }

    @Override
    public void setInverted(InvertType invertType) {
        super.setInverted(invertType);
        for(TalonFX t: followers) {
            t.setInverted(invertType);
        }
    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
        super.setNeutralMode(neutralMode);
        for(TalonFX t: followers) {
            t.setNeutralMode(neutralMode);
        }
    }

    public void configPIDF(double kp, double ki, double kd, double kf) {
        config_kP(0, kp);
        config_kI(0, ki);
        config_kD(0, kd);
        config_kF(0, kf);
    }

}
