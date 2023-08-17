package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ChassisConstants.*;

import java.util.ArrayList;

import frc.robot.RobotContainer;

public class Chassis extends SubsystemBase {

    TalonFX left;  // shortcut for leftMotors[0]
    TalonFX right;
    ArrayList<TalonFX> motors = new ArrayList<>(4);
    boolean brake;
    RobotContainer container; // for future use

    public Chassis(RobotContainer container) {
        super();
        this.container = container;
        left = initMotors(LeftFrontMotor, LeftBackMotor, LeftInverted);
        right = initMotors(RightFrontMotor, RightBackMotor, RightInverted);
        setCoast();
        SmartDashboard.putData("Chassis", this);
    }

    // Init motors for one side
    private TalonFX initMotors(int main, int follower, boolean invert) {
        TalonFX m = new TalonFX(main);
        TalonFX f = new TalonFX(follower);
        motors.add(m);
        motors.add(f);
        m.setInverted(invert);
        f.setInverted(invert);
        f.follow(m);
        setPID(m, VelocityKP, VelocityKI,VelocityKD);
        return m;
    }

    public void setBrake() {
        brake = true;
        for(TalonFX motor: motors) {
            motor.setNeutralMode(NeutralMode.Brake);
        }
    }

    public void setCoast() {
        brake = false;
        for(TalonFX motor: motors) {
            motor.setNeutralMode(NeutralMode.Coast);
        }
    }

    public void setPower(double l, double r) {
        left.set(ControlMode.PercentOutput, l);
        right.set(ControlMode.PercentOutput, r);
    }

    public void setVelocity(double l, double r) {
        // input in meter per seconds
        left.setIntegralAccumulator(0);
        right.setIntegralAccumulator(0);
        left.set(ControlMode.Velocity, VelocityToTalonVelocity(l));
        right.set(ControlMode.Velocity, VelocityToTalonVelocity(r));
    }
    public void setVelocity(double v) {
        setVelocity(v, v);
    }
    
    public void stop() {
        setPower(0,0);
    }

    private void setPID(TalonFX motor,double kp, double ki, double kd ) {
        motor.config_kP(0, kp);
        motor.config_kI(0, ki);
        motor.config_kD(0, kd);
    }

    public void setPID(double kp, double ki, double kd) {
        setPID(left, kp, ki, kd);
        setPID(right, kp, ki, kd);
    }

    public void setPID() { // read PID from network table
        setPID(SmartDashboard.getNumber("Velocity KP", VelocityKP),
                SmartDashboard.getNumber("Velocity KI", VelocityKI),
                SmartDashboard.getNumber("Velocity KD", VelocityKD));
    }

    // get functions
    public double getLeftDistance() {
        return left.getSelectedSensorPosition()/PulsePerMeter;
    }
    public double getRightDistance() {
        return right.getSelectedSensorPosition()/PulsePerMeter;
    }
    public double getDistance() {
        return (getLeftDistance() + getRightDistance())/2;
    }
    public double getLeftVelocity() {
        return TalonVelocityToVelocity(left.getSelectedSensorVelocity());
    }
    public double getRightVelocity() {
        return TalonVelocityToVelocity(right.getSelectedSensorVelocity());
    }
    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity())/2;
    }
    public boolean brakeMode() {
        return brake;
    }

    public double getLeftPower() {
        return left.getMotorOutputPercent();
    }
    public double getRightPower() {
        return right.getMotorOutputPercent();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Left Distance", this::getLeftDistance, null);
        builder.addDoubleProperty("Right Distance", this::getRightDistance, null);
        builder.addDoubleProperty("Distance", this::getDistance, null);
        builder.addDoubleProperty("Left Velocity", this::getLeftVelocity, null);
        builder.addDoubleProperty("Right Velocity", this::getRightVelocity, null);
        builder.addDoubleProperty("Velocity", this::getLeftVelocity, null);
        builder.addBooleanProperty("Brake", this::brakeMode, null);
        SmartDashboard.putNumber("Velocity KP", VelocityKP);
        SmartDashboard.putNumber("Velocity KD", VelocityKD);
        SmartDashboard.putNumber("Velocity KI", VelocityKI);
        SmartDashboard.putData("Brake", new InstantCommand(()->setBrake(),this).ignoringDisable(true));
        SmartDashboard.putData("Coast", new InstantCommand(()->setCoast(),this).ignoringDisable(true));
        addNTField(AutoVelocityID, 1);
    }


    // utilities
    public static double TalonVelocityToVelocity(double v) {
        return v * 10 / PulsePerMeter;
    }
    
    public static double VelocityToTalonVelocity(double v) {
        return v * PulsePerMeter / 10;
    }

    // add network table field
    private void addNTField(String name, double def) {
        if(SmartDashboard.getNumber(name, -1) == -1) {
            SmartDashboard.putNumber(name, def);
        }
    }


}
