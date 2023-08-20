package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ChassisConstants.*;

import frc.robot.RobotContainer;
import frc.robot.Util.TalonFXGroup;

public class Chassis extends SubsystemBase {

    TalonFXGroup left; 
    TalonFXGroup right;
    boolean brake;
    RobotContainer container; // for future use

    public Chassis(RobotContainer container) {
        super();
        this.container = container;
        left = initMotors(LeftFrontMotor, LeftBackMotor, LeftInverted);
        right = initMotors(RightFrontMotor, RightBackMotor, RightInverted);
        setCoast();
        setPID(VelocityKP, VelocityKI, VelocityKD);
        SmartDashboard.putData("Chassis", this);
    }

    // Init motors for one side
    private TalonFXGroup initMotors(int main, int follower, boolean invert) {
        TalonFXGroup m = new TalonFXGroup(main, follower);
        m.setInverted(invert);
        return m;
    }

    public void setBrake() {
        brake = true;
        left.setNeutralMode(NeutralMode.Brake);
        right.setNeutralMode(NeutralMode.Brake);
    }

    public void setCoast() {
        brake = false;
        left.setNeutralMode(NeutralMode.Coast);
        right.setNeutralMode(NeutralMode.Coast);
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
        setPower(0, 0);
    }

    private void setPID(double kp, double ki, double kd) {
        setPIDF(kp, ki, kd, 0);
    }
    private void setPIDF(double kp, double ki, double kd, double kf) {
        left.configPIDF(kp, ki, kd, kf);
        right.configPIDF(kp, ki, kd, kf);
    }

    public void setPID() { // read PID from network table
        setPID(SmartDashboard.getNumber("Velocity KP", VelocityKP),
                SmartDashboard.getNumber("Velocity KI", VelocityKI),
                SmartDashboard.getNumber("Velocity KD", VelocityKD));
    }

    // get functions
    public double getLeftDistance() {
        return left.getSelectedSensorPosition() / PulsePerMeter;
    }

    public double getRightDistance() {
        return right.getSelectedSensorPosition() / PulsePerMeter;
    }

    public double getDistance() {
        return (getLeftDistance() + getRightDistance()) / 2;
    }

    public double getLeftVelocity() {
        return TalonVelocityToVelocity(left.getSelectedSensorVelocity());
    }

    public double getRightVelocity() {
        return TalonVelocityToVelocity(right.getSelectedSensorVelocity());
    }

    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2;
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
        SmartDashboard.putData("Brake", new InstantCommand(() -> setBrake(), this).ignoringDisable(true));
        SmartDashboard.putData("Coast", new InstantCommand(() -> setCoast(), this).ignoringDisable(true));
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
        if (SmartDashboard.getNumber(name, -1) == -1) {
            SmartDashboard.putNumber(name, def);
        }
    }

}
