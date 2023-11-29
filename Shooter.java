package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  /*CANSparkMax leftShooter = new CANSparkMax(ShooterConstants.leftShoother_ID, MotorType.kBrushless);
  CANSparkMax rightShooter = new CANSparkMax(ShooterConstants.rightShoother_ID, MotorType.kBrushless);*/
  WPI_VictorSPX leftShooter = new WPI_VictorSPX(ShooterConstants.leftShoother_ID);
  WPI_VictorSPX rightShooter = new WPI_VictorSPX(ShooterConstants.rightShoother_ID);
  
  PIDController leftController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, ShooterConstants.kFF);
  PIDController rightController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, ShooterConstants.kFF);
  

  
  

  public Shooter() {
    leftShooter.setInverted(false);
    rightShooter.setInverted(true);

    leftShooter.setNeutralMode(NeutralMode.Brake);
    rightShooter.setNeutralMode(NeutralMode.Brake);

  }

  @Override
  public void periodic() {

  }

  public void setMotorsPower(double leftPower, double rightPower) {
    leftShooter.set(leftPower);
    rightShooter.set(rightPower);
  }

  /*public double currentLeftShooterSpeed() {
    return leftShooterEncoder.getVelocity();
  }

  public double currentRightShooterSpeed() {
    return rightShooterEncoder.getVelocity();
  }*/
 

  public void leftSetPoint(double setPoint) {
    leftController.setSetpoint(setPoint);
    //leftController.setReference(setPoint, ControlType.kVelocity);
  }

  public void rightSetPoint(double setPoint) {
    leftController.setSetpoint(setPoint);
    //rightController.setReference(setPoint, ControlType.kVelocity);
  }

  public void dispararCubo() {
    leftSetPoint(325);
    rightSetPoint(325);
  }
}