// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.AbsoluteEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveTurn extends SubsystemBase {
  /* Front Left Turn */
  private final TalonFX flTurn = new TalonFX(22);
  private final AbsoluteEncoder flEncoder = new AbsoluteEncoder(0, 1.64, true);

  /* Front Right Turn */
  private final TalonFX frTurn = new TalonFX(27);
  private final AbsoluteEncoder frEncoder = new AbsoluteEncoder(1, 0.94, true);

  /* Back Left Turn */
  private final TalonFX blTurn = new TalonFX(23);
  private final AbsoluteEncoder blEncoder = new AbsoluteEncoder(2, 1.99, false);

  /* Back Right Turn */
  private final TalonFX brTurn = new TalonFX(26);
  private AbsoluteEncoder brEncoder = new AbsoluteEncoder(3, 0.51, false);

  /* Imports */
  private boolean flag = false;

  private final ProfiledPIDController blTurnController = new ProfiledPIDController(Constants.kP3, Constants.kI3, Constants.kD3, new TrapezoidProfile.Constraints(15, 15));
  private final ProfiledPIDController brTurncontroller = new ProfiledPIDController(Constants.kP3, Constants.kI3, Constants.kD3, new TrapezoidProfile.Constraints(15, 15));
  private final ProfiledPIDController flTurncontroller = new ProfiledPIDController(Constants.kP3, Constants.kI3, Constants.kD3, new TrapezoidProfile.Constraints(15, 15));
  private final ProfiledPIDController frTurncontroller = new ProfiledPIDController(Constants.kP3, Constants.kI3, Constants.kD3, new TrapezoidProfile.Constraints(15, 15));
  
  private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(15);
  
  public SwerveTurn() {
    TalonFXConfiguration t = new TalonFXConfiguration();
    flTurn.getConfigurator().apply(t);
    frTurn.getConfigurator().apply(t);
    blTurn.getConfigurator().apply(t);
    brTurn.getConfigurator().apply(t);
    t.Voltage.PeakForwardVoltage = 12.0;

    flTurn.setInverted(false);
    frTurn.setInverted(false);
    blTurn.setInverted(false);
    brTurn.setInverted(false);

    flTurn.setNeutralMode(NeutralModeValue.Coast);
    frTurn.setNeutralMode(NeutralModeValue.Coast);
    blTurn.setNeutralMode(NeutralModeValue.Coast);
    brTurn.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("fl encoder angle", flEncoder.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("fr encoder angle", frEncoder.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("bl encoder angle", blEncoder.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("br encoder angle", brEncoder.getAbsoluteEncoderRad());
  }

  

  public void turn(double x, double rx){
    double[] speeds = new double[3];
    x = (x> Constants.deadband)? x:0;
    rx = (x> Constants.deadband)? x:0;

    //theta = (theta> Constants.deadband)? theta:0;
    speeds[0] = x+rx;
    speeds[1] = x+rx;
    speeds[2] = x-rx;
    speeds[3] = x-rx;
  
    double[] normalSpeeds = normalize(speeds);
    double[] finalSpeed = (flag) ? normalSpeeds:speeds;

    flTurn.setVoltage(flTurncontroller.calculate(flTurn.getVelocity().getValueAsDouble(), slewRateLimiter.calculate(finalSpeed[0])));
    frTurn.setVoltage(frTurncontroller.calculate(frTurn.getVelocity().getValueAsDouble(), slewRateLimiter.calculate(finalSpeed[1])));
    blTurn.setVoltage(blTurnController.calculate(blTurn.getVelocity().getValueAsDouble(), slewRateLimiter.calculate(finalSpeed[2])));
    brTurn.setVoltage(brTurncontroller.calculate(brTurn.getVelocity().getValueAsDouble(), slewRateLimiter.calculate(finalSpeed[3])));
  }
  public double[] normalize(double[]nums){
    flag = false;
    double sum = 0;
    for(int i = 0; i <3; i++){
      sum+= nums[i];
      if (Math.abs(nums[i]) > 1){
        flag = true;
      }
    }
    for(int j = 0; j< 3; j++){
      nums[j] /= sum;
    }
    return nums;}  
}
