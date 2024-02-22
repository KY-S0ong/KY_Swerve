// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */
  private final TalonFX flDrive = new TalonFX(6);
  private final TalonFX frDrive = new TalonFX(25);
  private final TalonFX blDrive = new TalonFX(20);
  private final TalonFX brDrive = new TalonFX(24);

  private final ProfiledPIDController blDriveController = new ProfiledPIDController(Constants.kP1, Constants.kI1, Constants.kD1, new TrapezoidProfile.Constraints(100, 15));
  private final ProfiledPIDController brDrivecontroller = new ProfiledPIDController(Constants.kP1, Constants.kI1, Constants.kD1, new TrapezoidProfile.Constraints(100, 15));
  private final ProfiledPIDController flDrivecontroller = new ProfiledPIDController(Constants.kP1, Constants.kI1, Constants.kD1, new TrapezoidProfile.Constraints(100, 15));
  private final ProfiledPIDController frDrivecontroller = new ProfiledPIDController(Constants.kP1, Constants.kI1, Constants.kD1, new TrapezoidProfile.Constraints(100, 15));
  

  public SwerveDrive() {
    TalonFXConfiguration t = new TalonFXConfiguration();
    flDrive.getConfigurator().apply(t);
    frDrive.getConfigurator().apply(t);
    blDrive.getConfigurator().apply(t);
    brDrive.getConfigurator().apply(t);
    t.Voltage.PeakForwardVoltage = 12.0;
    
    flDrive.setInverted(false);
    frDrive.setInverted(false);
    blDrive.setInverted(false);
    brDrive.setInverted(false);

    flDrive.setNeutralMode(NeutralModeValue.Coast);
    frDrive.setNeutralMode(NeutralModeValue.Coast);
    blDrive.setNeutralMode(NeutralModeValue.Coast);
    brDrive.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double input){
    flDrive.setVoltage(flDrivecontroller.calculate(flDrive.getVelocity().getValueAsDouble(), input));
    frDrive.setVoltage(frDrivecontroller.calculate(frDrive.getVelocity().getValueAsDouble(), input));
    blDrive.setVoltage(blDriveController.calculate(blDrive.getVelocity().getValueAsDouble(), input));
    brDrive.setVoltage(brDrivecontroller.calculate(brDrive.getVelocity().getValueAsDouble(), input));
  }
}
