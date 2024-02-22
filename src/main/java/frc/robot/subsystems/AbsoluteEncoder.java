// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AbsoluteEncoder extends SubsystemBase {
  private AnalogInput absoluteEncoder;
  private boolean absoluteEncoderReversed;
  private double absoluteEncoderOffsetRad;

  public AbsoluteEncoder(int absoluteEncoderId, double absoluteEncoderOffset, 
    boolean absoluteEncoderReversed){

    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new AnalogInput(absoluteEncoderId);
  }

  //public AbsoluteEncoder() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);

    }
}
