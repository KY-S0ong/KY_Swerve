// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveTurn;

public class DriveCommand extends Command {
  private final SwerveDrive swerveDrive;
  private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(30);
  private XboxController xc;
  private double y = xc.getLeftY();
  

  public DriveCommand(SwerveDrive swerveDrive, XboxController x) {
    this.swerveDrive = swerveDrive;
    xc = x;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.drive(slewRateLimiter.calculate(xc.getLeftY()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
