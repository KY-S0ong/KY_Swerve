// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveTurn;

public class TurnCommand extends Command {
  private final SwerveTurn swerveTurn;
  private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(30);
  private XboxController xc;

  private double x = xc.getLeftX();
  private double rx = xc.getRightX();

  public TurnCommand(SwerveTurn swerveTurn, XboxController x) {
    this.swerveTurn = swerveTurn;
    xc = x;
    addRequirements(swerveTurn);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //swerveTurn.turn(slewRateLimiter.calculate(xc.getLeftX()), slewRateLimiter.calculate(xc.getRightX()));
    swerveTurn.turn(slewRateLimiter.calculate(x),slewRateLimiter.calculate(rx));
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
