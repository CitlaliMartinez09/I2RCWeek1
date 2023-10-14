// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class PIDTurn extends CommandBase {
  DriveTrain dt;
  double setpointAngle;
  PIDController pid = new PIDController(0.00333333333, 0, 0);
  int motorSign;

  /** Creates a new PIDTurn. */
  public PIDTurn(DriveTrain dt, double setpointAngle) {
    this.dt = dt;
    this.setpointAngle = setpointAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
    if (setpointAngle >= 0){ //If the motor is one, it is a counterclockwise turn
      motorSign = 1;
    } else{
    motorSign = -1;
  }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetNavx();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
