// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ShooterTest extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  /**
   * Creates a new Command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterTest() {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  
  public void initialize() 
  {
    RobotContainer.shooter.setTopMotorSpeed(0);
    RobotContainer.shooter.setBottomMotorSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.

  double topMotorSpeed = 0;
  double bottomMotorSpeed = 0;

  int cooldown = 0;

  @Override
  public void execute() 
  {
    // making the code only run twice per second
    if (cooldown >= 30)
    {
      if (RobotContainer.controller.getXButtonReleased())
        topMotorSpeed += 0.05;
      if (RobotContainer.controller.getAButtonReleased())
        bottomMotorSpeed += 0.05;
      if (RobotContainer.controller.getYButtonReleased())
        topMotorSpeed -= 0.05;
      if (RobotContainer.controller.getBButtonReleased())
        bottomMotorSpeed -= 0.05;

      cooldown = 0;
    }

    // we don't want to go over/under the max speed, but more importantly, over 100% power
    if (Math.abs(topMotorSpeed) > Constants.ShooterConstants.MAX_SPEED)
      topMotorSpeed = Constants.ShooterConstants.MAX_SPEED * (topMotorSpeed / Math.abs(topMotorSpeed));
    if (Math.abs(bottomMotorSpeed) > Constants.ShooterConstants.MAX_SPEED)
      bottomMotorSpeed = Constants.ShooterConstants.MAX_SPEED * (bottomMotorSpeed / Math.abs(bottomMotorSpeed));


    System.out.println(topMotorSpeed + "," + bottomMotorSpeed);


    if (RobotContainer.controller.getBumper(Hand.kRight))
    {
      RobotContainer.shooter.setTopMotorSpeed(topMotorSpeed);
      RobotContainer.shooter.setBottomMotorSpeed(bottomMotorSpeed);
    }
    else
    {
      RobotContainer.shooter.setTopMotorSpeed(0);
      RobotContainer.shooter.setBottomMotorSpeed(0);
    }

    cooldown++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    initialize();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
