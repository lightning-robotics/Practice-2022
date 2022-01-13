// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */

    TalonSRX top_shooter_motor = new TalonSRX(Constants.ShooterConstants.TOP_MOTOR_PORT);
    TalonSRX bottom_shooter_motor = new TalonSRX(Constants.ShooterConstants.BOTTOM_MOTOR_PORT);

    public Shooter() {

        top_shooter_motor.setInverted(false);
        bottom_shooter_motor.setInverted(false);

        top_shooter_motor.setNeutralMode(NeutralMode.Coast);
        bottom_shooter_motor.setNeutralMode(NeutralMode.Coast);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public void setTopMotorSpeed(double speed)
  {
    top_shooter_motor.set(ControlMode.PercentOutput, speed);
  }
  public void setBottomMotorSpeed(double speed)
  {

    bottom_shooter_motor.set(ControlMode.PercentOutput, speed);
  }
}
