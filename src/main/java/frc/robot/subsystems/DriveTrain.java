// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */

    TalonSRX front_left_motor = new TalonSRX(Constants.DriveConstants.FRONT_LEFT_MOTOR_PORT);
    TalonSRX front_right_motor = new TalonSRX(Constants.DriveConstants.FRONT_RIGHT_MOTOR_PORT);
    TalonSRX back_left_motor = new TalonSRX(Constants.DriveConstants.BACK_LEFT_MOTOR_PORT);
    TalonSRX back_right_motor = new TalonSRX(Constants.DriveConstants.BACK_RIGHT_MOTOR_PORT);



    public DriveTrain() {
        back_left_motor.follow(front_left_motor);
        back_right_motor.follow(front_right_motor);

        front_right_motor.setInverted(true);
        back_right_motor.setInverted(true);
        front_left_motor.setInverted(false);
        back_left_motor.setInverted(false);

        front_right_motor.setNeutralMode(NeutralMode.Coast);
        front_left_motor.setNeutralMode(NeutralMode.Coast);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public void setLeftMotorSpeed(double speed)
  {
    if (speed >= -1 && speed <= 1)
        front_left_motor.set(ControlMode.PercentOutput, speed);
  }
  public void setRightMotorSpeed(double speed)
  {
    if (speed >= -1 && speed <= 1)
        front_right_motor.set(ControlMode.PercentOutput, speed);
  }
}
