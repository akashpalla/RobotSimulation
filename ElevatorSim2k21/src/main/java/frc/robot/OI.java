// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.MoveToSetpoint;

/** Add your docs here. */
public class OI {

    public OI(){
        SmartDashboard.putData("Elevator Bottom", new MoveToSetpoint(Constants.Elevator.kBottomSetpoint));
        SmartDashboard.putData("Elevator Top", new MoveToSetpoint(Constants.Elevator.kHightSetpoint));
        SmartDashboard.putData("Elevator Mid", new MoveToSetpoint(Constants.Elevator.kMidSetpoint));
    }




}
