// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.DigitalInputWrapper;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  
  private WPI_TalonSRX elevatorMaster, elevatorFollower;
  private DigitalInputWrapper bottomLimitSwitch;
  
  private DCMotor elevatorMotorsim;
  private ElevatorSim elevatorSim;


  private double motorOutput;
  private double setpoint;

  public enum ElevatorState{
    ZEROED, SETPOINT, DISABLED;
  }
  

  private ElevatorState currState;
 

  public Elevator() {

    elevatorMotorsim = DCMotor.getVex775Pro(2);
    elevatorSim = new ElevatorSim(elevatorMotorsim, Constants.Elevator.kGearRatio, Constants.Elevator.kCarriageMass, Constants.Elevator.kPulleyRadius, Constants.Elevator.kMinHeight, Constants.Elevator.kMaxHeight);
    
    elevatorMaster = new WPI_TalonSRX(1);
    elevatorFollower = new WPI_TalonSRX(2);

    bottomLimitSwitch = new DigitalInputWrapper(5);

    elevatorMaster.configFactoryDefault();   // Resets any motor settings such as inversions, current limiting from previous programs
    elevatorFollower.configFactoryDefault();

    elevatorFollower.follow(elevatorMaster);

    motorOutput = 0;
    setpoint = 0;

    updateState(ElevatorState.DISABLED);
  }

  
  double integralLimit = 1;
  double errorSum = 0;
  double lastError = 0;
  double lastTimeStamp = 0;

  public void periodic() {

    switch(currState){

      case SETPOINT:

        SmartDashboard.putString("Elevator State", "SETPOINT");
        double error = setpoint - getElevatorHeight();
        double dt = Timer.getFPGATimestamp() - lastTimeStamp;  // Should be approximately 0.02s
        double errorRate = (error - lastError) /dt;

        errorSum += error*dt;

        motorOutput = (Constants.Elevator.kP* error) + (Constants.Elevator.kI * errorSum) + (Constants.Elevator.kD * errorRate);

        
        // The above is PID and its calculations. This is all explained here: https://youtu.be/jIKBWO7ps0w?t=263
    
        lastError = error;
        lastTimeStamp = Timer.getFPGATimestamp();


      if(setpoint == Constants.Elevator.kBottomSetpoint && bottomLimitSwitch.get()){
          currState = ElevatorState.ZEROED;
        }

        if(setpoint == Constants.Elevator.kBottomSetpoint && getElevatorHeight() <= 0.1){
          currState = ElevatorState.ZEROED;
        }
      
        
        elevatorMaster.set(ControlMode.PercentOutput, motorOutput);
        break;

      case ZEROED:
    
        SmartDashboard.putString("Elevator State", "ZEROED");
        elevatorMaster.set(ControlMode.PercentOutput, 0);

        break;
      case DISABLED:
     
        SmartDashboard.putString("Elevator State", "DISABLED");
        elevatorMaster.set(ControlMode.PercentOutput, 0);
        break;
      
    }
    
    if(!Robot.isReal()){
      simPeriodic();
    }
  
    log();
    
  }
  


  public double getElevatorHeight(){
    if(RobotBase.isReal()){
      return elevatorMaster.getSelectedSensorPosition() * Constants.Elevator.kDistancePerTick;
    }
    return elevatorSim.getPositionMeters();
  }

  public void simPeriodic(){
      if(elevatorSim.getPositionMeters() <= .03)  // 3 CM is the approx threshold for the limit Switch
        bottomLimitSwitch.set(true);
      else 
        bottomLimitSwitch.set(false);

      elevatorSim.setInput(elevatorMaster.getMotorOutputPercent()* RobotController.getBatteryVoltage());
      elevatorSim.update(.02);

      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));


  }

  public void updateState(ElevatorState newState){
    currState = newState;
  }

  public void updateSetpoint(double newSetpoint){
    setpoint = newSetpoint;
  }



  /**
   * Prints data to Smart Dashboard
   */
  public void log(){
    SmartDashboard.putNumber("Motor Output [-1, 1]", motorOutput);
    SmartDashboard.putNumber("Elevator Position", elevatorSim.getPositionMeters());
    SmartDashboard.putNumber("Elevator Velocity", elevatorSim.getVelocityMetersPerSecond());
    SmartDashboard.putNumber("Setpoint", setpoint);
    SmartDashboard.putNumber("Error", setpoint - elevatorSim.getPositionMeters());
    SmartDashboard.putBoolean("Bottom Limit Switch", bottomLimitSwitch.get());

    
  }

}
