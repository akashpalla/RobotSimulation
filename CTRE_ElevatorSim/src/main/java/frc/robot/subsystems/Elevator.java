// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
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
  
  private DCMotor elevatorMotors;
  private ElevatorSim elevatorSim;
  private TalonSRXSimCollection elevatorMasterSim;

  private double setpointMeters;

  public enum ElevatorState{
    ZEROED, SETPOINT, DISABLED;
  }
  

  private ElevatorState currState;
 

  public Elevator() {

    elevatorMotors = DCMotor.getVex775Pro(2);
    elevatorSim = new ElevatorSim(elevatorMotors, Constants.Elevator.kGearRatio, Constants.Elevator.kCarriageMass, Constants.Elevator.kPulleyRadiusMeters, Constants.Elevator.kMinHeight, Constants.Elevator.kMaxHeight);
    
    elevatorMaster = new WPI_TalonSRX(1);
    elevatorFollower = new WPI_TalonSRX(2);

    bottomLimitSwitch = new DigitalInputWrapper(5);

    elevatorMaster.configFactoryDefault();   // Resets any motor settings such as inversions, current limiting from previous programs
    elevatorFollower.configFactoryDefault();
    elevatorFollower.follow(elevatorMaster);

    elevatorMaster.config_kP(0, Constants.Elevator.kP);
    elevatorMaster.config_kI(0, Constants.Elevator.kI);
    elevatorMaster.config_kD(0, Constants.Elevator.kD);
    
    elevatorMasterSim = elevatorMaster.getSimCollection();

    setpointMeters = 0;

    updateState(ElevatorState.DISABLED);
  }

  
  public void periodic() {

    switch(currState){

      case SETPOINT:

        SmartDashboard.putString("Elevator State", "SETPOINT");
      
        if(setpointMeters == Constants.Elevator.kBottomSetpoint && bottomLimitSwitch.get()){
          currState = ElevatorState.ZEROED;
        }

        if(setpointMeters == Constants.Elevator.kBottomSetpoint && getHeight() <= 0.1){
          currState = ElevatorState.ZEROED;
        }
      
        
        elevatorMaster.set(ControlMode.Position, heightToTicks(setpointMeters), DemandType.ArbitraryFeedForward, .267); // Arbitrary FeedForward = Gravity Compensation. Adds .267 to the motor output
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
  
  public double getHeight(){
    return ticksToHeight(elevatorMaster.getSelectedSensorPosition());
  }

  private int heightToTicks(double height){
    double pulleyRotations = height / (2 * Math.PI * Constants.Elevator.kPulleyRadiusMeters);
    double motorRotations = pulleyRotations * Constants.Elevator.kGearRatio;
    int ticks = (int)(motorRotations * Constants.Elevator.kTicksPerRevolution);

    return ticks;
  }

  private double ticksToHeight(double ticks){
    double motorRotations = ticks/Constants.Elevator.kTicksPerRevolution;
    double pulleyRotations = motorRotations / Constants.Elevator.kGearRatio;
    double height = pulleyRotations * (2  * Math.PI * Constants.Elevator.kPulleyRadiusMeters);
    
    return height;
  }

  public void simPeriodic(){
      if(elevatorSim.getPositionMeters() <= .03)  // 3 CM is the approx threshold for the limit Switch
        bottomLimitSwitch.set(true);
      else 
        bottomLimitSwitch.set(false);

      elevatorSim.setInput(elevatorMaster.getMotorOutputVoltage());     // Feed motor Output into the physics Sim
      elevatorSim.update(.02);

      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));   

      elevatorMasterSim.setQuadratureRawPosition(heightToTicks(elevatorSim.getPositionMeters()));         // Update Encoders based on WPIlib Sim
      elevatorMasterSim.setQuadratureVelocity(heightToTicks(elevatorSim.getVelocityMetersPerSecond()));

      elevatorMasterSim.setBusVoltage(RoboRioSim.getVInVoltage());
  }

  public void updateState(ElevatorState newState){
    currState = newState;
  }

  public void updateSetpoint(double newSetpoint){
    setpointMeters = newSetpoint;
  }


  /**
   * Prints data to Smart Dashboard
   */
  public void log(){
    SmartDashboard.putNumber("Motor Output [-1, 1]", elevatorMaster.getMotorOutputPercent());
    SmartDashboard.putNumber("Elevator Position", ticksToHeight(elevatorMaster.getSelectedSensorPosition()));
    SmartDashboard.putNumber("Setpoint", heightToTicks(setpointMeters));
    SmartDashboard.putNumber("Error", setpointMeters - getHeight());
    SmartDashboard.putBoolean("Bottom Limit Switch", bottomLimitSwitch.get());

    
  }

}
