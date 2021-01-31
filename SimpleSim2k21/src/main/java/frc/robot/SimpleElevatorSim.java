/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class SimpleElevatorSim {

    private double height, velocity, acceleration;
    private double gearRatio, pulleyRadius, carriageMass, minHeight, maxHeight;
    private SimpleMotor motor;


    public SimpleElevatorSim(double minHeight, double maxHeight, double gearRatio, double pulleyRadius, double carriageMass, SimpleMotor motor){

        this.minHeight = minHeight;
        this.maxHeight = maxHeight;
        this.gearRatio = gearRatio;
        this.pulleyRadius = pulleyRadius;
        this.carriageMass = carriageMass;
        this.motor = motor;
        
    }


    public void update(double motorOutput, double period){
        double motorVolts = motorOutput * 12;

        acceleration = motor.numberMotors * (motorVolts - (velocity * gearRatio * pulleyRadius)/motor.kV) * (motor.kT/(gearRatio * motor.resistance * carriageMass * pulleyRadius)) - 9.8;

   /*     if(height >= maxHeight && motorOutput > 0){
            acceleration = 0;
            velocity = 0;
        }else if(height <= minHeight && motorOutput<0){
            acceleration = 0;
            velocity = 0;
        }
*/
        height += velocity * period + acceleration * Math.pow(period, 2);
        velocity += acceleration * period;
    }

    public double getHeight(){
        return height;
    }

    public double getVelocity(){
        return velocity;
    }

    public void reset(){
        height = velocity = acceleration = 0;
    }

}
