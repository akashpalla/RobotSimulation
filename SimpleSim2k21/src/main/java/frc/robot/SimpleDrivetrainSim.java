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
public class SimpleDrivetrainSim {

    
    private double distance, velocity, acceleration;
    private double gearRatio, wheelRadius, robotMass;
    private SimpleMotor motor;

    /**
     * Creates a New SimpleDrivetrainSim object
     * @param gearRatio         the gear ratio of the drivetrain with a number > 1 indicating a reduction 
     * @param wheelRadius       the radius of the drivetrain wheels in meters
     * @param robotMass         the total mass of the robot in kg
     * @param motor             SimpleMotor object corresponding to the drivetrain motor 
     */
    public SimpleDrivetrainSim(double gearRatio, double wheelRadius, double robotMass, SimpleMotor motor ){

        this.gearRatio = gearRatio;
        this.wheelRadius = wheelRadius;
        this.robotMass = robotMass;
        this.motor = motor;

        distance = 0;
        velocity = 0;
        acceleration = 0;
    }

    /**
     *  Simulates the drivetrain for a given period of time. 
     * @param motorOutput       motor output [-1,1] applied 
     * @param period            period of time in seconds to simulate 
     */
    public void update(double motorOutput, double period){

        double motorVolts = motorOutput * 12;

        acceleration = motor.numberMotors * (motorVolts - (velocity * gearRatio * wheelRadius)/motor.kV) * (motor.kT/(gearRatio * motor.resistance * robotMass * wheelRadius));   
        distance += velocity * period + acceleration * Math.pow(period, 2);
        velocity += acceleration * period;
    
    }


    public double getDistance(){
        return distance;
    }

    public double getVelocity(){
        return velocity;
    }

    public void reset(){
        distance = velocity = acceleration = 0;
    }
}
