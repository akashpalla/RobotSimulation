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
public class SimpleMotor {

    public double numberMotors, resistance, kT, kV;

    public SimpleMotor(double numberMotors, double resistance, double kT, double kV){

        this.numberMotors = numberMotors;
        this.resistance = resistance;
        this.kT = kT;
        this.kV = kV;
    }


    public static SimpleMotor getFalcon500(double quantity){
        return new SimpleMotor(quantity, .0648, .0209, 431.66);
    }
}
