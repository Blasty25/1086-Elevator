// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.elevator.ElevatorConstants.*;

/** Add your docs here. */
public class ElevatorIOReal implements ElevatorIO{

    private TalonFX leftKrack = new TalonFX(leftID);
    private TalonFX rightKrack = new TalonFX(rightID);

    private TalonFXConfiguration config = new TalonFXConfiguration();


    public ElevatorIOReal(){
        config.CurrentLimits.StatorCurrentLimit = currentLimit;

        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;

        config.Slot0.GravityType = GravityTypeValue.valueOf(gravity);
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = positionConversionFactor;
        
        config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Audio.BeepOnBoot = true;
        

        config.MotionMagic.MotionMagicCruiseVelocity = positionConversionFactor / 60;

        leftKrack.getConfigurator().apply(config);

        rightKrack.setControl(new Follower(leftID, true));

        leftKrack.setPosition(0.0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.currentHeight = leftKrack.getPosition().getValue().in(Rotations) * positionConversionFactor;
        
        inputs.leftCurrent = leftKrack.getStatorCurrent().getValue();
        inputs.rightCurrent = rightKrack.getStatorCurrent().getValue();

        inputs.leftVolts = leftKrack.getSupplyVoltage().getValue();
        inputs.rightVolts = rightKrack.getSupplyVoltage().getValue();

        inputs.velocity = (leftKrack.getPosition().getValue().in(Rotations) * positionConversionFactor) / 60;
    }

    @Override
    public void setVolts(double volts) {
        leftKrack.setVoltage(volts);
    }
}
