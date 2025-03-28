// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.elevator.ElevatorConstants.*;

/** Add your docs here. */
public class ElevatorIOReal implements ElevatorIO{

    private TalonFX leftKrak = new TalonFX(leftID);
    private TalonFX rightKrak = new TalonFX(rightID);

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

        leftKrak.getConfigurator().apply(config);

        rightKrak.setControl(new Follower(leftID, true));

        rightKrak.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.currentHeight = leftKrak.getPosition().getValue().in(Rotations) * ElevatorConstants.drumRadius;
        
        inputs.leftCurrent = leftKrak.getStatorCurrent().getValue();
        inputs.rightCurrent = rightKrak.getStatorCurrent().getValue();

        inputs.leftVolts = leftKrak.getMotorVoltage().getValue();
        inputs.rightVolts = rightKrak.getMotorVoltage().getValue();

        inputs.velocity = leftKrak.getVelocity().getValue().in(RadiansPerSecond);
    }

    @Override
    public void setVolts(double volts) {
        leftKrak.setVoltage(volts);
    }
}
