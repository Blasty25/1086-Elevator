// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ElevatorConstants;

import static edu.wpi.first.units.Units.*;

/** Add your docs here. */
public class ElevatorIOReal implements ElevatorIO{

    private TalonFX leftKrak;
    private TalonFX rightKrak;

    private TalonFXConfiguration config = new TalonFXConfiguration();


    public ElevatorIOReal(int leftId, int rightId){
        leftKrak = new TalonFX(leftId);
        rightKrak = new TalonFX(rightId);

        config.Audio.BeepOnBoot = true;
        config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.currentLimit.in(Amps);
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = ElevatorConstants.gearRatio;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;
        
        leftKrak.getConfigurator().apply(config);
        rightKrak.getConfigurator().apply(config);

        rightKrak.setControl(new Follower(leftId, true));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.currentHeight = Meters.of(leftKrak.getPosition().getValue().in(Radians) * ElevatorConstants.radius.in(Meters));
        
        inputs.leftCurrent = leftKrak.getStatorCurrent().getValue();
        inputs.rightCurrent = rightKrak.getStatorCurrent().getValue();

        inputs.leftVolts = leftKrak.getMotorVoltage().getValue();
        inputs.rightVolts = rightKrak.getMotorVoltage().getValue();

        inputs.leftTemp = leftKrak.getDeviceTemp().getValue();
        inputs.rightTemp = rightKrak.getDeviceTemp().getValue();

        inputs.velocity = MetersPerSecond.of(leftKrak.getVelocity().getValue().in(RadiansPerSecond) * ElevatorConstants.radius.in(Meters));
    }

    @Override
    public void setVolts(Voltage volts) {
        leftKrak.setControl(new VoltageOut(volts));
    }
}