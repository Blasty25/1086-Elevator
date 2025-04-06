package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOReal implements ElevatorIO {
    private TalonFX leftMotor;
    private TalonFX rightMotor;

    private TalonFXConfiguration config = new TalonFXConfiguration();

    public ElevatorIOReal(int leftId, int rightId) {
        leftMotor = new TalonFX(leftId);
        rightMotor = new TalonFX(rightId);

        config.Audio.BeepOnBoot = true;
        config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.currentLimit.in(Amps);
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = ElevatorConstants.gearRatio;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;

        leftMotor.getConfigurator().apply(config);

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        rightMotor.getConfigurator().apply(config);

        rightMotor.setControl(new Follower(leftId, true));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leftCurrent = leftMotor.getStatorCurrent().getValue();
        inputs.rightCurrent = rightMotor.getStatorCurrent().getValue();

        inputs.leftTemperature = leftMotor.getDeviceTemp().getValue();
        inputs.rightTemperature = rightMotor.getDeviceTemp().getValue();

        inputs.leftVolts = leftMotor.getMotorVoltage().getValue();
        inputs.rightVolts = rightMotor.getMotorVoltage().getValue();

        inputs.position = Meters.of(leftMotor.getPosition().getValue().in(Radians) * ElevatorConstants.radius.in(Meters));
        inputs.velocity = MetersPerSecond.of(leftMotor.getVelocity().getValue().in(RadiansPerSecond) * ElevatorConstants.radius.in(Meters));
    }

    @Override
    public void setVolts(Voltage volts) {
        leftMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void resetEncoder() {
        leftMotor.setPosition(Radians.zero());
    }

    @Override
    public void resetEncoder(Distance distance) {
        leftMotor.setPosition(Rotations.of(distance.div(ElevatorConstants.radius).magnitude()));
    }
}