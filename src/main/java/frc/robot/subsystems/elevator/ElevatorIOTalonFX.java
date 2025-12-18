package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.tools.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {

    private final TalonFX motor;

    private final PositionVoltage positionControl;

    private final StatusSignal<Angle>       positionSignal;
    private final StatusSignal<Temperature> temperatureSignal;
    private final StatusSignal<Voltage>     voltageSignal;
    private final StatusSignal<Current>     currentSignal;

    public ElevatorIOTalonFX(int id) {
        motor = new TalonFX(id);

        positionControl = new PositionVoltage(0.0);
        positionControl.withSlot(0);

        TalonFXConfigurator configator = motor.getConfigurator();

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0 = 
            new Slot0Configs()
                .withKP(0.0001)
                .withKI(0.0)
                .withKD(0.0);

        config.MotorOutput = 
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        config.CurrentLimits = 
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true);

        config.Feedback = 
            new FeedbackConfigs()
                .withSensorToMechanismRatio(ElevatorConstants.kSensorRatio);

        PhoenixUtil.tryUntilOk(3, () -> configator.apply(config));

        positionSignal    = motor.getPosition();
        temperatureSignal = motor.getDeviceTemp();
        voltageSignal     = motor.getMotorVoltage();
        currentSignal     = motor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50, 
            positionSignal,
            temperatureSignal,
            voltageSignal,
            currentSignal
        );

        motor.optimizeBusUtilization();
    }

    @Override
    public void setMotorVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setSetpoint(Distance position) {
        PhoenixUtil.tryUntilOk(
            3, 
            () -> motor.setControl(
                positionControl.withPosition(
                    position.in(Meters)
                )
            )
        );
    }

    @Override
    public Distance getPosition() {
        return Meters.of(positionSignal.getValueAsDouble());
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.isMotorConnected = motor.isConnected();

        inputs.position = getPosition().in(Meters);
        inputs.tempature = temperatureSignal.getValueAsDouble();
        inputs.voltage = voltageSignal.getValueAsDouble();
        inputs.current = currentSignal.getValueAsDouble();
    }
}
