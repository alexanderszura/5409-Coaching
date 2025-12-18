package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Distance;

public interface ElevatorIO {

    @AutoLog
    public class ElevatorInputs {
        public boolean isMotorConnected = false;
        public double position = 0.0;
        public double tempature = 0.0;
        public double voltage = 0.0;
        public double current = 0.0;
    }

    public default void updateInputs(ElevatorInputs inputs) {}

    public default void setMotorVoltage(double voltage) {}

    public default void setSetpoint(Distance position) {}

    public default Distance getPosition() {
        return Meters.of(0.0);
    }
}
