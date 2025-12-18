package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Centimeters;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    public Command setSetpoint(Distance distance) {
        return Commands.runOnce(() -> io.setSetpoint(distance), this)
        .andThen(
            Commands.waitUntil(() -> io.getPosition().isNear(distance, Centimeters.of(2.0)))
        );
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
