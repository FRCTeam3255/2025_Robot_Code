package frc.robot.loggers;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.units.Units;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {
  public TalonFXLogger() {
    super(TalonFX.class);
  }

  @Override
  public void update(EpilogueBackend backend, TalonFX motor) {
    backend.log("Faults", motor.getFaultField().getValue());
    backend.log("CAN ID", motor.getDeviceID());
    backend.log("Position (rotations)", motor.getPosition().getValue().in(Units.Rotations));
    backend.log("Position (degrees)", motor.getPosition().getValue().in(Units.Degrees));
    backend.log("Requested Speed", motor.get());
    backend.log("Motor Voltage", motor.getMotorVoltage().getValue());
    backend.log("Stator Current", motor.getStatorCurrent().getValue());
    backend.log("Supply Current", motor.getSupplyCurrent().getValue());
    backend.log("Temperature", motor.getDeviceTemp().getValue());
    backend.log("Velocity (RPM)", motor.getVelocity().getValue().in(Units.RPM));
    backend.log("Acceleration (RPM^2)",
        motor.getAcceleration().getValue().in(Units.RotationsPerSecondPerSecond) / 60);
    backend.log("Closed Loop Error", motor.getClosedLoopError().getValue());
  }
}
