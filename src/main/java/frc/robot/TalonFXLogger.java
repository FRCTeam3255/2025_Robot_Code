package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {
  public TalonFXLogger() {
    super(TalonFX.class);
  }

  @Override
  public void update(EpilogueBackend backend, TalonFX motor) {
    if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      backend.log("Faults", motor.getFaultField().getValue());
      backend.log("Description", motor.getDescription());
      backend.log("ID", motor.getDeviceID());
    }
    backend.log("Position", motor.getPosition().getValue());
    backend.log("Requested Speed", motor.get());
    backend.log("Motor Voltage", motor.getMotorVoltage().getValue());
    backend.log("Stator Current", motor.getStatorCurrent().getValue());
    backend.log("Supply Current", motor.getSupplyCurrent().getValue());
    backend.log("Temperature", motor.getDeviceTemp().getValue());
    backend.log("Velocity", motor.getVelocity().getValue());
    backend.log("Acceleration", motor.getAcceleration().getValue());
  }
}
