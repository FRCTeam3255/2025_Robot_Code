package frc.robot.loggers;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

@CustomLoggerFor(Angle.class)
public class AngleLogger extends ClassSpecificLogger<Angle> {
  public AngleLogger() {
    super(Angle.class);
  }

  @Override
  public void update(EpilogueBackend backend, Angle Angle) {
    backend.log("Degrees", Angle.in(Units.Degrees));
    backend.log("Rotations", Angle.in(Units.Rotations));
    backend.log("Radians", Angle.in(Units.Radians));
  }
}
