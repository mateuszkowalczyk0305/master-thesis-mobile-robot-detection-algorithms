# Motor control notes

## Speed range

The motor PWM range used in this project:

- Minimum PWM for movement under load: **1000**
- Maximum PWM: **1500**

Values below **1000** are typically not sufficient to start the motor when mechanical resistance is present.

## Startup behaviour

Due to static friction and motor startup torque requirements, the motor requires a short high-power impulse.

Recommended startup behaviour:

1. Apply PWM close to **1500**
2. After startup reduce PWM to desired operating value (**1000–1300**)

Example:

motor.setSpeed(1500);
motor.setSpeed(1100);

## Observed behaviour

- Setting PWM directly below **1000** does not start the motor.
- Increasing PWM gradually (e.g. `200 → 800`) does not start the motor.
- A strong initial impulse is required to overcome static friction.

## Notes

The exact startup threshold depends on:

- supply voltage
- mechanical load
- gearbox friction