package frc.robot.subsystems

import frc.robot.CanDevice
import frc.robot.ElevatorConstants
import frc.robot.JOYSTICK_TOLERANCE
import kotlin.math.abs

data class ElevatorState(
    val leftBottom: Double = 0.0,
    val rightBottom: Double = 0.0,
    val calibrated: Boolean = false,
    val leftPosition: Double = -124.0,
    val rightPosition: Double = -16.0,
    val prevLeftPosition: Double = -124.0,
    val prevRightPosition: Double = -16.0,
)

fun ElevatorState.calibrateIfRequested(manipulator: ManipulatorControls): ElevatorState =
    if (!manipulator.prevX && manipulator.x) {
        copy(
            leftBottom = prevLeftPosition,
            rightBottom = prevRightPosition,
            calibrated = true,
        )
    } else this

fun ElevatorState.updatePosition(encoder: CanDevice.Encoder, position: Double): ElevatorState =
    when (encoder) {
        CanDevice.Encoder.LeftElevator -> copy(
            prevLeftPosition = position,
            leftPosition = updateNetDegrees(
                prevAbsDeg = prevLeftPosition,
                currAbsDeg = position,
                netDeg = leftPosition
            )
        )

        CanDevice.Encoder.RightElevator -> copy(
            prevRightPosition = position,
            rightPosition = updateNetDegrees(
                prevAbsDeg = prevRightPosition,
                currAbsDeg = position,
                netDeg = rightPosition
            )
        )

        else -> this
    }

fun ElevatorState.getElevatorPower(joystick: Double): Pair<Double, Double> {
    fun power(
        motorDir: ElevatorConstants.Direction,
        encoderDir: ElevatorConstants.Direction,
        motorPos: Double,
        bottom: Double,
    ): Double {
        val joystickPower =
            when (motorDir) {
                ElevatorConstants.Direction.POSITIVE -> joystick
                ElevatorConstants.Direction.NEGATIVE -> -joystick
            }
        val offset = when (encoderDir) {
            ElevatorConstants.Direction.POSITIVE -> ElevatorConstants.MAX_DISPLACEMENT
            ElevatorConstants.Direction.NEGATIVE -> -ElevatorConstants.MAX_DISPLACEMENT
        }
        val target =
            if (joystick > JOYSTICK_TOLERANCE) bottom + 100 + offset //100 is a test buffer
            else bottom + 100 //100 is a test buffer

        val encoderError = target - motorPos
        val motorError = if (motorDir == encoderDir) encoderError else -encoderError
        val isClose = abs(motorError) < ElevatorConstants.TOLERANCE
        val pid = if (!isClose) ElevatorConstants.KP * motorError else 0.0
        val oppositeSigns = ((pid < 0.00001 && joystickPower > 0) || (pid > -0.00001 && joystickPower < 0))
        return if (!calibrated) {
            joystickPower
        } else {
            if ((abs(pid) < abs(joystickPower) && !isClose) || oppositeSigns) pid else joystickPower
        }
    }

    return Pair(
        power(
            motorDir = ElevatorConstants.LEFT_MOTOR_UP_DIR,
            encoderDir = ElevatorConstants.LEFT_ENCODER_UP_DIR,
            motorPos = leftPosition,
            bottom = leftBottom,
        ),
        power(
            motorDir = ElevatorConstants.RIGHT_MOTOR_UP_DIR,
            encoderDir = ElevatorConstants.RIGHT_ENCODER_UP_DIR,
            motorPos = rightPosition,
            bottom = rightBottom,
        ),
    )
}

// This function updates a cumulative angle by adding a change in angle. It handles all wrapping needed.
// prevAbsDeg, currAbsDeg: absolute CANCoder readings in [-180, 180]
// netDeg: your current cumulative tracking in degrees (e.g., 370 means +370° since start)
fun updateNetDegrees(prevAbsDeg: Double, currAbsDeg: Double, netDeg: Double): Double {
    // Wrap a difference into (-180, 180]
    fun shortestDeltaDeg(diff: Double): Double {
        // Shift by 540 to handle negatives, mod 360, then shift to (-180, 180]
        return ((diff + 540.0) % 360.0) - 180.0
    }

    val diff = currAbsDeg - prevAbsDeg
    val delta = shortestDeltaDeg(diff)
    return netDeg + delta
}
