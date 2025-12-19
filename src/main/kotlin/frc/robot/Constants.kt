package frc.robot

import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.hardware.TalonFX
import teaforge.platform.RoboRio.CanDeviceToken

typealias Kraken = CanDevice.Motor.Kraken
const val JOYSTICK_TOLERANCE: Double = 0.01

class ElevatorConstants {

    enum class Direction {
        POSITIVE,
        NEGATIVE
    }

    companion object {

        // The bottom values for each encoder
        const val LEFT_BOTTOM: Double = 236.0
        const val RIGHT_BOTTOM: Double = -376.0
        const val MAX_DISPLACEMENT: Double = 1000.0

        // Feedback Control
        const val KP: Double = 0.001
        const val TOLERANCE: Int = 10

        val LEFT_MOTOR_UP_DIR: Direction = Direction.NEGATIVE
        val RIGHT_MOTOR_UP_DIR: Direction = Direction.POSITIVE
        val LEFT_ENCODER_UP_DIR: Direction = Direction.POSITIVE
        val RIGHT_ENCODER_UP_DIR: Direction = Direction.NEGATIVE
    }
}

sealed interface CanDevice<TToken : CanDeviceToken> {
    val id: Int

    sealed interface Motor<TToken : CanDeviceToken.MotorToken> : CanDevice<TToken> {
        override val id: Int

        enum class Neo(override val id: Int) : Motor<CanDeviceToken.MotorToken.NeoMotorToken> {
            Right(13), // Not blue
            Left(14) // Blue
        }
        enum class Kraken(override val id: Int) : Motor<CanDeviceToken.MotorToken.TalonMotorToken> {
            FrontLeftDrive(4),
            FrontLeftSteer(1),
            FrontRightDrive(6),
            FrontRightSteer(5),
            BackLeftDrive(7),
            BackLeftSteer(0),
            BackRightDrive(2),
            BackRightSteer(3)
        }
    }
    enum class Encoder(override val id: Int) : CanDevice<CanDeviceToken.EncoderToken> {
        FrontLeft(9),
        FrontRight(10),
        BackLeft(11),
        BackRight(8),
        LeftElevator(16), // Blue
        RightElevator(15), // Not blue
    }
    enum class Pigeon(override val id: Int) : CanDevice<CanDeviceToken.PigeonToken> {
        CentralPigeon(12)
    }
}

enum class SwervePod(val driveMotor: Kraken, val steeringMotor: Kraken, val encoder: CanDevice.Encoder) {
    FrontLeft(Kraken.FrontLeftDrive, Kraken.FrontLeftSteer, CanDevice.Encoder.FrontLeft),
    FrontRight(Kraken.FrontRightDrive, Kraken.FrontRightSteer, CanDevice.Encoder.FrontRight),
    BackLeft(Kraken.BackLeftDrive, Kraken.BackLeftSteer, CanDevice.Encoder.BackLeft),
    BackRight(Kraken.BackRightDrive, Kraken.BackRightSteer, CanDevice.Encoder.BackRight)
}

val elevatorDevices: List<CanDevice<*>> = listOf(
    CanDevice.Motor.Neo.Left,
    CanDevice.Motor.Neo.Right,
    CanDevice.Encoder.LeftElevator,
    CanDevice.Encoder.RightElevator
)

val allDevices: List<CanDevice<*>> = listOf(
    CanDevice.Motor.Kraken.entries,
    CanDevice.Motor.Neo.entries,
    CanDevice.Encoder.entries,
    CanDevice.Pigeon.entries
).flatten()