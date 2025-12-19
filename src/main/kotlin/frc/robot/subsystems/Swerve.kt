package frc.robot.subsystems

import edu.wpi.first.math.geometry.Rotation3d
import frc.robot.JOYSTICK_TOLERANCE
import frc.robot.Kraken
import frc.robot.SwervePod
import frc.robot.kP
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt

data class SwerveState(
    val steeringMotorPositions: Map<SwervePod, Double> = emptyMap(),
    val heading: Double = 0.0,
    val offset: Double = 0.0,
)

fun SwerveState.recalibrateIfRequested(driver: DriverControls): SwerveState =
    if (!driver.prevX && driver.x) copy(offset = heading + offset) else this

fun SwerveState.updateWithPodPosition(pod: SwervePod, position: Double): SwerveState =
    copy(steeringMotorPositions = steeringMotorPositions + (pod to position))

fun SwerveState.updateHeading(value: Rotation3d): SwerveState =
    copy(heading = wrapTo180(Math.toDegrees(value.z) - offset))

/**
 * Get the desired speeds of all drive motors for TeleOp drive.
 */
fun getDriveMotorSpeeds(
    turningStates: Map<SwervePod, Double>,
    forward: Double,
    strafe: Double,
    rotation: Double,
    heading: Double,
    maxTranslationalSpeed: Double,
    maxRotationalSpeed: Double
): Map<Kraken, Double> {
    if (abs(forward) < JOYSTICK_TOLERANCE && abs(strafe) < JOYSTICK_TOLERANCE && abs(rotation) < JOYSTICK_TOLERANCE) {
        return SwervePod.entries.flatMap {
            listOf(it.driveMotor to 0.0, it.steeringMotor to 0.0)
        }.toMap()
    }

    fun cosd(d: Double) = cos(Math.toRadians(d))
    fun sind(d: Double) = sin(Math.toRadians(d))

    // Field-centric translational direction in your convention (0 = forward, +90 = left)
    val translationalDirection =
        wrapTo180(Math.toDegrees(atan2(forward, strafe)) - 90.0 - heading)
    val translationalVelocity = sqrt(forward * forward + strafe * strafe) * maxTranslationalSpeed

    // Per-module rotation directions (in your convention)
    val rotationalDirections: Map<SwervePod, Double> = mapOf(
        SwervePod.FrontLeft to -45.0,
        SwervePod.FrontRight to -135.0,
        SwervePod.BackLeft to 45.0,
        SwervePod.BackRight to 135.0
    )
    val rotationalVelocity = rotation * maxRotationalSpeed

    // Precompute translation vector components (x right, y forward)
    // Angle convention: a=0 -> (x=0,y=+1), a=+90 -> (x=-1,y=0)
    val tX = translationalVelocity * (-sind(translationalDirection))
    val tY = translationalVelocity * (cosd(translationalDirection))

    val out = mutableMapOf<Kraken, Double>()

    for (m in SwervePod.entries) {
        val currentAngle = turningStates[m] ?: 0.0

        // Rotation vector for this module
        val rDir = rotationalDirections.getValue(m)
        val rX = rotationalVelocity * (-sind(rDir))
        val rY = rotationalVelocity * (cosd(rDir))

        // Resultant desired wheel vector
        val vX = tX + rX
        val vY = tY + rY
        val magnitude = hypot(vX, vY)

        // If we have no command, hold steer; zero drive
        val desiredAngle = if (magnitude < 0.05) {
            currentAngle
        } else {
            // Inverse of x=-sin(a), y=cos(a)  =>  a = atan2(-x, y)
            wrapTo180(Math.toDegrees(atan2(-vX, vY)))
        }

        // 180° flip optimization: keep steering move within ±90°
        var targetAngle = desiredAngle
        var driveSign = 1.0
        var error = wrapTo180(targetAngle - currentAngle)
        if (abs(error) > 90.0) {
            targetAngle = wrapTo180(targetAngle - 180.0)
            driveSign = -1.0
            error = wrapTo180(targetAngle - currentAngle)
        }

        // Steering speed: P-control on angle error
        val steerSpeed = (kP * error).coerceIn(-1.0, 1.0)

        // Drive speed: magnitude, flipped if needed, and cosine-scaled by alignment error
        val rawDrive = (magnitude * driveSign)
        val driveScaledByCos = rawDrive * cos(Math.toRadians(error))
        val driveSpeed = driveScaledByCos.coerceIn(-1.0, 1.0)

        out[m.steeringMotor] = steerSpeed
        out[m.driveMotor] = -driveSpeed
    }

    return out
}

fun wrapTo180(deg: Double): Double {
    val angle = ((deg % 360) + 360) % 360
    return if (angle > 180) angle - 360 else angle
}
