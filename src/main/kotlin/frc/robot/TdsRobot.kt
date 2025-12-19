package frc.robot

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.Orchestra
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.wpilibj.Filesystem
import teaforge.platform.RoboRio.*
import teaforge.utils.*
import kotlin.math.*

const val kP = 0.0026
const val kI = 0.0
const val kD = 0.0

data class PIDState(var integral: Double = 0.0, var prevError: Double = 0.0)

val tieDieSamuraiRobot =
        RoboRioProgram(
                init = ::init,
                update = ::update,
                subscriptions = ::subscriptions,
        )

data class Model(
        val robotMode: RunningRobotState,

        val leftBottom: Double,
        val rightBottom: Double,
        val calibrated: Boolean,

        // 8 when all talons initialized. This turns to 0 once songs are
        // initialized for the first time and stays zero for the rest of the program.
        val talonsInitialized: Int,

        val loadedSongs: Map<Kraken, Int>, // Int represents song file
        val activeSongs: Map<Kraken, Int>,

        val steeringMotorPositions: Map<SwervePod, Double>,
        val leftElevatorPosition: Double,
        val rightElevatorPosition: Double,
        val prevLeftElevatorPosition: Double,
        val prevRightElevatorPosition: Double,

        val heading: Double, // [-180, 180]; 0 is forward. Equal to Pigeon Z value minus offset
        val offset: Double, // This is set equal to Pigeon Z value when calibrated.

        val leftJoystickX: Double,
        val leftJoystickY: Double, // Up is positive
        val rightJoystickX: Double,
        val prevX: Boolean,
        val x: Boolean,

        val manipulatorLeftJoystickY: Double,
        val dpadDown: Boolean,
        val dpadDownPressed: Boolean,
        val dpadUp: Boolean,
        val dpadUpPressed: Boolean,

        val pidStates : Map<Kraken, PIDState>,

        val tokens: CanDeviceRegistry,
        val krakens: Map<Kraken, CanDeviceToken.MotorToken.TalonMotorToken>,
        val orchestras: Map<Kraken, OrchestraToken>,
        val hidInputs: Map<Int, HidInputToken>,
)

sealed interface Message {
        data class InitDevice(val type: CanDeviceType, val id: Int, val token: Result<CanDeviceToken, Error>) : Message
        data class RobotState(val state: RunningRobotState) : Message
        data class RobotStateChanged(val prevState: RunningRobotState, val newState: RunningRobotState) : Message

        data class LogError(val context: String, val error: Maybe<Error>) : Message
        data class LogErrors(val errors: List<Error>) : Message
        data class LogDio(val result: Result<DioPort, Error>) : Message
        data class Log(val str: String) : Message

        data class DriverControllerState(val controllerState: HidValue) : Message
        data class ManipulatorControllerState(val controllerState: HidValue) : Message

        data class InitSong(val motor: Pair<Kraken, Int>, val song: Result<ByteArray, Error>) : Message
        data class ProcessLoadedSong(val motor: Pair<Kraken, Int>, val error: Maybe<Error>) : Message
        data class ActivateSong(val motor: Pair<Kraken, Int>, val error: Maybe<Error>) : Message
        data class RestartSong(val motor: Pair<Kraken, Int>, val error: Maybe<Error>) : Message

        data class UpdateSwervePodPosition(val pod: SwervePod, val position: Double) : Message
        data class UpdateElevatorPosition(val encoder: CanDevice.Encoder, val position: Double) : Message
        data class PigeonValue(val pigeon: CanDevice.Pigeon, val value: Rotation3d) : Message
}

fun update(msg: Message, model: Model): Pair<Model, List<Effect<Message>>> {
        return when (msg) {
                is Message.DriverControllerState -> {
                        val controllerState = msg.controllerState
                        val axisValues = controllerState.axisValues

                        // TODO (max): Account for (missing) controller in platform
                        try {
                                model.copy(
                                    leftJoystickX = axisValues[0],
                                    leftJoystickY = -axisValues[1],
                                    rightJoystickX = axisValues[2],
                                    prevX = model.x,
                                    x = controllerState.buttonValues[1],
                                ) to emptyList()
                        } catch (_: Exception) {
                                model to emptyList()
                        }
                }
                is Message.RobotState -> {
                        val returnBeforeSong = when (msg.state) {
                                RunningRobotState.Teleop -> {

                                        // Recalibrate
                                        val newModel = if (!model.prevX && model.x){
                                                model.copy(offset = model.heading + model.offset)
                                        } else model

                                        // Update swerve
                                        val driveMotorSpeeds: Map<Kraken, Double> = getDriveMotorSpeeds(
                                                turningStates = newModel.steeringMotorPositions,
                                                forward = newModel.leftJoystickY,
                                                strafe = newModel.leftJoystickX,
                                                rotation = newModel.rightJoystickX,
                                                heading = newModel.heading,
                                                maxTranslationalSpeed = 0.2,
                                                maxRotationalSpeed = 0.1,
                                        )

                                        // Generate swerve & stop song effects
                                        val swerveSongEffects: List<Effect<Message>> = driveMotorSpeeds.flatMap { entry ->
                                                model.krakens[entry.key]?.let {
                                                        if (abs(entry.value) > JOYSTICK_TOLERANCE) {
                                                                if (model.activeSongs.contains(entry.key))
                                                                        listOf(Effect.StopSong(
                                                                                token = model.orchestras[entry.key]!!,
                                                                                message = { result ->
                                                                                        val error = if (result is Result.Error) Maybe.Some(result.value) else Maybe.None
                                                                                        Message.LogError(
                                                                                                "stopping song for ${entry.key.name}", error
                                                                                        )
                                                                                })) else emptyList()
                                                        } else { emptyList() } + Effect.SetCanMotorSpeed(it, entry.value)
                                                } ?: listOf(Effect.Log(
                                                        "Failed to fetch drive state of ${entry.key.name}"
                                                ))
                                        }

                                        // Elevator effects
                                        val power = getElevatorPower(
                                                joystick = model.manipulatorLeftJoystickY,
                                                leftPos = model.leftElevatorPosition,
                                                rightPos = model.rightElevatorPosition,
                                                model = model
                                        )
                                        val elevatorEffects = listOf(
                                                model.tokens.get(CanDevice.Motor.Neo.Left).unwrap(
                                                        default = Effect.Log("Failed to fetch neo LEFT"),
                                                        fn = { Effect.SetCanMotorSpeed(it, power.first) }
                                                ),
                                                model.tokens.get(CanDevice.Motor.Neo.Right).unwrap(
                                                        default = Effect.Log("Failed to fetch neo RIGHT"),
                                                        fn = { Effect.SetCanMotorSpeed(it, power.second) }
                                                ))

                                        // Remove stopped songs from active songs
                                        newModel.copy(
                                                activeSongs = newModel.activeSongs.filter { entry ->
                                                    driveMotorSpeeds[entry.key]?.let { abs(it) <= JOYSTICK_TOLERANCE } != false
                                                },
                                                loadedSongs = newModel.activeSongs.filter { entry ->
                                                    driveMotorSpeeds[entry.key]?.let { abs(it) <= JOYSTICK_TOLERANCE } != false
                                                }
                                        ) to (swerveSongEffects + elevatorEffects)
                                }
                                else -> model to emptyList()
                        }
                        // Play song
                        if (model.talonsInitialized == 8)
                                returnBeforeSong.copy(
                                        first = returnBeforeSong.first.copy(talonsInitialized = 0),
                                        second = returnBeforeSong.second + playSong()
                                )
                        else returnBeforeSong
                }
                is Message.RobotStateChanged -> {
                        val effects =
                                if (msg.newState == RunningRobotState.Disabled) {
                                        if (model.activeSongs.isEmpty()) playSong()
                                        else model.activeSongs.map { entry ->
                                                val motor = entry.key
                                                model.tokens.get(motor).unwrap(
                                                        default = Effect.Log("Failed to stop song on ${motor.name}"),
                                                        fn = { Effect.StopSong(
                                                                token = model.orchestras[motor]!!,
                                                                message = { result ->
                                                                        val error = if (result is Result.Error) Maybe.Some(result.value) else Maybe.None
                                                                        Message.RestartSong(
                                                                                motor = entry.toPair(),
                                                                                error = error
                                                                ) }
                                                        )}
                                                )
                                        }

                                } else {
                                        emptyList()
                                }

                        val newModel = if (msg.newState == RunningRobotState.Disabled)
                                model.copy(
                                        robotMode = msg.newState,
                                        loadedSongs = emptyMap(),
                                        activeSongs = emptyMap()
                                ) else model.copy(robotMode = msg.newState)

                        newModel to effects + Effect.Log("State changed to ${msg.newState.name}")
                }
                is Message.InitSong -> {
                        val motor = msg.motor.first
                        model to when (msg.song) {
                                is Result.Success -> {

                                        listOf(model.tokens.get(motor).unwrap(
                                                default = Effect.Log("Failed to init song on ${motor.name}"),
                                                fn = { Effect.LoadSong(
                                                        motor = it,
                                                        songData = msg.song.value,
                                                        message = { result ->
                                                                val error = if (result is Result.Error) Maybe.Some(result.value) else Maybe.None
                                                                Message.ProcessLoadedSong(msg.motor, error)
                                                        }
                                                )}
                                        ))
                                }
                                is Result.Error -> {
                                        logError(
                                                context = "reading song file for motor ${motor.name}",
                                                error = Maybe.Some(msg.song.value)
                                        )
                                }
                        }
                }

                is Message.ProcessLoadedSong -> {
                        val logErrorEffect: List<Effect<Message>> = logError(
                                context = "loading song for motor ${msg.motor}",
                                error = msg.error
                        )
                        if (logErrorEffect.isNotEmpty()) model to logErrorEffect

                        val loadedSongs = model.loadedSongs + msg.motor
                        val effects = if (loadedSongs.keys.containsAll(Kraken.entries)) {
                                loadedSongs.map { entry ->
                                        val motor = entry.key
                                        model.tokens.get(motor).unwrap(
                                                default = Effect.Log("Couldn't load song on ${motor.name}"),
                                                fn = { Effect.PlaySong(
                                                        token = model.orchestras[entry.key]!!,
                                                        message = { result ->
                                                                val error = if (result is Result.Error) Maybe.Some(result.value) else Maybe.None
                                                                Message.ActivateSong(entry.toPair(), error)
                                                        }
                                                ) }
                                        )
                                }
                        } else emptyList()

                        model.copy(loadedSongs = loadedSongs) to effects
                }

                is Message.LogError -> model to logError(msg.context, msg.error)

                is Message.UpdateSwervePodPosition -> {
                        val motorPosPair: Pair<SwervePod, Double> = msg.pod to msg.position
                        val steeringMotorPositions = model.steeringMotorPositions + motorPosPair
                        model.copy(steeringMotorPositions = steeringMotorPositions) to emptyList()
                }

                is Message.PigeonValue -> {
                        val heading = wrapTo180(Math.toDegrees(msg.value.z) - model.offset)
                        model.copy(heading = heading) to emptyList()
                }

                is Message.LogErrors -> {
                        model to msg.errors.map {
                                Effect.Log(it.toString())
                        }

                }

                is Message.ActivateSong -> {
                        val effects = logError(
                                context = "playing song on motor ${msg.motor.first}",
                                error = msg.error
                        )

                        return if (effects.isNotEmpty()) model to effects
                        else model.copy(activeSongs = model.activeSongs + msg.motor) to effects
                }

                is Message.LogDio -> {
                        val log: String = when (msg.result) {
                                is Result.Error -> "Failed to set DIO: ${msg.result.value}"
                                is Result.Success -> "Successfully set DIO ${msg.result.value}"
                        }

                        model to listOf(Effect.Log(log))
                }

                is Message.Log -> {
                        model to listOf(Effect.Log(msg.str))
                }

                is Message.InitDevice -> {
                        val mismatchError: List<Effect<Message>> = listOf(Effect.Log("Could not find CAN device" +
                                " of type ${msg.type} with id ${msg.id}"))
                        when (msg.token) {
                                is Result.Success -> {
                                        getDeviceById(msg.id).unwrap(
                                                default = model to mismatchError,
                                                fn = {
                                                        val log = "Successfully registered " +
                                                                "${msg.type} at CAN " +
                                                                msg.id
                                                        val registry = model.tokens.plus(it, msg.token.value)
                                                        val newModel = if (msg.type == CanDeviceType.Talon) {
                                                                model.copy(
                                                                        tokens = registry,
                                                                        talonsInitialized = model.talonsInitialized + 1
                                                                )
                                                        } else model.copy(tokens = registry)
                                                         newModel to listOf(Effect.Log(log))
                                                }
                                        )

                                }
                                is Result.Error -> {
                                        model to logError(
                                                context = "Registering CAN device with id ${msg.id}",
                                                error = Maybe.Some(msg.token.value)
                                        )
                                }
                        }
                }

                is Message.RestartSong -> {
                        val musicDir = "${Filesystem.getDeployDirectory().absolutePath}/music"
                        val log = logError("stopping song for ${msg.motor.first.name}", msg.error)
                        model to (log + Effect.ReadFile(
                                path = "$musicDir/${msg.motor.second}.chrp",
                                message = { result -> Message.InitSong(msg.motor, result) }
                        ))
                }

                is Message.ManipulatorControllerState -> {
                        val controllerState = msg.controllerState
                        val axisValues = controllerState.axisValues

                        // TODO (max): Account for (missing) controller in platform
                        try {
                                val dpadDown = controllerState.buttonValues[4]
                                val dpadUp = controllerState.buttonValues[5]
                                val currentX = controllerState.buttonValues[1]

                                if (!model.x && currentX) {
                                    model.copy(
                                        leftBottom = model.prevLeftElevatorPosition,
                                        rightBottom = model.prevRightElevatorPosition,
                                        calibrated = true,
                                        manipulatorLeftJoystickY = -axisValues[1],
                                        dpadDown = dpadDown,
                                        dpadUp = dpadUp,
                                        dpadDownPressed = !model.dpadDown && dpadDown,
                                        dpadUpPressed = !model.dpadUp && dpadUp
                                    ) to emptyList()
                                } else {
                                    model.copy(
                                        manipulatorLeftJoystickY = -axisValues[1],
                                        dpadDown = dpadDown,
                                        dpadUp = dpadUp,
                                        dpadDownPressed = !model.dpadDown && dpadDown,
                                        dpadUpPressed = !model.dpadUp && dpadUp
                                    ) to emptyList()
                                }
                        } catch (_: Exception) {
                                model to emptyList()
                        }
                }

                is Message.UpdateElevatorPosition -> {
                        /*when (msg.encoder) {
                                CanDevice.Encoder.LeftElevator -> println("Left: ${msg.position}")
                                CanDevice.Encoder.RightElevator -> println("Right: ${msg.position}")
                                else -> {}
                        }*/
                        when (msg.encoder) {
                                CanDevice.Encoder.LeftElevator -> model.copy(
                                        prevLeftElevatorPosition = msg.position,
                                        leftElevatorPosition = updateNetDegrees(
                                                prevAbsDeg = model.prevLeftElevatorPosition,
                                                currAbsDeg = msg.position,
                                                netDeg = model.leftElevatorPosition
                                        )
                                )
                                CanDevice.Encoder.RightElevator -> model.copy(
                                        prevRightElevatorPosition = msg.position,
                                        rightElevatorPosition = updateNetDegrees(
                                                prevAbsDeg = model.prevRightElevatorPosition,
                                                currAbsDeg = msg.position,
                                                netDeg = model.rightElevatorPosition
                                        )
                                )
                                else -> model
                        } to emptyList()
                }
        }
}

fun subscriptions(model: Model): List<Subscription<Message>> {
        val pigeon = CanDevice.Pigeon.CentralPigeon
        return listOf(
                model.hidInputs[0]?.let {
                        listOf(Subscription.HidPortValue(
                                token = it,
                                message = { controllerState -> Message.DriverControllerState(controllerState) },
                        ))
                } ?: emptyList(),
                model.hidInputs[1]?.let {
                        listOf(Subscription.HidPortValue(
                                token = it,
                                message = { controllerState -> Message.DriverControllerState(controllerState) },
                        ))
                } ?: emptyList(),
                listOf(Subscription.RobotStateChanged(
                        message = { prevState, newState -> Message.RobotStateChanged(prevState, newState) }
                )),
                listOf(Subscription.RobotState(
                        message = { state -> Message.RobotState(state) }
                )),
                model.tokens.get(pigeon).unwrap(
                        default = emptyList(), //TODO (max): maybe find a more visually appealing replacement
                        fn = {
                                listOf(Subscription.PigeonValue(
                                        pigeon = it,
                                        millisecondsBetweenReads = 10,
                                        message = { value -> Message.PigeonValue(pigeon, value) }
                                ))
                        }
                )

        ).plus(CanDevice.Encoder.entries.map { encoder ->
                val subscription = model.tokens.get(encoder).unwrap(
                        default = emptyList(),
                        fn = {
                                listOf(Subscription.CANcoderValue(
                                        token = it,
                                        millisecondsBetweenReads = 10,
                                        message = { pos ->
                                                if (elevatorDevices.contains(encoder))
                                                        Message.UpdateElevatorPosition(encoder, pos)
                                                else getSwervePod(encoder).unwrap(
                                                        default = Message.Log("fetching corresponding swerve pod to " +
                                                                "encoder: ${encoder.name}"),
                                                        fn = { pod -> Message.UpdateSwervePodPosition(pod, pos) }
                                                )

                                        }
                                ))
                        }
                )
                subscription
        }).flatten()
}

fun init(args: List<String>): Pair<Model, List<Effect<Message>>> {
        CANBus()

        val model = Model(
                robotMode = RunningRobotState.Disabled,
                talonsInitialized = 0,

                leftBottom = 0.0,
                rightBottom = 0.0,
                calibrated = false,

                activeSongs = emptyMap(),
                loadedSongs = emptyMap(),

                steeringMotorPositions = emptyMap(),
                leftElevatorPosition = -124.0,
                rightElevatorPosition = -16.0,
                prevLeftElevatorPosition = -124.0,
                prevRightElevatorPosition = -16.0,

                heading = 0.0,
                offset = 0.0,

                leftJoystickX = 0.0,
                leftJoystickY = 0.0,
                rightJoystickX = 0.0,
                prevX = false,
                x = false,

                manipulatorLeftJoystickY = 0.0,
                dpadDown = false,
                dpadDownPressed = false,
                dpadUp = false,
                dpadUpPressed = false,

                pidStates = Kraken.entries.associateWith { PIDState() },

                tokens = CanDeviceRegistry(),
                krakens = emptyMap(),
                orchestras = emptyMap(),
                hidInputs = emptyMap()
        )

        val initDevices: List<Effect<Message>> =
                listOf(
                        CanDevice.Motor.Kraken.entries to CanDeviceType.Talon,
                        CanDevice.Motor.Neo.entries    to CanDeviceType.Neo,
                        CanDevice.Encoder.entries      to CanDeviceType.Encoder,
                        CanDevice.Pigeon.entries       to CanDeviceType.Pigeon
                )
                        .flatMap { (entries, type) -> entries.map { it to type } }
                        .map { (device, type) ->
                                Effect.InitCanDevice(
                                        type = type,
                                        id = device.id,
                                        message = { device, id, token -> Message.InitDevice(device, id, token) }
                                )
                        }

        return model to initDevices
}

/**
 * Get the desired speeds of all drive motors for TeleOp drive.
 *
 * @param turningStates a Map of turning motors to their current positions.
 *                      For example, if all wheels are pointed directly forward,
 *                      all values in this map would be PI / 2
 *
 * @param forward A value between -1.0 and 1.0 that represents the forward velocity vector of the robot,
 *                where positive is forward
 *
 * @param strafe A value between -1.0 and 1.0 that represents the strafe velocity vector of the robot,
 *               where positive is right
 *
 * @param rotation A value between -1.0 and 1.0 that represents the angular velocity vector of the robot,
 *                 where positive is clockwise
 *
 * @param heading A value on the interval [-180, 180] that represents the robots current heading,
 *                where 0 is forward
 *
 * @param maxTranslationalSpeed A value between 0 and 1.0 which represents the robots translational speed at full
 *                              throttle, where 0 is stopped and 1.0 is as fast as the robot can go.
 *
 * @param maxRotationalSpeed A value between 0 and 1.0 which represents the robots rotational speed at full
 *                           throttle, where 0 is stopped and 1.0 is as fast as the robot can go.
 *
 * @return a Map of all motors (turning and drive) to their desired speed (between -1.0 and 1.0)
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
        if (abs(forward) < JOYSTICK_TOLERANCE && abs(strafe) < JOYSTICK_TOLERANCE && abs(rotation) < JOYSTICK_TOLERANCE){
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
                SwervePod.FrontLeft  to -45.0,
                SwervePod.FrontRight to -135.0,
                SwervePod.BackLeft   to 45.0,
                SwervePod.BackRight  to 135.0
        )
        val rotationalVelocity = rotation * maxRotationalSpeed

        // Precompute translation vector components (x right, y forward)
        // Angle convention: a=0 -> (x=0,y=+1), a=+90 -> (x=-1,y=0)
        val tX = translationalVelocity * (-sind(translationalDirection))
        val tY = translationalVelocity * ( cosd(translationalDirection))

        val out = mutableMapOf<Kraken, Double>()

        for (m in SwervePod.entries) {
                val currentAngle = turningStates[m] ?: 0.0

                // Rotation vector for this module
                val rDir = rotationalDirections.getValue(m)
                val rX = rotationalVelocity * (-sind(rDir))
                val rY = rotationalVelocity * ( cosd(rDir))

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

                // Output to both steering and drive motors

                out[m.steeringMotor] = steerSpeed

                out[m.driveMotor] = -driveSpeed

        }

        return out
}

// Returns power in order left, right
fun getElevatorPower(joystick: Double, leftPos: Double, rightPos: Double, model: Model) : Pair<Double, Double> {

        fun power(
                motorDir: ElevatorConstants.Direction,
                encoderDir: ElevatorConstants.Direction,
                motorPos: Double,
                bottom: Double,
        ) : Double {
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
                if (!model.calibrated) {
                    return joystickPower
                } else {
                    return if ((abs(pid) < abs(joystickPower) && !isClose) || oppositeSigns) pid else joystickPower
                }
                /*
                 *TODO
                 * For the time being, I have PID Disabled. I am just returning joystick power.
                 * We need to fix "zero-ing" before finishing.
                 */
        }
        return Pair(
                power(
                        motorDir = ElevatorConstants.LEFT_MOTOR_UP_DIR,
                        encoderDir = ElevatorConstants.LEFT_ENCODER_UP_DIR,
                        motorPos = leftPos,
                        bottom = model.leftBottom,
                ),
                power(
                        motorDir = ElevatorConstants.RIGHT_MOTOR_UP_DIR,
                        encoderDir = ElevatorConstants.RIGHT_ENCODER_UP_DIR,
                        motorPos = rightPos,
                        bottom = model.rightBottom,
                ),
        )

}

fun wrapTo180(deg: Double) : Double {
        val angle = ((deg % 360) + 360) % 360
        return if (angle > 180) angle - 360 else angle
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


fun playSong() : List<Effect<Message>> {
        val musicDir = "${Filesystem.getDeployDirectory().absolutePath}/music"
        return Kraken.entries.mapIndexed{ i, motor ->
                Effect.ReadFile(
                        path = "$musicDir/$i.chrp",
                        message = { result -> Message.InitSong(motor to i, result) }
                )
        }
}

fun logError(context: String, error: Maybe<Error>) : List<Effect<Message>> {
        return error.unwrap(emptyList()) {
                val log: String = it.toString()
                listOf(Effect.Log("Error occurred when $context: $log"))
        }
}

fun getSwervePod(encoder: CanDevice.Encoder) : Maybe<SwervePod> {
        return SwervePod.entries.associateBy { it.encoder }[encoder]?.let {
                Maybe.Some(it)
        } ?: Maybe.None
}

fun getDeviceById(id: Int) : Maybe<CanDevice<*>> =
        allDevices.find { it.id == id }?.let { Maybe.Some(it) } ?: Maybe.None

class CanDeviceRegistry private constructor(
        private val backing: Map<CanDevice<*>, CanDeviceToken>
){
        constructor() : this(emptyMap())

        fun plus(device: CanDevice<*>, token: CanDeviceToken) : CanDeviceRegistry =
                CanDeviceRegistry(backing + (device to token)) //TODO (max) re-add type safety (requires platform reworking)

        @Suppress("UNCHECKED_CAST")
        fun <T : CanDeviceToken> get(device: CanDevice<T>): Maybe<T> =
                (backing[device] as? T)?.let { Maybe.Some(it) } ?: Maybe.None

        fun contains(device: CanDevice<*>) = device in backing
}
