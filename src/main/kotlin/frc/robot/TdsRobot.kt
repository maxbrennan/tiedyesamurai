package frc.robot

import com.ctre.phoenix6.CANBus
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.wpilibj.Filesystem
import frc.robot.subsystems.*
import teaforge.platform.RoboRio.*
import teaforge.utils.*
import kotlin.math.abs

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
        val operator: OperatorInterfaceState,
        val swerve: SwerveState,
        val elevator: ElevatorState,
        val music: MusicState,
        val pidStates : Map<Kraken, PIDState>,
        val tokens: CanDeviceRegistry,
        val orchestras: Map<Kraken, OrchestraToken>,
        val hidInputs: Map<Int, HidInputToken>,
)

sealed interface Message {
        data class InitDevice(val type: CanDeviceType, val id: Int, val token: Result<CanDeviceToken, Error>) : Message
        data class RobotState(val state: RunningRobotState) : Message
        data class RobotStateChanged(val prevState: RunningRobotState, val newState: RunningRobotState) : Message

        data class LogError(val context: String, val error: Maybe<Error>) : Message
        data class LogErrors(val errors: List<Error>) : Message
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
                        val operator = model.operator.updateDriver(msg.controllerState)
                        model.copy(operator = operator) to emptyList()
                }
                is Message.ManipulatorControllerState -> {
                        val operator = model.operator.updateManipulator(msg.controllerState)
                        val elevator = model.elevator.calibrateIfRequested(operator.manipulator)
                        model.copy(operator = operator, elevator = elevator) to emptyList()
                }
                is Message.RobotState -> handleRobotState(msg.state, model)
                is Message.RobotStateChanged -> handleRobotStateChanged(msg, model)
                is Message.InitSong -> handleInitSong(msg, model)
                is Message.ProcessLoadedSong -> handleProcessLoadedSong(msg, model)
                is Message.ActivateSong -> handleActivateSong(msg, model)
                is Message.RestartSong -> handleRestartSong(msg, model)
                is Message.LogError -> model to logError(msg.context, msg.error)
                is Message.LogErrors -> {
                        model to msg.errors.map { Effect.Log(it.toString()) }

                }
                is Message.Log -> model to listOf(Effect.Log(msg.str))

                is Message.InitDevice -> handleInitDevice(msg, model)

                is Message.UpdateSwervePodPosition -> {
                        val swerve = model.swerve.updateWithPodPosition(msg.pod, msg.position)
                        model.copy(swerve = swerve) to emptyList()
                }

                is Message.PigeonValue -> {
                        val swerve = model.swerve.updateHeading(msg.value)
                        model.copy(swerve = swerve) to emptyList()
                }

                is Message.UpdateElevatorPosition -> {
                        val elevator = model.elevator.updatePosition(msg.encoder, msg.position)
                        model.copy(elevator = elevator) to emptyList()
                }
        }
}

private fun handleRobotState(state: RunningRobotState, model: Model): Pair<Model, List<Effect<Message>>> {
        val (updatedModel, effects) = when (state) {
                RunningRobotState.Teleop -> handleTeleop(model)
                else -> model to emptyList()
        }

        return if (updatedModel.music.talonsInitialized == 8)
                updatedModel.copy(music = updatedModel.music.copy(talonsInitialized = 0)) to (effects + playSong())
        else updatedModel to effects
}

private fun handleTeleop(model: Model): Pair<Model, List<Effect<Message>>> {
        val swerve = model.swerve.recalibrateIfRequested(model.operator.driver)

        val driveMotorSpeeds: Map<Kraken, Double> = getDriveMotorSpeeds(
                turningStates = swerve.steeringMotorPositions,
                forward = swerve.operatorForward(model.operator),
                strafe = swerve.operatorStrafe(model.operator),
                rotation = swerve.operatorRotation(model.operator),
                heading = swerve.heading,
                maxTranslationalSpeed = 0.2,
                maxRotationalSpeed = 0.1,
        )

        val movingMotors = driveMotorSpeeds.filter { abs(it.value) > JOYSTICK_TOLERANCE }.keys
        val (music, stopSongEffects) = stopSongsForMotors(movingMotors, model)

        val swerveEffects: List<Effect<Message>> = driveMotorSpeeds.flatMap { entry ->
                model.tokens.get(entry.key).unwrap(
                        default = listOf(Effect.Log(
                                "Failed to fetch drive state of ${entry.key.name}"
                        )),
                        fn = { token ->
                                listOf(Effect.SetCanMotorSpeed(token, entry.value))
                        }
                )
        }

        val power = model.elevator.getElevatorPower(model.operator.manipulator.leftJoystickY)
        val elevatorEffects = listOf(
                model.tokens.get(CanDevice.Motor.Neo.Left).unwrap(
                        default = Effect.Log("Failed to fetch neo LEFT"),
                        fn = { token -> Effect.SetCanMotorSpeed(token, power.first) }
                ),
                model.tokens.get(CanDevice.Motor.Neo.Right).unwrap(
                        default = Effect.Log("Failed to fetch neo RIGHT"),
                        fn = { token -> Effect.SetCanMotorSpeed(token, power.second) }
                ))

        val updatedModel = model.copy(
                swerve = swerve,
                music = music,
        )

        return updatedModel to (swerveEffects + elevatorEffects + stopSongEffects)
}

private fun SwerveState.operatorForward(operator: OperatorInterfaceState): Double = operator.driver.leftJoystickY
private fun SwerveState.operatorStrafe(operator: OperatorInterfaceState): Double = operator.driver.leftJoystickX
private fun SwerveState.operatorRotation(operator: OperatorInterfaceState): Double = operator.driver.rightJoystickX

private fun stopSongsForMotors(motors: Set<Kraken>, model: Model): Pair<MusicState, List<Effect<Message>>> {
        if (motors.isEmpty()) return model.music to emptyList()

        val effects = motors.mapNotNull { motor ->
                model.music.activeSongs[motor]?.let {
                        model.orchestras[motor]?.let { orchestra ->
                                Effect.StopSong(
                                        token = orchestra,
                                        message = { result ->
                                                val error = if (result is Result.Error) Maybe.Some(result.value) else Maybe.None
                                                Message.LogError(
                                                        context = "stopping song for ${motor.name}",
                                                        error = error
                                                )
                                        })
                        } ?: Effect.Log("Failed to fetch orchestra for ${motor.name}")
                }
        }

        val updatedMusic = model.music.copy(
                activeSongs = model.music.activeSongs.filterKeys { it !in motors },
                loadedSongs = model.music.loadedSongs.filterKeys { it !in motors }
        )
        return updatedMusic to effects
}

private fun handleRobotStateChanged(msg: Message.RobotStateChanged, model: Model): Pair<Model, List<Effect<Message>>> {
        val effects =
                if (msg.newState == RunningRobotState.Disabled) {
                        if (model.music.activeSongs.isEmpty()) playSong()
                        else model.music.activeSongs.map { entry ->
                                val motor = entry.key
                                model.tokens.get(motor).unwrap(
                                        default = listOf(Effect.Log("Failed to stop song on ${motor.name}")),
                                        fn = { _ ->
                                                model.orchestras[motor]?.let { orchestra ->
                                                        listOf(
                                                                Effect.StopSong(
                                                                        token = orchestra,
                                                                        message = { result ->
                                                                                val error = if (result is Result.Error) Maybe.Some(result.value) else Maybe.None
                                                                                Message.RestartSong(
                                                                                        motor = entry.toPair(),
                                                                                        error = error
                                                                                )
                                                                        }
                                                                )
                                                        )
                                                } ?: listOf(Effect.Log("Failed to find orchestra for ${motor.name}"))
                                        }
                                )
                        }.flatten()

                } else {
                        emptyList()
                }

        val newModel = if (msg.newState == RunningRobotState.Disabled)
                model.copy(
                        robotMode = msg.newState,
                        music = model.music.resetLoaded()
                ) else model.copy(robotMode = msg.newState)

        return newModel to effects + Effect.Log("State changed to ${msg.newState.name}")
}

private fun handleInitSong(msg: Message.InitSong, model: Model): Pair<Model, List<Effect<Message>>> {
        val motor = msg.motor.first
        return model to when (msg.song) {
                is Result.Success -> {

                        listOf(model.tokens.get(motor).unwrap(
                                default = Effect.Log("Failed to init song on ${motor.name}"),
                                fn = { token -> Effect.LoadSong(
                                        motor = token,
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

private fun handleProcessLoadedSong(msg: Message.ProcessLoadedSong, model: Model): Pair<Model, List<Effect<Message>>> {
        val logErrorEffect: List<Effect<Message>> = logError(
                context = "loading song for motor ${msg.motor}",
                error = msg.error
        )
        if (logErrorEffect.isNotEmpty()) return model to logErrorEffect

        val loadedSongs = model.music.loadedSongs + msg.motor
        val effects = if (loadedSongs.keys.containsAll(Kraken.entries)) {
                loadedSongs.map { entry ->
                        val motor = entry.key
                        model.tokens.get(motor).unwrap(
                                default = Effect.Log("Couldn't load song on ${motor.name}"),
                                fn = { _ ->
                                        model.orchestras[motor]?.let { orchestra ->
                                                Effect.PlaySong(
                                                        token = orchestra,
                                                        message = { result ->
                                                                val error = if (result is Result.Error) Maybe.Some(result.value) else Maybe.None
                                                                Message.ActivateSong(entry.toPair(), error)
                                                        }
                                                )
                                        } ?: Effect.Log("Failed to find orchestra for ${motor.name}")
                                }
                        )
                }
        } else emptyList()

        return model.copy(music = model.music.copy(loadedSongs = loadedSongs)) to effects
}

private fun handleActivateSong(msg: Message.ActivateSong, model: Model): Pair<Model, List<Effect<Message>>> {
        val effects = logError(
                context = "playing song on motor ${msg.motor.first}",
                error = msg.error
        )

        return if (effects.isNotEmpty()) model to effects
        else model.copy(music = model.music.copy(activeSongs = model.music.activeSongs + msg.motor)) to effects
}

private fun handleInitDevice(msg: Message.InitDevice, model: Model): Pair<Model, List<Effect<Message>>> {
        val mismatchError: List<Effect<Message>> = listOf(Effect.Log("Could not find CAN device" +
                " of type ${msg.type} with id ${msg.id}"))
        return when (msg.token) {
                is Result.Success -> {
                        getDeviceById(msg.id).unwrap(
                                default = model to mismatchError,
                                fn = { device ->
                                        val log = "Successfully registered " +
                                                "${msg.type} at CAN " +
                                                msg.id
                                        val registry = model.tokens.plus(device, msg.token.value)
                                        val musicState = if (msg.type == CanDeviceType.Talon) {
                                                model.music.incrementTalonsInitialized()
                                        } else model.music
                                        val newModel = model.copy(
                                                tokens = registry,
                                                music = musicState
                                        )
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

private fun handleRestartSong(msg: Message.RestartSong, model: Model): Pair<Model, List<Effect<Message>>> {
        val musicDir = "${Filesystem.getDeployDirectory().absolutePath}/music"
        val log = logError("stopping song for ${msg.motor.first.name}", msg.error)
        return model to (log + Effect.ReadFile(
                path = "$musicDir/${msg.motor.second}.chrp",
                message = { result -> Message.InitSong(msg.motor, result) }
        ))
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
                                message = { controllerState -> Message.ManipulatorControllerState(controllerState) },
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
                        fn = { token ->
                                listOf(Subscription.PigeonValue(
                                        pigeon = token,
                                        millisecondsBetweenReads = 10,
                                        message = { value -> Message.PigeonValue(pigeon, value) }
                                ))
                        }
                )

        ).plus(CanDevice.Encoder.entries.map { encoder ->
                val subscription = model.tokens.get(encoder).unwrap(
                        default = emptyList(),
                        fn = { token ->
                                listOf(Subscription.CANcoderValue(
                                        token = token,
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
                operator = OperatorInterfaceState(),
                music = MusicState(),
                swerve = SwerveState(),
                elevator = ElevatorState(),
                pidStates = Kraken.entries.associateWith { PIDState() },
                tokens = CanDeviceRegistry(),
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

fun getSwervePod(encoder: CanDevice.Encoder) : Maybe<SwervePod> {
        return SwervePod.entries.associateBy { it.encoder }[encoder]?.let {
                Maybe.Some(it)
        } ?: Maybe.None
}

fun getDeviceById(id: Int) : Maybe<CanDevice<*>> =
        allDevices.find { it.id == id }?.let { Maybe.Some(it) } ?: Maybe.None
