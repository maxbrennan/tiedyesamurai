package frc.robot.subsystems

import edu.wpi.first.wpilibj.Filesystem
import frc.robot.ControlMessage
import frc.robot.ControlSubsystem
import frc.robot.Kraken
import teaforge.platform.RoboRio.Effect
import teaforge.platform.RoboRio.Error
import teaforge.utils.Maybe

data class MusicState(
    val talonsInitialized: Int = 0,
    val loadedSongs: Map<Kraken, Int> = emptyMap(),
    val activeSongs: Map<Kraken, Int> = emptyMap(),
)

fun MusicState.incrementTalonsInitialized(): MusicState =
    copy(talonsInitialized = talonsInitialized + 1)

fun MusicState.resetLoaded(): MusicState = copy(loadedSongs = emptyMap(), activeSongs = emptyMap())

fun playSong(): List<Effect<ControlMessage>> {
    val musicDir = "${Filesystem.getDeployDirectory().absolutePath}/music"
    return Kraken.entries.mapIndexed { i, motor ->
        Effect.ReadFile(
            path = "$musicDir/$i.chrp",
            message = { result -> ControlMessage(ControlSubsystem.Message.InitSong(motor to i, result)) }
        )
    }
}

fun logError(context: String, error: Maybe<Error>): List<Effect<ControlMessage>> {
    return when (error) {
        is Maybe.None -> emptyList()
        is Maybe.Some -> {
            val log: String = error.value.toString()
            listOf(Effect.Log("Error occurred when $context: $log"))
        }
    }
}
