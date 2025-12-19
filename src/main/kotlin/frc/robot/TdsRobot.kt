package frc.robot

import teaforge.platform.RoboRio.RoboRioProgram

data class Model(
        val control: ControlSubsystem.Model
)

sealed interface Message {
        data class Control(val message: ControlSubsystem.Message) : Message
}

val tieDieSamuraiRobot =
        RoboRioProgram(
                init = ::init,
                update = ::update,
                subscriptions = ::subscriptions,
        )

fun update(msg: Message, model: Model) =
        when (msg) {
                is Message.Control -> {
                        val (newControl, effects) = ControlSubsystem.update(msg.message, model.control)
                        Model(newControl) to effects
                }
        }

fun subscriptions(model: Model) =
        ControlSubsystem.subscriptions(model.control)

fun init(args: List<String>): Pair<Model, List<teaforge.platform.RoboRio.Effect<Message>>> {
        val (controlModel, effects) = ControlSubsystem.init(args)
        return Model(controlModel) to effects
}
