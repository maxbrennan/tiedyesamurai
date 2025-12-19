package frc.robot

import frc.robot.button.ButtonSubsystem
import teaforge.platform.RoboRio.*

val tieDieSamuraiRobot =
        RoboRioProgram(
                init = ::init,
                update = ::update,
                subscriptions = ::subscriptions,
        )

data class Model(
        val buttonModel: ButtonSubsystem.Model
)

sealed interface Message {
        data class Button(val message: ButtonSubsystem.Message) : Message
}

fun update(msg: Message, model: Model): Pair<Model, List<Effect<Message>>> {
        when (msg) {
                is Message.Button -> {
                        val result = ButtonSubsystem.update(msg.message, model.buttonModel)
                        return model.copy(buttonModel = result.first) to result.second
                }
        }
}


fun subscriptions(model: Model): List<Subscription<Message>> {
        return ButtonSubsystem.subscriptions(model.buttonModel)
}

fun init(args: List<String>): Pair<Model, List<Effect<Message>>> {
        val init = ButtonSubsystem.init(args)
        return Model(buttonModel = init.first) to init.second
}
