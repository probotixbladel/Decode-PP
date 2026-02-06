package org.firstinspires.ftc.teamcode.drive;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name="PedroTeleop Single", group="Linear OpMode")
public class PedroTeleopSingle extends PedroTeleop {
    @Override
    public void init() {
        singlePlayer = true;
        super.init();
    }
}
