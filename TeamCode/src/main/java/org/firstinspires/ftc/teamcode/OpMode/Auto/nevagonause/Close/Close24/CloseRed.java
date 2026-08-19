package org.firstinspires.ftc.teamcode.OpMode.Auto.nevagonause.Close.Close24;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Autonomous(name = "Auto Red 24 (cu clearuri)")
public class CloseRed extends Close24 {
    @Override
    public void init() {
        Info.alliance = Alliance.RED;

        super.init();
    }

}