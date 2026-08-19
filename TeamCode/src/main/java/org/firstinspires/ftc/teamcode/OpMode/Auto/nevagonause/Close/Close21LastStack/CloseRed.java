package org.firstinspires.ftc.teamcode.OpMode.Auto.nevagonause.Close.Close21LastStack;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Autonomous(name = "Auto RED 21 Last Stack ")
public class CloseRed extends Close18LastStack {
    @Override
    public void init() {
        Info.alliance = Alliance.RED;

        super.init();
    }

}