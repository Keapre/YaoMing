package org.firstinspires.ftc.teamcode.Util.Wrapper;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.Controllers.RingBuffer;
import org.firstinspires.ftc.teamcode.Util.Math.Debouncer;


public class DigitalWrapper {
    private static final int BUFFER_SIZE = 3;

    private final Debouncer debouncer;
    private final DigitalChannel device;
    private final RingBuffer<Boolean> ringBuffer;
    private boolean state = false;

    public DigitalWrapper(HardwareMap hardwareMap, String name) {
        device = hardwareMap.get(DigitalChannel.class, name);
        device.setMode(DigitalChannel.Mode.INPUT);

        debouncer = new Debouncer(0.15, Debouncer.DebounceType.kBoth);
        ringBuffer = new RingBuffer<>(BUFFER_SIZE, false);
    }

    public boolean getValue() {
        boolean debounced = debouncer.calculate(getRaw());
        ringBuffer.getValue(debounced);
        if (ringBuffer.allValuesSame()) {
            state = debounced;
        }
        return state;
    }
    public boolean getRaw() {
        return device.getState();
    }
}