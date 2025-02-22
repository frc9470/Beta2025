package com.team9470.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleControlFrame;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.team9470.Ports;
import com.team9470.led.Color;
import com.team9470.led.TimedLEDState;
import com.team9470.subsystems.vision.Vision;
import com.team9470.util.AllianceFlipUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private static LEDs mInstance;

    public static LEDs getInstance() {
        if (mInstance == null) {
            mInstance = new LEDs();
        }
        return mInstance;
    }

    // TODO: verify if we actually have this many LEDs
    private final int kNumLeds = 300;

    private boolean mDisabled = false;
    private final CANdle mCandle = new CANdle(Ports.CANdle.getDeviceNumber(), Ports.CANdle.getBus());
    private LEDSection mLEDStatus = new LEDSection(0, kNumLeds);

    public boolean hasCoral = false;

    public LEDs() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = true;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 1.0;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        mCandle.configAllSettings(configAll, 5000);
        mCandle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 255);
        mCandle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_1_General, 10);
        mCandle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_2_ModulatedVBatOut, 255);
        applyStates(TimedLEDState.DISABLE_BLUE);
    }

    @Override
    public void periodic() {
        readPeriodicInputs();
        outputTelemetry();
    }

    public void readPeriodicInputs() {
        /** setting specific states */
        if (mDisabled) {
            if (!Vision.getInstance().isFullyConnected()) {
                applyStates(TimedLEDState.NO_VISION);
            } else {
                if (AllianceFlipUtil.shouldFlip()) {
                    applyStates(TimedLEDState.DISABLE_RED);
                } else {
                    applyStates(TimedLEDState.DISABLE_BLUE);
                }
            }
        } else {
            if (hasCoral) {
                applyStates(TimedLEDState.HAS_CORAL);
            } else {
                applyStates(TimedLEDState.NO_CORAL);
            }
        }

        // TODO: should this be wrapped in an else statement?

        /** setting LEDs based on the state */

        double timestamp = Timer.getFPGATimestamp();
        
        // TODO: consider moving to a function within LEDSection?
        if (mLEDStatus.state.interval != Double.POSITIVE_INFINITY) {
            if (timestamp - mLEDStatus.lastSwitchTime >= mLEDStatus.state.interval) {
                mLEDStatus.nextColor();
                mLEDStatus.lastSwitchTime = timestamp;
            }
        }

        // TODO: also consider moving to within LEDSection
        Color color = mLEDStatus.getWantedColor();
        mCandle.setLEDs(color.r, color.g, color.b, 0, mLEDStatus.startIDx, 100);
    }

    // setter functions
    public void applyStates(TimedLEDState TimedState) {
        mLEDStatus.setState(TimedState);
    }


    public void outputTelemetry() {
        SmartDashboard.putString("LED Status", mLEDStatus.state.name);
        SmartDashboard.putString("LED Colors", mLEDStatus.getWantedColor().toString());
    }

    /**
     * Updates the LEDs to a specific color or animation.
     *
     * @param wanted_state Wanted LED color/animation.
     */
    public Command stateRequest(TimedLEDState wanted_state) {
        return new Command() {

            @Override
            public void execute() {
                applyStates(wanted_state);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    // Class for holding information about each section
    private class LEDSection {
        private TimedLEDState state = TimedLEDState.OFF; // current TimedState
        private double lastSwitchTime = 0.0; // timestamp of last color cycle
        private int colorIndex = 0; // tracks current color in array
        private int startIDx, LEDCount; // start and end of section

        public LEDSection(int startIndex, int endIndex) {
            startIDx = startIndex;
            
            // TODO: what does this line of code do
            LEDCount = endIndex - startIndex;
        }

        public void setState(TimedLEDState wantedTimedState) {
            if (wantedTimedState != state) {
                colorIndex = 0;
                lastSwitchTime = Timer.getFPGATimestamp();
                state = wantedTimedState;
            }
        }

        public Color getWantedColor() {
            Color color;
            try {
                color = state.colors[colorIndex];
            } catch (Exception e) {
                // TODO: replace with N/A enum? what if color.off is a requested color
                color = Color.off();
            }
            return color;
        }

        // cycle to next color in array
        public void nextColor() {
            if (state.colors.length == 1) {
                return;
            }

            /* looping logic */
            if (colorIndex == state.colors.length - 1) {
                colorIndex = 0;
            
            } else {
                colorIndex++;
            }
        }

        // TODO: is this used?

        /** resets class to "factory" settings */
        public void reset() {
            state = TimedLEDState.OFF;
            lastSwitchTime = 0.0;
            colorIndex = 0;
        }
    }
}
