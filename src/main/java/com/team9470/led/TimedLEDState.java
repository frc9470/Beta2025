package com.team9470.led;

public class TimedLEDState {
	/** default flash interval for when don't want to specify */
	private static final double kFlashInterval = 0.2;

	/** stays turned off forever */
	public static final TimedLEDState OFF = new TimedLEDState("OFF", Double.POSITIVE_INFINITY, Color.off());
	public static final TimedLEDState DISABLE_BLUE = new TimedLEDState("DISABLE_BLUE", 1.0, Color.GREEN_DIMMED, Color.BLUE_DIMMED);
	public static final TimedLEDState DISABLE_RED = new TimedLEDState("DISABLE_RED", 1.0, Color.GREEN_DIMMED, Color.RED_DIMMED);
	public static final TimedLEDState NO_VISION = new TimedLEDState("NO_VISION", Double.POSITIVE_INFINITY, Color.ORANGE);
	public static final TimedLEDState EMERGENCY = new TimedLEDState("EMERGENCY", kFlashInterval, Color.RED, Color.off());

	public static final TimedLEDState NO_CORAL = new TimedLEDState("NO_CORAL", Double.POSITIVE_INFINITY, Color.RED);
	public static final TimedLEDState HAS_CORAL = new TimedLEDState("HAS_CORAL", Double.POSITIVE_INFINITY, Color.GREEN);
	public static final TimedLEDState CORAL_OBTAINED = new TimedLEDState("CORAL_OBTAINED", kFlashInterval, Color.GREEN, Color.off());

//	// some sort of blinking for aligning
//	public static final TimedLEDState ALIGNING = new TimedLEDState("ALIGNING", kFlashInterval, Color.BLUE, Color.GREEN);
//	// aligned
//	public static final TimedLEDState IS_ALIGNED = new TimedLEDState("IS_ALIGNED", Double.POSITIVE_INFINITY, Color.GREEN);

//	public static final TimedLEDState ELEVATOR_LOADING = new TimedLEDState("ELEVATOR_LOADING", kFlashInterval, Color.RED, Color.off());
//	public static final TimedLEDState ELEVATOR_LOADED = new TimedLEDState("ELEVATOR_LOADED", Double.POSITIVE_INFINITY, Color.BLUE);
//	public static final TimedLEDState FIRING = new TimedLEDState("FIRING", Double.POSITIVE_INFINITY, Color.ORANGE);
//
//	public static final TimedLEDState CONTINUOUS_SHOT = new TimedLEDState("CONTINUOUS_SHOT", 0.2, Color.ORANGE, Color.off());

	public Color[] colors; // array of colors to iterate over
	public double interval; // time in seconds between states
	public String name; // name of state

	public TimedLEDState(String name, double interval, Color... colors) {
		this.colors = colors;
		this.interval = interval;
		this.name = name;
	}
}
