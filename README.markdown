Flat Mate - a battery voltage monitor with ATTiny45
===================================================

This project monitors the voltage of a battery and displays a
5-element bar-graph indicating battery charge state.  Optionally
MOSFET control may be used to cut off supply to protect the battery
from over-discharge.

It is particularly applicable for multi-cell Lithium-ion Polymer
(LiPo) packs, which are easily ruined if over-discharged.

It is intended for use in high power applications such as portable
lighting, robotics and power tools where a charged SLA or LiPo battery
is discharged in the presence of a human operator.  The first
deployment was for a high-power bicycle headlamp.  Design was inspired
by the simple state-of-charge indicators present on some electric
bicycles.

The advantage over existing inexpensive "off the shelf" protection
circuits is that this circuit gives a visual indication of
state-of-charge, presented as an easy-to-read bar-graph.  Existing
protection circuits (often designed for flashlights or model aircraft)
either have no display (sudden surprise cutoff), digital voltage
display (requires memorizing cutoff voltages) or audio-only alert (no
visual display).

An Atmel ATTiny45 microcontroller is used to sample the battery
voltage and control the LED bargraph.  Power for the microcontroller
is derived from a 78L05 regulator connected to the main battery.
Battery voltage is monitored using an analog input via a voltage
divider and compared to the microcontroller's 1.1v internal voltage
reference.

The bargraph will show 4 LEDs when at "full charge" and 
1 LED when at minimum usable voltage.  When voltage is critical
a 5th "alert" LED will illuminate (and output supply can be terminated).

The circuit can be used purely as a voltage indicator by leaving off
the optional MOSFET switch components.  If a MOSFET is fitted this can
be used to disconnect the main load when the battery is depleted.

Hardware Details
----------------

![schematic](hw/flatmate.png)

The bargraph consists of 5 3mm LEDs (green, yellow and red) arranged
GYYYR.  The first four (GYYY) form the bargraph and the 5th (red) is
the critical alert.  The first three are connected to dedicated output
pins, the final two share a single pin in a totem-pole arrangement.
The fourth LED shares state with the output FET (if fitted), when this
LED is lit, power is good and the FET is enabled.  When this output
pin is cleared, the fifth LED (the complementary red CRITICAL
alert) is illuminated and the FET is disabled.

As the monitor circuit is indended for use in high-drain situations
(0.5--10A), the approx 20mA consumed by the monitoring circuit is
considered negligible.  In particular the voltage divider is not
isolated from supply between samples, so presents a constant drain of
around 850uA.  Current through the bargraph elements is limited by a
1kΩ resistor to around 3mA per element, so the bargraph consumes up to
12mA when at FULL level.


Software Details
----------------

Software is configurable for the type and number of cells.   The
initial application of a 3-cell (11.4v nominal) Lithium-ion Polymer
(LiPo) battery will be used to describe the code.

Four voltage thresholds are configured (in millivolts), depending on battery type.
For a 3S LiPo these are:

	    BMV_FULL = 12.000v
	    BMV_GOOD = 11.000v
	    BMV_LOW  = 10.000v
	    BMV_CRIT =  9.000v

The code converts these thresholds to analog sample values at compile
time, defining corresponding VL_FULL..CRIT constants.

These thresholds give the following LED readouts:

	    if >= FULL output GYYY_
	    if >= GOOD output _YYY_
	    if >= LOW  output __YY_
	    if >= CRIT output ___Y_
	    if  < CRIT output ____R

### Setup

   * Configure analog pin as input
   * Configure LED pins as outputs
   * Strobe the LEDs for 1s to see that they all work
   * Configure timer 1 to give 4Hz interrupts
   * Configure ADC for internal 1v1 reference, selecting single-ended channel 2 
   * Set ADC clock prescaler to give 125kHz, take and discard one sample

### Main loop

   * do nothing, go straight to sleep mode until timer interrupt.

### Timer interrupt handler

   * Read battery voltage
      * initiate conversion
      * busy-wait until conversion complete
      * read completed value
   * Update sample value
      * add sample value to accumulator
      * if 4 samples are accumulated, calculate the average and reset accumulator
   * Update the display whenver accumulator is reset
      * Clear all LEDs
      * Light the CRITICAL LED (and kill power) if level is <=VL_CRIT
      * Set the bargraph appropriately if level is >VL_CRIT (see above)

Since samples are accumulated at 4Hz, and each 4 samples are averaged,
the outputs will be changed only once per second.  Averaging is used
to avoid falsely detecting a CRITICAL state when transient spikes such
as motor start occur.

Construction
============

The circuit was prototyped on a solderless breadboard, with battery
simulated using a 24v supply and an adjustable-output regulator
(LM317T).

A selection of LEDs with one leg replaced by a current-limiting
resistor have been made to simplify use of LEDs in breadboard
prototypes.

For in-circuit programming I use the tuxgraphics AVRUSB500v2, as this
programmer features a 5-pin SIL programming header, which is very
convenient for breadboards.   

Once the design was proven, a version was constructed on through-hole
prototyping board with components arranged to take advantage of
pin-bridging where possible.  This turned out to be difficult since
the chosen double-sided prototype board featured a solder mask that
was *VERY* resistant to bridging (this would normally be a laudable
feature).  A plain copper single-sided protoboard would have actually
been a better choice.

The prototype has been deployed for use as a bicycle headlamp battery
pack.

Subsequent revisions will use ATTiny25 in SMD package, and SMD
components.

