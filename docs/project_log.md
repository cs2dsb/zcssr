# Project log

## 5-Sept-2019

* Breadboard prototyped zero crossing circuit based on [this circuit](SimpleIsolatedZeroCrossDetector.pdf) (originally from drextel.net but the domain has sadly gone dormant). ![breadboard poc](img/breadboard_zc_poc.png)
* There are simpler ZC circuits but this one appears to have some additional benefits, works really well and doesn't add much to the BOM cost
* Arduino POC code using hardware timer to detect ZC edges and ISR to record the timer count
    * Works ok, measures the width and time of the pulse with good precision
    * GPIO output on scope looks good against AC waveform ![gpio output against AC](img/zc_scope_output.png)

## 6-Sept-2019

* Received Couzet SSRs
* Tore one down to make sure they are genuine ![ssr teardown](img/ssr_teardown.png)
* Hooked up SSR output to scope and showed that triggering GPIO in ISR was sufficiently before the ZC to correctly enable the SSR in time for the ZC

## 8-Sept-2019

* Ordered TH_v0.1 boards from JLCPCB

## 25-Sept-2019

* Received TH_v0.1 boards ![Image of TH v0.1 fully assembled](img/th_0.1.jpg)
* Missing pullup on RST. Working as expected other than that
* 14-Segment Adafruit display working nicely over I2C
* Rotary encoder seems like a reasonable input device for the kind of config this needs (pulse1, delay1, pulse2, delay2)
* NTC working

## 7-Oct-2019

* Ordered SMD_v0.1 boards assembled from JLCPCB
* STM32F1 available for assembly as a basic part
* STM32 has richer set of peripherals than ATMEGA328P
    * Timer can do quadrature encoder input with no software interaction
    * Timer can do PWM input capture to measure the frequency and duty of the ZC signal
    * Timer can be configured in one-shot mode to enable the SSR for a given time and have the hardware turn it off regardless of a crashed program or delayed ISR or something
* STM32 can also be programmed with rust and the F1 has pretty good support

## 22-Oct-2019

* Received SMD_v0.1 boards from JLCPCB ![Image of v0.1 from jlc](img/smd_0.1_jlc.jpg)
* Identified issues:
    * Q1 Base and Collector are switched. Resolved with some solder bodging
    * Varistor/Mov leads are too thick for holes. Resolved with some solder bodging
    * Pwr led is annoyingly bright with 150Ω resistor. Had a 680Ω to replace it with but still too bright. Changed to 1k in schematic
    * ZC input on TIM2_CH3 can't be used for PWM input. Fortunately CH1 & CH2 are adjacent so resolved with some solder bodging

## 28-Oct-2019

* Note to self: don't plug the mains in when loose dupont wires are nearby. Destroyed ST-Link and F1 on POC board
* Ordered fuse cover to partially protect mains contacts
* Added longish ribbon with all low voltage outputs and fitted lid of box so nothing can get in by accident

## 3-Nov-2019

* Recieved replacement ST-Link
* Basically all the bits are implemented in rust. Few PRs for stm32f1xx-hal to enable extra timers
* Implemented driver for 14-segment display [adafruit-alphanum4.rs](https://github.com/cs2dsb/adafruit-alphanum4.rs)
    * Added to rust-embedded awesome list which is nice. Hopefully it'll be useful to someone

## 11-Nov-2019

* Did quite a bit of documentation for the repo
* Finished updating SMD_v0.2 PCB ready to order
    * Added MAX31855K K-Type thermocouple interface IC to enable reflow oven use case
    * Added USB port, mainly as an experiment to see if I can get it working and useful (USB DFU mode would be amazing but failing that, something similar to the TS-100 USB functionality would be ok)
    * Added a physical reset button. In theory, SWD doesn't need hardware reset and openocd should be able to connect to the core with just CLK, DIO and GND but in practice I've had boards fairly regularly get into a state where it can't connect reliably. Resetting by shorting NRST to GND resolves this and the board will work without reset until the ST-Link is unplugged or the boards power is cycled. I haven't been able to get to the bottom of it - the core is running, the gpio for SWD hasn't been reconfigured and I've tried every permutation of reset_config in openocd. There's some speculation on some forums that the ST-Link firmware is to blame as it's not a proper JTAG adapter and merely emulates some features; I could buy a proper JTAG adapter to rule this out but I want these boards to be programmable by anyone with a cheap adapter. This programming faff partially fed into the desire to add USB and explore USB bootloading. 

## 13-Nov-2019

* Changed AC-DC module from 3.3V to 5V to allow 5V SSR (or relay) output
* Switched LDO to AP2112 as used on several Adafruit boards. It has better specs and doesn't require tantilum caps to be stable
* 5V in allows using simple diodes to block backfeeding AC-DC or USB 5V inputs
* Added transistors to control SSR and MISC outputs so they can be 5V rather than directly driven from MCU GPIO pins

## 15-Nov-2019

* Added tiny 1x1mm rgb LED to spare pins near the existing LED. Mainly want to see how bright this tiny LED is at ~5ma

## 16-Nov-2019

* Placed an order for the next version of the PCB

## 29-Nov-2019

* v0.2 PCBs arrived
* Updated the board pin hardware definitions
* Tested all the previously implemented features
    * It all still works
* RGB led added in this version is oriented incorrectly
* Haven't tested USB

## 9-Jan-2020

* Finished implementing [MAX31855](https://github.com/cs2dsb/max31855.rs) and testing thermocouple

## 14-Jan-2020

* Tested RTFM 5: issue I was previously seeing with hardware interrupts not getting called appears to be resolved
* Converted spot_welder to use RTFM 5 and some general housekeeping

## 19-Jan-2020

* Fitted the new hardware into the case and hooked up a transformer to test the ZC and mains side of things
* All working as expected - probably shouldn't be surprising but after taking a 2 month break I automatically assume all the hardware and code was rubbish and wouldn't work.

## 25-Jan-2020

* Brought all the tested bits a pieces together into basically finished spot welder
    * UI settings are snapshotted when the weld is triggered
    * Oscilloscope shows pulse lengths match configured values
* Refactored the UI out into zcssr lib
    * I originally thought it might be a useful reusable thing but actually a 4 digit display controlled with a rotary encoder is probably too specific to be much use elsewhere. Maybe it can be extracted later; some of the code like scrolling text might be useful...
    * As part of the refactoring I converted the display strings into GenericArray<AsciiChar, typenum::Uxx> and got rid of the current text buffer. This means there's no copying the strings when changing screens and each string is only as long as necessary not some MAX_LENGTH constant. I haven't worked out the memory savings but it feels better to have a zero cost, no copying version.
    * Experimented with using trait objects/dynamic dispatch to make the Ui/Screen api nicer but found that dynamic dispatch introduced a measurable (oscilloscope on toggled IO pin) delay of several hundred cycles. Since I'm matching on the current screen anyway static dispatch isn't much more of a hassle but does prevent something clean like self.with_current_screen(|screen| screen.do_the_thing()). To reduce duplicated match (self.current_screen) blocks I made a macro to plop the match around a closure - it basically mimics the api I would have done with trait objects except the closures are manually monomorphised by the macro.
* Trigger cable is picking up noise and causing the interrupt to run erroneously
    * Added hardware debouncing to the trigger input
    * Since a false interrupt on the trigger could cause a weld at an inopportune time I'll have to revisit it once the HW debouncing is in to see if I need to add a software solution on top - requiring a press longer than a given time and not occuring again within a given time would be fairly easy to implement and could provide more safety.
* Related to trigger noise, I'm also seeing erroneous data on the UART dongle I'm using when big mains devices turn on/off in the same room (electic heater, lights, printer). This happens when the mains plug for the ZCSSR isn't connected and everything is powered over USB so I don't really know what the exact cause is - the UART dongle has a high quality shielded cable, maybe it's noise being conducted to the USB ground? Who knows...

## 26-Jan-2020

* Moved majority of the hardware configuration out of spot_welder (the bin) into the crate board.rs
    * This should mean that creating new bins for reflow_oven, sous_vide, etc. will reuse most of this code and be fairly trivial - just the control algorithms for those specifc devices
    * The pattern I went for is the board has a trait `Configure` which it implements for the type aliases it defines for the hardware. The consumer then does `LedStatus::configure((gpiox.px1, etc))` to get a configured `LedStatus` back. The pattern of having an associated type on the trait that specifies the requred parameters was nicked from amethyst.rs's system trait. I'm not sure if this gives any benefit over a simple function configure_led_status(...)

## 27-Jan-2020

* Took the I2C hung fix I hacked into local copy of the hal out and put it in a separate crate: [i2c_hung_fix](https://github.com/cs2dsb/i2c_hung_fix.rs) since it only needs embedded-hal InputPin and Output pin and should work for anything that implements those on the I2C pins
* Moved the UI string to GenericArray code into a proc_macro so that the length of the array is correctly enforced. It still uses unsafe union transmuting but both sides lengths are calculated in the proc macro instead of manually entered so it should be safe enough. Once core::mem::transmute becomes a const_fn this will be unnecessary [see #53605](https://github.com/rust-lang/rust/issues/53605)