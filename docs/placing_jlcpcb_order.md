# Placing an order for assembled boards with JLCPCB

Make sure you've run through [Generating gerbers and fab outputs](generating_gerbers_and_fab_outputs.md) first.

1. Head over to [JLCPCB](https://jlcpcb.com/quote) and log in/register
2. Press `Add your gerber file`
    
    ![Add gerber button](img/jlcpcb_add_gerber_button.png)
3. Upload `print/print.zip` 
4. Check the options to make sure they are compatible with SMT assembly:
    * Layers: 2
    * PCB Thickness: 1.6
    * PCB Color: Green
    * Copper Weight: 1oz
    * Panel By JLCPCB: No
    * Gold Fingers: No
    * Castellated Holes: No
    * Different Design: No
    * **Specify Order Number: Yes**
5. Enable `Assemble your PCB boards`

    ![Assemble your PCB boards](img/jlcpcb_assemble_your_boards.png)
6. Select the top side for assembly
7. Press `Confirm`
8. Press `Add BOM file`
    
    ![Add bom file](img/jlcpcb_add_bom_file.png)
9. Upload `print/zero_crossing_detector-bom.csv`
10. Press `Add CPL file`
    
    ![Add cpl file](img/jlcpcb_add_cpl_file.png)
11. Upload `print/zero_crossing_detector-top-pos.csv`
12. Press `Next`
13. Check and confirm every part against the BOM
    * It's pretty good and matching most but occasionally you'll have to pick manually
    * There are currently no connectors in the parts database so the USB and other connectors will have to be ordered separately and manually installed
14. Press `Next`
15. Check the Pin 1 markers (red dots) in the `Review Parts Placement` preview
16. Press `Save to cart`
17. Checkout

## LCSC Parts
There are some parts that JLCPCB can't assemble which can be ordered on LCSC.

If you registered with LCSC before 12-Mar-2019 you can get a discount/free shipping on LCSC parts when you have a JCPCB order waiting production. [See for details](https://support.lcsc.com/article/24-do-you-offer-combine-shipment-with-pcbs)

These parts need to be ordered (check BOM in case this list gets missed from an update):
* C52028 - MAX31855
* C89120 - Phoenix Contact 1711725 (Mains input)
* C89122 - Phoenix Contact 1751248 (Thermocouple input)
* C206887 - Littlefuse fuse holder
* C187509 - Littlefuse fuse cover
* C343641 - Header pins (note these need to be cut down, they don't stock a 28 position one currently)
* C136145 - NTC thermistor
* C140500 - 500mA slow blow fuse
* C209904 - 3.3V AC-DC power module
* C233184 - Varistor
* C361165 - ALPS rotary encoder (This one is detentless and doesn't have a threaded mounting column. There are a dozen different styles if you search for ALPS rotary encoders)

## Other Parts
* [Enclosure](https://www.banggood.com/100x68x50mm-IP65-Waterproof-Electronic-Project-Enclosure-Case-DIY-Enclosure-Instrument-Case-p-1260023.html?rmmds=myorder&cur_warehouse=CN) - the pcb is designed to fit in this box and there's **probably** enough space to fit the display, rotary encoder, LEDs and maybe even the SSR inside too. Check the [docs folder](..) for assembly details once the next version of the hardware has been tested
* [Display](https://www.banggood.com/5pcs-4-bit-Pozidriv-0_54-Inch-14-segment-LED-Digital-Tube-Module-Green-I2C-Control-2-line-Control-LED-Display-Screen-Module-p-1565722.html?rmmds=search&cur_warehouse=CN) - the firmware expects this display but you could modify it to accept any I2C display
* [STLink](https://www.banggood.com/3pcs-3_35V-XTW-ST-LINK-V2-STM8STM32-Simulator-Programmer-Downloader-Debugger-With-20cm-Dupont-Wire-p-1183115.html?rmmds=myorder&cur_warehouse=UK) - for programming the board. Listing is for 3, it's probably a good idea to get spares in case you blow one up (I did)
* SSR - I bought several `Crouzet 25A` and tore one down to make sure it was genuine. The Fotek SSR-25 relays are common and widely used but they are also almost all clones of varying quality. Some clones have 16A triacs inside which would still be perfectly sufficient but others have 4-8A triacs which is probably a fire waiting to happen if you're building a welder. You can get Crydom and other genuine SSRs on farnell/mouser/digikey/etc but expect to pay 5x as much as the clones. **If you do go with an SSR of unknown quality never leave it plugged in to the mains unattended and be prepared in case of fire**. Regardless of which SSR you go for make sure the NTC thermistor is connected to it's heatsink during operation. Some videos about clone SSRs and failures: [melted down](https://youtu.be/FV9t1GFVbhU), [clone teardown](https://youtu.be/DxEhxjvifyY).