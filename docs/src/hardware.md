# M2 Hardware ![MODAQ M2 Hardware](img/m2_icon.png#right)

This section discusses the hardware supported by M2 and presents a reference design (RD) example that is supported by the M2 source code provided in the <a href="https://github.com/MODAQ2" target="_blank">GitHub repository</a>. It should be noted that any hardware mentioned has been either used in or evaluated for M2 projects, however this should not be interpreted as an endorsement or approval of any mentioned hardware product by NLR. Due to the general and flexible nature of the M2 software, M2 applications are not necessarily limited to these hardware selections. 

!!! note 

    Reliability, durability, and suitability of hardware selections should be assessed prior to use in applications of significance. General purpose hardware may vary in design or production tolerances that can result in operational or performance differences between units and batches. 

## Hardware Reference Design
The purpose of providing this reference design is to offer a functional demonstration of the M2 platform. This hardware, along with the instructions in the [Software](software.md) section that follow, should yield the DIY user a properly functioning M2 build. Users seeking an "out of the box" experience, might consider our [M2GO](m2go.md) hardware loan program.

Since each project tends to be unique and measurement objectives can vary considerably, there is no typical configuration that will satisfy all situations. However, there's usually significant overlap in base functionality that is common in most MODAQ projects, such as: measuring analog signals, motions, position, and temperatures. Advanced users can start with the RD and build upon it to create the necessary functionality- that's what we do.  

The following is a diagram of the M2 Reference Design:

![](img/m2rd_diagram.png#center)

!!! success "Equipment List"

    | Brand       | Model             | Qty | Description       |
    | ----------- | ----------------- | --- | ----------------- |
    | OnLogic     | Karbon 410        |  1  | Controller        |
    | LabJack     | T8                |  1  | Multi-I/O         |
    | Netgear     | GS305             |  1  | Ethernet Switch    |
    | Adv. Nav.   | GNSS Compass      |  1  | Heading, GPS, PTP Server |
    | Xsens       | MTi-G-710         |  1  | Inertial Sensor   |
    | BrainBoxes  | ED-582            |  1  | RTD to Ethernet   |
    | Samsung     | T5                |  1  | SSD Storage       |

The above M2 RD is a high-performance system designed for quality measurements. It's capable of simultaneously sampling up to 8 analog channels at high-speed. The selected [T8 Multi-I/O interface](hardware.md#labjack-t8) can be configured for a variety of input types and has both analog and digital outputs for control applications. Advanced features are included in the RD software such as: user HMI, email alerting, configuration utility, data uploading, and event logging. 

## Controller Considerations

### Modular Architecture

M2 is designed with a modular approach that enables flexible hardware configurations and use of devices from various manufacturers and connection types. 

Many of the devices supported in this RD communicate with the controller over ethernet (TCP/IP). Ethernet brings the following advantages: 

1. Connection simplicity: Devices can be connected using standard networking components
2. PoE: Some devices may support Power over Ethernet, which further reduces the wiring complexity. 
3. Flexible hardware placement: I/O modules that are TCP/IP capable can be moved closer to the measurement location, which reduces sensor/instrument wiring run lengths and the likelihood of signal corruption from noise sources. An M2 can be architected in zones or as a distributed system.

!!! note

    Ethernet is non-deterministic and may not be suitable in time-critical applications that rely on precision timing and synchronization. Read more [here](techref.md#real-time).


## Controller
M2 supports controller hardware based on either the x86_64 (e.g. Intel, AMD64) or Arm64 CPU architectures. Examples of such CPUs include the Intel Pentium, Atom, and Celeron series chips, as well as higher-powered variants in the Core series. For Arm64, only the <a href="https://www.raspberrypi.com/products/compute-module-5/?variant=cm5-104032" target="_blank">Raspberry Pi CM5</a> has been evaluated to work with the provided M2 code. It's possible that other Arm64 based computers will work as well, though other devices have not yet been validated.

!!! warning
    Tests conducted with the Raspberry Pi Zero 2W were disappointing. It's suspected that the 512MB of SDRAM is the limiting factor, since all attempts to build the M2 code on the board failed. Building the code on another target and moving the executables to the Zero 2W was not tested. 

Some considerations for CPU selection include:

- Power efficiency
- Performance expectations
- Number of processing cores and threads

The controller main board will include a preinstalled CPU. Some main boards may offer a CPU selection, however most do not. When selecting a main board, it's necessary to balance onboard features and specifications against the CPU capabilities. 

Depending on the application, the main board might be sold as a Single Board Computer (SBC) or be prepackaged in an enclosure or case (e.g. industrial PC). It's recommended to avoid general-purpose consumer oriented boards and instead look for boards targeted for industrial or embedded applications. Some devices marketed for 'Edge' computing may also be suitable. Desirable features in a main board might include:

- Minimum of 4 GB RAM
- M.2 slots that support NVMe (or SATA) SSD storage 
- Multiple USB ports (3.0 or greater preferred)
- At least one gigabit (or better) ethernet port
- Precision Time Protocol (PTP, IEEE 1588v2) support in ethernet controller (e.g. Intel i210)
- Real Time Clock (RTC) with battery backup
- Serial communications port (RS-232/RS-485/UART)
- Wide input voltage tolerances
- Video display interface (optional)

Depending on the operating environment, it might be desirable to select a model that is rated for a wide operational temperature and humidity range. 

!!! note 

    The above features are preferred, but not mandatory. Individual use-cases and performance requirements may indicate need for greater or fewer capabilities. 



## Controller options that have been evaluated for M2.

### Onlogic Karbon 410

![Karbon 410](img/k410.png#right)
While there are many vendors that sell ready-made rugged, industrial computers, we selected the relatively low-cost and modestly equipped <a href="https://www.onlogic.com/store/k410/" target="_blank">Karbon 410</a> from Onlogic for evaluation as a mid-range M2 controller.  

The Karbon 410 is a fanless design and comes in a heavy-duty rail-mountable enclosure with a large passive heatsink. Specification-wise, it includes all of the desirable features listed in the previous section and can be optioned with either GPIO or two serial ports (RS-232/422/485). 

Some notable features of the K410:

- Choice of either a dual or quad core Atom CPU
- Intel i210 ethernet controllers support PTP
- 9-48 VDC input power range
- Carries numerous regulatory certifications, including for shock and vibration
- Full-stress power draw (when equipped with Atom x6211E CPU) is <15W and can deep sleep <<1W


### ODROID H3
![ODROID H3](img/h3.png#right)
The <a href="https://www.hardkernel.com/shop/odroid-h3/" target="_blank">ODROID H3</a> is a very low cost fanless single board computer with a quad core Celeron N5105 CPU. it's a basic, moderately powered performer that lacks the ruggedization and certification bona fides of the K410, however it's versatile and well equipped. 

Some notes on the H3:

- Small 110x110mm footprint
- Dual SODIMM DDR4 RAM slots and one NVMe m.2 slot
- PTP support on the Realtek RTL8125B ethernet controllers <a href="https://askubuntu.com/questions/1502690/realtek-rtl8125-ethernet-controller-ieee-1588-ptp-support-in-ubuntu-22-04-3" target="_blank">requires some effort</a> and not as robust as the Intel ethernet controllers.[^1]
- Supports input power in the range of 14-20 VDC, which is a little unusual.
- Full stress power consumption is ~18W and <1W in suspend mode

### Raspberry Pi CM-5
Raspberry Pi <a href="https://investors.raspberrypi.com/" target="_blank">claims</a> that approximately 75% of its sales are to the Industrial and Embedded sector and their devices are found <a href="https://www.raspberrypi.com/for-industry/powered-by/" target="_blank">powering</a> a variety of devices in assorted applications that are similar or adjacent to MODAQ. Robustness and dependability are important to us, and is one of our stated design requirements, therefore we're cautiously optimistic when it comes to the CM5. Our experience thus far has been positive, however users are encouraged to perform their own evaluations and weigh the risks. 

![CM5](img/cm5_carrier.png#right)
The CM5 is a credit-card sized microcomputer that needs to be inserted into a carrier or expansion board in order to expose the I/O, communications, storage, and power connections. This can increase the overall footprint and power draw, but since the CM5 is basically useless without a carrier board, they must be assessed together when evaluating options. In our tests, the CM5 with the <a href="https://www.raspberrypi.com/products/compute-module-5-io-board/" target="_blank">OEM carrier board</a> power draw under MODAQ operation was measured around 3.4 watts (NOTE: this power consumption value is just for the controller, not including any external peripherals such as the LabJack T8).

Notable features:

- NVMe support on the OEM carrier board
- Hardware PTP support
- Onboard RTC with battery backup
- Compatible with Raspberry Pi HATs
- 3rd party carrier boards and industrial housings available for greater flexibility

Notable disappointments:

- No low power idle mode or hibernation/sleep
- OEM carrier board is rather large (160 x 90mm) and only has one ethernet port
- Published MTBF (Mean Time Between Failure) under high stress environmental conditions is <a href="https://pip-assets.raspberrypi.com/categories/944-raspberry-pi-compute-module-5/documents/RP-008180-DS-7-cm5-datasheet.pdf" target="_blank">16k hours</a> (~2 years at 24/7 duty). MTBF is closer to 10 years in stable, controlled conditions. (to be fair, we could not find published MTBF for the Karbon 410 nor the ODROID H3. The Karbon 410 does meet a variety of IEC and MIL-STD shock and vibration standards)

## Sensors, Instruments, and I/O Options

The M2 Reference Design includes support for the following devices:

- LabJack T8
- Xsens MTi-G-710 GNSS/INS
- Advanced Navigation GNSS Compass
- Brainboxes ED-582 

These provide the following capabilities:

- ±10 VDC analog input and output
- [4-20 mA current loops](techref.md/#4-20-ma-current-loops)
- Logic level digital input and output (also known as TTL or DIO)
- Position, heading, velocity, acceleration, rotation, and orientation
- IEEE-1588v2 PTP time server
- Temperature measurements using PT100 or PT1000 RTD sensors

!!! note

    The devices listed will work on all controllers mentioned in the previous section.

M2 supports considerably more hardware options than are included in the Reference Design distribution. <a href="https://www.nlr.gov/water/open-water-testing" target="_blank">Contact us</a> to learn more. 

### LabJack T8
![Labjack T8](img/t8.png#right)

The LabJack T8 provides a lot of useful I/O in a single highly performant ethernet or USB connected device.  

Some highlights:

- 8 simultaneous sampling 24-bit analog to digital converters (ADC), with channel-to-channel isolation and sampling up to 40 kHz
- ADC is delta-sigma (Texas Instruments <a href="https://www.ti.com/lit/ds/symlink/ads131a04.pdf" target="_blank">ADS131A02</a>)
- Up to 20 channels of digital I/O (DIO)
- USB or ethernet connectivity- can also be powered by USB or PoE
- Available option for 14-bit, ±10 VDC digital to analog (DAC) output
- Available option for 4-20 mA current loop input

The Reference design includes M2 software nodes to stream the ADC channels at up to 40 kHz or read the channels on demand. It also includes nodes to command an output voltage to the DAC module and read/write to the DIO. 

### BrainBoxes ED-582
![ED-582](img/ed582.png#right)

The <a href="https://www.brainboxes.com/product/remote-io/temperature/ed-582" target="_blank">ED-582</a> is a simple solution for acquiring up to 4 RTD measurements over an ethernet connection. Each channel can be configured independently for either PT100 or PT1000 temperature sensors and their associated temperature coefficients. 

It has 16-bit resolution and can acquire at up to 5 samples per second- which provides just over 1 Hz sample rate with all 4 channels enabled. 

### Xsens MTi-G-710
![MTi-G-710](img/Xsens_G-710.png#right)
The world of inertial sensors has a very broad price, performance, accuracy, and quality spectrum, ranging from cheap hobby grade to high-end, export-controlled devices costing many 10's of thousands of dollars. We tend to specify 'moderately' priced models such as the <a href="https://www.movella.com/products/sensor-modules/xsens-mti-g-710-gnss-ins" target="_blank">Xsens MTi-G-710</a> in many projects.  

This device uses a sensor fusion engine to develop 3D orientation solutions using GNSS satellite signals and built-in 3-axis accelerometers, gyros, and magnetometers. 

### Advanced Navigation GNSS Compass

The M2 reference design includes support for the <a href="https://www.advancednavigation.com/inertial-navigation-systems/satellite-compass/gnss-compass/" target="_blank">GNSS Compass</a> from Advanced Navigation. This ethernet connected, PoE powered device includes a GNSS satellite receiver with RTK support, built-in dual antennas, and a PTP time server. 

This device is a complement to the Xsens Mti-G-710, since the GNSS Compass, with its dual antennas can provide a superior heading estimate and with an RTK correction signal available, can provide superior positioning estimates. In addition, it can provide an estimate of heave and provides a highly accurate time reference for the M2 system. 

### Networking
We purposely designed the RD with simple, basic networking. On the plus side, this is cheap and requires no special configuration. It's also sufficiently fast for the selected components. However, in more elaborate builds, especially those with multiple PTP capable devices, a managed switch with PTP support will be more desirable. 

The specified switch <a href="https://mundowin.com/en/How-to-configure-a-local-network-with-a-switch-without-internet/" target="_blank">does not have have a router</a>, so all devices will need to configured with a static IP address all on the same subnet. A laptop with an IP on the same subnet could be attached to one of the unused ethernet ports to access the [HMI](hmi.md).

[^1]: The newly released ODROID H4 comes with <a href="https://www.intel.com/content/www/us/en/products/sku/210599/intel-ethernet-controller-i226v/specifications.html" target="_blank">Intel I226-V</a> ethernet controller chips, which have much better PTP support.