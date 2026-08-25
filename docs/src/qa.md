# Q&A ![MODAQ M2 Q&A](img/m2_icon.png#right)

## How does MODAQ differ from pre-built commercial options?
This is a good question and important to understand where MODAQ fits in with commercial alternatives. Due to a variety of factors, the answer is not that simple. One needs to consider budget, performance expectations, system flexibility, functional requirements, and the technical aptitude of the end user. 

Here's some things to consider:

1. Data acquisition systems can be found ranging from a few hundred dollars to well over $100,000 in cost. The price point is often a clue as to how performant and capable (and complex) the system will likely be.  
2. While some commercial products provide user software to configure, manage, and operate the system, anything beyond the basics will require programming. 
3. If the measurements are important, for instance being used for model validation, design basis, safety, or for financial/production purposes, the quality and integrity of those measurements must be able to withstand scrutiny. 

Data acquisition is an engineering discipline with numerous sub-specialties that can take years and lots of study to become competent. By offering MODAQ publicly, users can leverage the efforts of the national lab. 

## Why not just use generic DAQ controllers?
There are a lot of options out there, but it does not take long to realize that:

1. You're just trading one ecosystem for another.
2. Specification to price ratio might be (much) worse.
3. Documentation and libraries may be lacking.
4. Device compatibility may become a bit of an issue.
5. Your preferred programming language or operating system might not be supported.
6. Software still needs to be developed (or in some cases configured) to achieve expected results.
7. It could end up being a lot of work!

Of course it depends on individual needs and expectations, but whether going with MODAQ or an alternative, there's no getting away from the tasks of programming the system and assuring it will perform to expectations and deliver quality results.

Some vendors sell controllers and I/O modules and offer software packages that require some configuration to achieve basic and limited data acquisition that can be considered on the easy end of the spectrum. However, doing anything beyond simple acquisitions often requires accessing a vendor supplied API and writing code in a traditional language like C/C++ or python - or worse - writing code in a vendor developed scripting language.

There's a class of controllers that include an array of built-in I/O that are nice, well thought out, easy to configure, and performant, however under closer inspection, they lack some key features that make them unsuitable in an embedded application deployed in the field. For example, these options generally have limited data logging capability, supporting only small (usually 64 GB or less) and slow microSD cards. There's also limitations when it comes to more technical matters, such as sampling speed, multiplexing (or scanning) vs simultaneous sampling, data synchronization, bit depth, multitasking, determinism, and noise. They also lack flexibility of local filtering or signal processing of data, local QC, and communications/remote supervision. Of course, use case and measurement objectives matter, so these can be perfectly fine for some applications.

## Is MODAQ plug-n-play? 
Generally, no. Each user will have their own unique set of measurement objectives and requirements which cannot realistically be fulfilled by a general MODAQ design. The one exception is the Reference Design presented in the previous sections of this documentation. If the user selects the exact hardware presented in the [Hardware Reference Design section](hardware.md/#hardware-reference-design) and follows the instructions in the [Software section](software.md), the system should function. NLR has a limited number of fully built and configured MODAQ2 Reference Design systems that can be loaned out under the M2GO program. 

## What is M2GO?
M2GO is a fully built, configured, functional, and tested manifestation of the M2 reference design presented in this web document that is available for loan from NLR to qualified 3rd party end users. This is first come, first served and availability is limited. To learn more, contact us <a href="https://www.nlr.gov/water/modaq" target= "_blank">here</a>

## Do I need to know programming to use MODAQ?

## Which sensors or instruments should I use? 
This is a difficult question to answer, since it depends on a number of factors including measurement objectives, range of value(s) to be measured, quality of measurements, and more. Before actually shopping for devices, it's best to consider the following questions:

**What am I trying to measure?** This might sound like an obvious question, but often the first step is define the measurement objective and deconstruct what is needed to satisfy it. Perhaps that leads to measuring the pressure in a pipe. M2 cannot measure pressure directly, but it can measure voltages in the range of ±10VDC. Therefore a pressure sensor is needed that converts the pipe pressure to a ±10VDC signal. The voltage measured by M2 is then scaled to give the result as pressure in the desired units (PSI, ATM, Pa, etc). 

**What are the measuring range I need?** Sensors and instruments should be selected so that they cover the expected variance in the property under measure, with some safety margin. A device with a smaller range than necessary may clip the measurement or get damaged. A device with too broad a range may not be able to capture the variance of the property with sufficient precision. If, for example, you're measuring pressure in a pipe that's expected to range from 0-350 PSI, a pressure sensor with a range of 0-500 PSI might be the right choice, since it has some overhead available for unexpected pressure spikes. 

**What's the appropriate sampling strategy necessary to capture data useful for my purpose(s)?** Sampling strategy covers a range considerations, including sample rate, bit depth/resolution, and filtering. Selection of such should be governed by the measurement objectives and subsequent analytic uses. It's important to be practical when setting the sampling strategy requirements, since high-performance sensors/instruments and their required input modules can not only impact the budget but also generate a lot of data. 

When it comes to sample rates, the M2 RD allows for a mix of high speed (up to 40 kHz) and low speed (1 - 100 Hz) input channels. It's quite common for parameters like voltage and accelerations to be measured with a high-speed scheme, while slower responding parameters, such as temperatures and pressures be measured at a lower rate. 

The bit-depth is one factor that determines the smallest change in a parameter's value that can be measured by the equipment. This is a base-2 value (2<sup>n</sup>), where n == number of bits. In the case of 16-bit sampling, 2<sup>16</sup> = 65,536 discrete values. Therefore, on a 16-bit analog input set to ±10VDC, the smallest value or change in value measurable is 20v / 65,536 = 0.000030518 VDC or 305.18 µV. That's pretty good, but often a 16-bit sampler can't actually achieve the full 16 bits due to noise or quantization error and usually contains a specification for the Effective Number of Bits (ENOB), which can be several bits less.  

**What associated factors do I need to consider?** Often the environment where the sensor will be placed needs to be considered. Can the device withstand expected temperatures? Shock, vibration, UV exposure? Will it be submerged or exposed to precipitation? What about dust, sand, grit, or suspended solids in water? If you're measuring flow or temperature within a pipe, perhaps the sensor can adequately cover the measurement range of those values, but can it withstand the pressure from the fluid in the pipe? 

**How will these measurements be used?** Again, a seemingly obvious question, but if the measurements are to support accredited testing or likely to face scrutiny, then that sets the bar to a certain level. Perhaps the measurements will be used as in input to a model or for model validation- input from the modeling team may be necessary to focus on the appropriate measurements.

**What quality of measurements is 'good enough'?** 

**What about signaling?** The device will have a signaling method that requires an appropriate interface on your MODAQ system. This may be ±10VDC analog signal, RS232 serial, 4-20 mA current loop, or other. Often devices are available with a choice of signaling methods. It's recommended to prioritize a method that matches available I/O ports on your equipment and that don't require additional code development to acquire and process. For example, consider an encoder for measuring rotary or linear position. Some encoders give you <a href="https://novanta.com/robotics-automation/product/midi-incoder-inductive-angle-encoder/" target="_blank">dozens of options</a> for the signaling. Selecting the wrong one could require the purchase of an expensive interface and extra coding effort. For a simpler example, a pressure sensor may be available in with either ±10VDC or 4-20 mA output. Either choice is natively supported by the M2 Reference Design and M2GO, however the 4-20 mA might be the better choice since it's less susceptible to noise, but it requires a shunt resistor and possibly an excitation source (see this [discussion](techref.md#4-20-ma-current-loops)) 

**Can I supply the device with its required power?** Some applications may have power limitations that can constrain equipment decisions for a particular measurement campaign. 

**What's your budget?**


## Can NLR help configure or customize MODAQ?

<a href="https://www.nlr.gov/news/video/discovering-the-ocean-through-data-modaq-text.html" target = "_blank">MODAQ</a> is funded by the Department of Energy's <a href="https://www.energy.gov/cmei/water/hydropower-and-hydrokinetic-office" target="_blank">Hydropower and Hydrokinetic Office (H2O)</a> for the purpose of advancing the development and testing of Marine Energy devices.  There are <a href="https://www.nlr.gov/water/work-with-us.html" target="_blank">mechanisms in place</a> for large and small organizations to engage NLR. <a href="https://www.nlr.gov/water/modaq" target="_blank">Contact</a> the development team.