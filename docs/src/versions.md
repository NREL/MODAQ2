# MODAQ 2 Versions

<div class="row">
  <div class="column">
    <img src="../img/m2_x86.png" title ="M2 for Intel/x64 platforms" width = 150>
  </div>
  <div class="column">
    <img src="../img/m2_arm.png" title ="M2 for ARM platforms" width = 150>
  </div>
  <div class="column">
    <img src="../img/m2_m2go.png" title ="M2 prebuild for loan" width = 150>
  </div>
</div>
<p />
The MODAQ 2 codebase has been developed to operate on controllers with either x86_64 (Intel/AMD64) or ARM64 CPU architectures. Initially we assumed there would be enough differences in these architectures to require developing and maintaining a codebase for each. However, the differences were few and easily addressed in the code. To keep things simple, there is a single unified version of the M2 software that supports both Intel and Arm architectures. 

In the following [Hardware](hardware.md) and [Software](software.md) sections, guidance is provided on these topics:

1. Controller selection
2. Sensor and Instrument support
3. Hardware reference design specification
4. Operating system selection
5. Getting up and running with the ROS2 environment
6. M2 reference design software nodes and their usage

## Scaling M2 Designs
M2 easily scales from small, low powered designs to large, high-performance distributed systems using the same M2 codebase. The following sections outline a Reference Design for a performant mid-tier MODAQ system, however the controller could be upgraded or downgraded to dial in the desired level of performance, size, and power consumption. Additional I/O modules can be added for more sensors or instruments as measurement needs evolve. Multiple M2s could be used in a distributed design that could each share data in near-real-time or synchronized later in post-processing using precision timestamped data. 

## M2GO
[M2GO](m2go.md) is a prebuilt and pre-configured version of the M2 reference design using an Intel-based controller 
that NLR makes available for loan to qualifying 3rd parties. It's not a separate M2 version, rather it's 
just a physical build of the reference design described in the following sections of this documentation. 

For parties interested in obtaining an M2GO loaner, please follow this <a href="https://www.nlr.gov/water/modaq" target= "_blank">link</a> to contact us.