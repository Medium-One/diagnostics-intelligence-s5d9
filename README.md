# Diagnostics Intelligence

This project requires SSP v1.2.1, which can be downloaded from the Synergy Gallery (https://synergygallery.renesas.com/ssp). It also requires the Medium One Cloud Agent for Synergy, which is a VSA which can also be downloaded from the Synergy Gallery (https://synergygallery.renesas.com/addon). Please download a version of the Medium One Cloud Agent for Synergy supporting SSP 1.2.x. 

## Installation
Install the VSA by extracting the contents into the *m1* folder in the Diagnostics Intelligence project. Install the S5D9 IoT Fast Prototyping BSP pack file to the *Packs* directory of your e2Studio installation.

## Drivers
The sensor drivers used in this project are based on the drivers available here:
* ENS210
    * <https://download.ams.com/ENVIRONMENTAL-SENSORS/ENS210/Driver>
* BMC150
    * <https://github.com/BoschSensortec/BMM050_driver>
    * <https://github.com/BoschSensortec/BMA2x2_driver>
* MS5637
    * <https://github.com/TEConnectivity/MS5637_Generic_C_Driver>
