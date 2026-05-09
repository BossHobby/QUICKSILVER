# Changelog

## v0.11.2

changes since 0.11.1:

- add ICM42622P and ICM42686P gyro support
- add LSM6DSO, LSM6DSV16X, LSM6DSK320X gyro support
- fix usb rx buffer overflow with proper flow control
- fix usb arming race by moving state to ISR
- fix serial tx_done race condition
- vtx: avoid power level underflow
- vtx: stop cycling protocols, cycle workarounds instead
- max7456: add stat polling to read/write
- osd: fix incorrectly limited element X positions
- blackbox: fix debug flag check
- motor: set MOTOR_OFF for zero/negative mix values in motor test/turtle mode
- reset: clear systicks and interrupts
- stm32g4: fix jlink device

## v0.11.1

changes since 0.11.0:

- fetch gyro data async to the main loop
- **add stm32g4 support**
- split off more tasks from generic util task
- sample bmi270 temperature
- increase blackbox debug field size
- fix divide by zero in stick vector _for real_
- show crsf mode in the lqi osd element
- blackbox: automatically erase uninitialized flash
- sa: only set unlock bit if required
- add osd profiles, allowing to to switch between two different layouts via aux
- fix HD osd positioning
- add rolling update for HD osd to improve reliability
- make AUX_FPV_SWITCH function while on usb
- osd: allow setting aux to ON/OFF
- scheduler: fix scaling of desired task period
- osd: reduce looptime impact for HD systems, making it quite a bit more responsive
- sdcard: reset looptime during init, useful for boards without sdcard detect pin
- add missing pin definitions for f4
- make target pin parser more reliable
- enforce consistent filter period
- switch to pt1 for ibat/vbat sampling
- improve msp support
- add support for more clash chips
- enable oversampling on g4/h7, reduce filter cut off

## v0.11.1-rc5

changes since 0.11.1-rc4:

- add missing pin definitions for f4
- make target pin parser more reliable
- enforce consistent filter period
- switch to pt1 for ibat/vbat sampling

## v0.11.1-rc4

changes since 0.11.1-rc3:

- osd: allow setting aux to ON/OFF
- scheduler: fix scaling of desired task period
- osd: reduce looptime impact for HD systems, making it quite a bit more responsive
- sdcard: reset looptime during init, useful for boards without sdcard detect pin

## v0.11.1-rc3

changes since 0.11.1-rc2:

- fix HD osd positioning
- add rolling update for HD osd to improve reliability
- make AUX_FPV_SWITCH function while on usb

## v0.11.1-rc2

changes since 0.11.1-rc1:

- fix divide by zero in stick vector _for real_
- show crsf mode in the lqi osd element
- blackbox: automatically erase uninitialized flash
- sa: only set unlock bit if required
- add osd profiles, allowing to to switch between two different layouts via aux

## v0.11.1-rc1

changes since 0.11.0:

- fetch gyro data async to the main loop
- **add stm32g4 support**
- split off more tasks from generic util task
- sample bmi270 temperature
- increase blackbox debug field size

intention of this rc is to get the stm32g4 support in more peoples hands.
for some reason or another it took use quite a while to get this one verified.

## v0.11.0

changes since 0.11.0-rc3:

- revert pulling motor lines high, motor beeps were confusing
- update default filters
- update presets for 65mm and 75mm

changes since 0.10.5:

- add a dynamic notch filter pass to the gyro pipeline. a sdft algorithm is used to precisely track noise peaks in the gyro signal and then insert notch filters at these frequencies.
- add a very simple cooperative scheduler to divide compute heavy tasks (like dynamic notch) across multiple loops.
- optimize floating point accuracy of the angle mode pipeline
- include yaw in stickvector calculation to increase accuracy at extreme angles. (thanks @.quickflash)
- various performance optimizations
- add crosshair osd element (thanks @teeu)
- fix error vector rotation equation (thanks @dusking1)
- add filtered voltage warnings
- rework osd init and update prodecdure to take better advantage of the scheduler
- ensure osd status is properly reset
- send esc-passthrough packets in one go
- expose target name via usb descriptor

Profiles should all carry over, but please re-verify.
With the dynamic notch filter active, you will most likely need to re-tune you craft.

## v0.11.0-rc3

changes since 0.11.0-rc2:

- fix bug in at32 gpio init causing no output on motor pins
- pull motor lines high during boot to stop esc beeps
- increase default tda percent

## v0.11.0-rc2

changes since 0.11.0-rc1:

- add filtered voltage warnings
- rework osd init and update prodecdure to take better advantage of the scheduler
- ensure osd status is properly reset
- send esc-passthrough packets in one go
- expose target name via usb descriptor

## v0.11.0-rc1

This release is contains significant changes to quicksilvers flight characteristics,
with some of of them being in development and flight testing for literal years.

changes since 0.10.5:

add a dynamic notch filter pass to the gyro pipeline. a sdft algorithm is used to precisely track noise peaks in the gyro signal and then insert notch filters at these frequencies.
add a very simple cooperative scheduler to divide compute heavy tasks (like dynamic notch) across multiple loops.
optimize floating point accuracy of the angle mode pipeline
include yaw in stickvector calculation to increase accuracy at extreme angles. (thanks @Quick-Flash)
various performance optimizations
add crosshair osd element (thanks @TieuLongHo)
fix error vector rotation equation (thanks @DusKing1)

Profiles should all carry over, but please re-verify.

With the dynamic notch filter active, you will most likely need to re-tune you craft.
Most of the changes have been in testing for quite some time, however they are still significant, so please proceed with caution.

## v0.10.5

- **fix mspvtx when no rx serial port is selected** (regression in 0.10.4)
- improve floating point precision of angle mode input calculation
- refactor osd status messages

## v0.10.4

- remove gesture pid-tunning
- remove auto throttle
- add bmi323 imu support
- **VBATTLOW: raise default to 3.6**
- **gyro: change default filter to pt3@100hz**
- **add support for mspvtx via crsf** (required for HM SuperX)
- arming: rework to check throttle before

## v0.10.3

- osd: adjust throttle boost step to 0.1
- h7: fix flash settings write
- h7: fix cpu temperature
- sdcard: fix port and nss config

## v0.10.2

- icm42: increase hardware filtering to account for additional noise incurred by the gyro stall fix
- misc cleanups

if you run a quad with the icm42 i highly recommend you update to this version

## v0.10.1

- elrs spi: speed up link recovery after flash-save
- icm42: disable function causing gyro stalls
- msp-vtx: ignore flash-saves
- misc cleanups

## v0.10.0

This release is probably the biggest update to QS since its initial release.
It removes the target configurations from the firmware source code and instead reads them from flash during boot.
This gives us greater flexibility and ensures we can maintain QS long-term with an ever-growing list of targets.
During development of this feature AT32 boards came along, so it made sense to implement support for them at the same time.

The Target definitions can be found here, pull requests welcome:
https://github.com/BossHobby/Targets

Profiles should all carry over, but please verify the serial port assignments again, especially if you had been running one of the develop versions.

- Add support for blackbox presets (thanks @sakitume)
- refactor blackbox for more flexibility
- improve usb serial responsiveness
- add support for loading target definition from flash
- add dynamic allocation for timers
- add at32 support
- elrs: improve time-to-lock after flash-save
- elrs: improve rssi responsiveness
- improve looptime calculation accuracy
- osd: dynamically move decimal point
- icm42: lower filter cutoff
- switch to "rc" naming scheme for pre-releases
- fix loading default gyro orientation from target
- fix floating point error for osd adjustments
- fix buffer overwrite for bindstorage
- rework motor mixer for less clipping
- automatically detect rx filter cutoff
- fix uart7 on at32 targets (thanks @kikoqiu)
- enable voltage display on dji osd (thanks @kikoqiu)
- improve gyro still detection (thanks @kikoqiu)
- elrs: ensure radio chip is only init once
- refactor stick rate calculation
- add support for 4mb flash at32 chip variants (at32f435m)
- stop relying on the usb_detect pin
- supply ibat and vbat scale defaults from targets where applicable
- prevent stick deadband from being applied on rx telemetry packets
- drop broken support for ICM20649
- decrease rx filter cutoff slightly
- ensure crsf telemetry packets take up all the available payload size
- gate iterm windup while on-ground
- handle msp-vtx in serial passthrough

## v0.10.0-rc3

- stop relying on the usb_detect pin
- supply ibat and vbat scale defaults from targets where applicable
- prevent stick deadband from being applied on rx telemetry packets
- drop broken support for ICM20649
- decrease rx filter cutoff slightly
- ensure crsf telemetry packets take up all the available payload size
- gate iterm windup while on-ground

RC was re-published to fix gyro display in the config.

## v0.10.0-rc2

- switch to "rc" naming scheme for pre-releases
- fix loading default gyro orientation from target
- fix floating point error for osd adjustments
- fix buffer overwrite for bindstorage
- **rework motor mixer for less clipping**
- **automatically detect rx filter cutoff**
- fix uart7 on at32 targets (thanks @kikoqiu)
- enable voltage display on dji osd (thanks @kikoqiu)
- improve gyro still detection (thanks @kikoqiu)
- elrs: ensure radio chip is only init once
- refactor stick rate calculation
- add support for 4mb flash at32 chip variants (at32f435m)

## v0.10.0-dev

This release is probably the biggest update to QS since its initial release.
It removes the target configurations from the firmware source code and instead reads them from flash during boot.
This gives us greater flexibility and ensures we can maintain QS long-term with an ever-growing list of targets.
During development of this feature AT32 boards came along, so it made sense to implement support for them at the same time.

The Target definitions can be found here, pull requests welcome:
https://github.com/BossHobby/Targets

Profiles should all carry over, but please verify the serial port assignments again, especially if you had been running one of the develop versions.

changes since 0.9.6:

- Add support for blackbox presets (thanks @sakitume)
- refactor blackbox for more flexibility
- improve usb serial responsiveness
- add support for loading target definition from flash
- add dynamic allocation for timers
- add at32 support
- elrs: improve time-to-lock after flash-save
- elrs: improve rssi responsiveness
- improve looptime calculation accuracy
- osd: dynamically move decimal point
- icm42: lower filter cutoff

new targets:

- betafpvf435 (AT32!)
- neutronrcf435mini(AT32!)
- neutronrcf435se (AT32!)
- iflight_blitz_f435 (AT32!)
- iflight_blitz_mini_f435 (AT32!)
- tunercf411

Many thanks to betafpv, neutronrc and iflight for supporting this development with FC samples!

## v0.9.6-dev

- improve spi driver performance and memory usage
- icm42xxx configure anti-aliasing filter to ~200hz and 3rd order
- icm42xxx fix bank switching
- increase accel calibration limit
- quacmode
- fix motor beep melody timing
- bmi270 enable feature for better axis accuracy (CAS & IOC), thanks @DusKing1
- bmi270 enable hardware axis calibration (CRT), thanks @DusKing1
- source file naming improvments
- add configurable vtx powertable
- fix in-config configuration for bb51 escs
- fix reversal after aborted and manually flipped turtle, thanks @pwojt

new targets:

- speedybeef405v3
- iflight_f411_pro
- zeezwhoop
- tmotorf411
- foxeerf722_v2, thanks @damian-kolakowski
- tmotorf411_elrs
- mambaf405us_i2c, thanks @damian-kolakowski
- flywoof411_5in1_aio
- cross_f4

## v0.9.6

- improve spi driver performance and memory usage
- icm42xxx configure anti-aliasing filter to ~200hz and 3rd order
- icm42xxx fix bank switching
- increase accel calibration limit
- quacmode
- fix motor beep melody timing
- bmi270 enable feature for better axis accuracy (CAS & IOC), thanks @DusKing1
- bmi270 enable hardware axis calibration (CRT), thanks @DusKing1
- source file naming improvments
- add configurable vtx powertable
- fix in-config configuration for bb51 escs
- fix reversal after aborted and manually flipped turtle, thanks @pwojt

new targets:

- speedybeef405v3
- iflight_f411_pro
- zeezwhoop
- tmotorf411
- foxeerf722_v2, thanks @damian-kolakowski
- tmotorf411_elrs
- mambaf405us_i2c, thanks @damian-kolakowski
- flywoof411_5in1_aio
- cross_f4

## v0.9.5-dev

- some general codebase housekeeping :broom:
- bring back proper versions for use in the configurator
- update esc-passthrough code to allow interacting with BB51 escs
- improve blackbox reliability
- do not record blackbox during turtle
- fix spi-elrs battery telemetry
- improve vtx admin reliability

## v0.9.5

changes since 0.9.1:

- improve safety of spi transaction scheduling
- initial cut for msp vtx support (to be improved)
- improve state-tracking for motor reversal
- add elrs 3.0.0 support
- ensure turtle triggers only if we are fully upside down
- crsf: fix packet rate and filtering for elrs 3.0.0
- crsf: read vtx telemetry
- spi-elrs: read vtx telemetry
- initial flysky support (thanks @sakitume )
- improve usb handling (better reliability for blackbox)
- stop init of invalid rx serials (delayed H7 boot)
- various improvements to data-flash (better reliability for blackbox)
- re-init rx after import of bind-storage, preventing race-condition with ongoing bind for frsky
- overclock f411 to 120mhz
- optimize redpine loop impact
- add 5th powerlevel for vtx
- spi-elrs re-init after apply in the config, rssi should come up without reboot
- fix msp-vtx on soft-serial (can block boot!)
- dshot: try returning to normal rotation on every arm
- some general codebase housekeeping :broom:
- bring back proper versions for use in the configurator
- update esc-passthrough code to allow interacting with BB51 escs
- improve blackbox reliability
- do not record blackbox during turtle
- fix spi-elrs battery telemetry
- improve vtx admin reliability

new targets:

- matekf411rx brushed (re-added)
- tmveloxf411
- betafpvf411rx_frsky
- tbs_podraceraio
- jhef411
- hobbywing_xrotorf7conv
- tcmmf411
- flywoof411_v2

## v0.9.4-dev

- spi-elrs re-init after apply in the config, rssi should come up without reboot
- fix msp-vtx on soft-serial (can block boot!)
- dshot: try returning to normal rotation on every arm

## v0.9.3-dev

- ensure turtle triggers only if we are fully upside down
- crsf: fix packet rate and filtering for elrs 3.0.0
- crsf: read vtx telemetry
- spi-elrs: read vtx telemetry
- initial flysky support (thanks @sakitume )
- improve usb handling (better reliability for blackbox)
- stop init of invalid rx serials (delayed H7 boot)
- various improvements to data-flash (better reliability for blackbox)
- re-init rx after import of bind-storage, preventing race-condition with ongoing bind for frsky
- overclock f411 to 120mhz
- optimize redpine loop impact
- add 5th powerlevel for vtx

new targets:

- tbs_podraceraio
- jhef411
- hobbywing_xrotorf7conv
- tcmmf411
- flywoof411_v2

## v0.9.2-dev

- improve safety of spi transaction scheduling
- initial cut for msp vtx support (to be improved)
- improve state-tracking for motor reversal
- add elrs 3.0.0 support

new targets:

- matekf411rx brushed (re-added)
- tmveloxf411
- betafpvf411rx_frsky

## v0.9.1

- fix frsky packet-rate counting during for telemetry sends
- improve gyro init for brushed boards
- use fixed lengths for frsky hop table entries
- switch to github actions as a ci/cd

new targets:

- tunercf405
- NeutronRCF411RX
- flywoof405s_aio
- SpeedybeeF7MiniV2

## v0.9.0

- improvements to the serial dsm code including better filter cut selection
- add logic to allow for profile migration in the configurator
- add two distinct rate profiles, select able via the config or the osd
- add support for actual rates
- fix expo term for betaflight rates
- add support for hdzero osd (canvas/msp displayport)
- a lot of little tweaks and improvements to the osd in general
- split cell count and voltage osd elements
- calculate per cell voltage average and display that in the osd
- preserve bluejay startup tone configuration when using the esc setting in the configurator
- improve performance in osd check function
- improve dataflash performance (switch to txn)
- fix issue where deadband got applied every loop instead of every packet
- vtx ensure powerlevel is always within valid range
- display currently active pid and rate profile in osd
- various improvements to dataflash blackbox
- make plus mixer easier to configure
- use more precise coefficient calculation for pt1 and pt2 filters
- add pt3 filter
- remove betafpv angle mode algorithm
- auto detect gyro chip model
- fix objcopy for building on windows
- filters: use auto-detected looptime for period
- enable lto
- crsf: send cell voltage
- H7 support for H743
- rework all spi devices to use spi-txn
- various improvements to spi-txn
- bmi270: change hardware lpf to use approximately the same cut-off as mpu6000
- blackbox: write entries tightly packed to flash, increasing recording time
- add halfduplex serial pass-through support
- add mspv2 support
- add msp serial pass-through support
- changes iterm relax to fade
- improve hdzero osd update latency
- pass sticks to hdzero vtx to be able to use hdzero stick gestures
- add LRL stick gesture to force osd re-draw
- more closely track motor direction for turtle mode
- add channel mapping functionality
- expose stick calibration wizard to the configurator
- fix osd stick-calibration wizard
- fix telemetry transmission for spi-frsky
- reduce chances of corruption for data-flash blackbox
- enable all RX protocols for a given hardware configuration
- fix turtle throttle percent adjust in osd
- add motor limit functionality
- improve serial passthrough to support openvtx-configurator
- various smaller improvements to SA handling
- add AUX_PREARM, this aux must be active while ARM occurs
- allow osd screens to scroll
- add throttle mid and expo functionality
- display stick throttle vs applied throttle in osd
- fix motor and pidoutput clipping for iterm relax
- revert iterm relax to more conservative values
- ensure dshot values for a given digital idle match previous versions 1:1
- slightly reduce spi speed for sx1280 (elrs), seems it can miss a beat in high traffic situations

new targets:

- tmotorf7
- betafpvf411
- zeusf722_aio
- hglrcf722
- hifionrc_f722
- foxeerf745_aio
- iflight_h743_aio_v1
- flywoof411rx
- aikon_f7
- jhef405pro
- iflight_blitz_f411_elrs
- betafpv_f405

## v0.8.6-dev

- fix motor and pidoutput clipping for iterm relax
- revert iterm relax to more conservative values
- ensure dshot values for a given digital idle match previous versions 1:1
- slightly reduce spi speed for sx1280 (elrs), seems it can miss a beat in high traffic situations

## v0.8.5-dev

- changes iterm relax to fade
- improve hdzero osd update latency
- pass sticks to hdzero vtx to be able to use hdzero stick gestures
- add LRL stick gesture to force osd re-draw
- more closely track motor direction for turtle mode
- add channel mapping functionality
- expose stick calibration wizard to the configurator
- fix osd stick-calibration wizard
- fix telemetry transmission for spi-frsky
- reduce chances of corruption for data-flash blackbox
- enable all RX protocols for a given hardware configuration
- fix turtle throttle percent adjust in osd
- add motor limit functionality
- improve serial passthrough to support openvtx-configurator
- various smaller improvements to SA handling
- add AUX_PREARM, this aux must be active while ARM occurs
- allow osd screens to scroll
- add throttle mid and expo functionality
- display stick throttle vs applied throttle in osd

new targets:

- iflight_h743_aio_v1
- jhef405pro
- iflight_blitz_f411_elrs
- betafpv_f405

## v0.8.4-dev

- adds flywoof411rx target
- fix objcopy for building on windows
- filters: use auto-detected looptime for period
- enable lto
- crsf: send cell voltage
- adds aikon_f7 target
- adds hifionrc_f722 target
- adds foxeerf745_aio target
- H7 support for H743
- rework all spi devices to use spi-txn
- various improvements to spi-txn
- bmi270: change hardware lpf to use approximately the same cut-off as mpu6000
- blackbox: write entries tightly packed to flash, increasing recording time
- add halfduplex serial pass-through support
- add mspv2 support
- add msp serial pass-through support

## v0.8.3-dev

- adds zeusf722_aio target
- display currently active pid and rate profile in osd
- various improvements to dataflash blackbox
- make plus mixer easier to configure
- add hglrcf722 target
- use more precise coefficient calculation for pt1 and pt2 filters
- add pt3 filter
- remove betafpv angle mode algorithm
- auto detect gyro chip model

the existing (pt1 and pt2) filters should feel and behave _close_ to identical.
the pt3 filter is heavier but can be run a single pass (~100hz) which can lead to improved performance on nosier whoops.
it will require you to retune, with most notably reduced dterm.

## v0.8.2-dev

- adds betafpvf411 (non-rx) target
- improve performance in osd check function
- improve dataflash performance (switch to txn)
- **fix issue where deadband got applied every loop instead of every packet**
- vtx ensure powerlevel is always within valid range

## v0.8.1-dev

- improvements to the serial dsm code including better filter cut selection
- add target tmotorf7
- add logic to allow for profile migration in the configurator
- add two distinct rate profiles, select able via the config or the osd
- add support for actual rates
- fix expo term for betaflight rates
- add support for hdzero osd (canvas/msp displayport)
- a lot of little tweaks and improvements to the osd in general
- split cell count and voltage osd elements
- calculate per cell voltage average and display that in the osd
- preserve bluejay startup tone configuration when using the esc setting in the configurator

## v0.8.0

- F7 support for F722, F745 and F765
- support for the icm42605 and bmi270 gyros
- fixes required for the 12a betafpv elrs board
- blheli tidyup and blheli32 support
- current sensor support
- dynamic filter cuts for serial elrs
- 1khz flrc elrs serial support
- softserial support

## v0.7.0

- spi-elrs implementation, including more tweaks since 0.6.1-dev for time-to-link and reliability
- new spi-transaction api allowing spi-elrs to run mostly of interrupts making it light on cpu load
- overclock to 108mhz on F411, allowing for plenty of headroom
- improved motor beep behavior
- openvtx support both on SA and TRAMP
- vtx settings are saved to flash and restored on boot
- vtx pit mode can now disabled/enabled with AUX_FPV_SWITCH
- various smaller fixes to SA and TRAMP
- boot logo support both in firmware and config, make sure to update the osd font to get the QS logo.

## v0.6.1

- enable Crossfire and FrSky D8 telemetry
- add OSD arm time plus VTX channel and band display
- add AUX_BUZZER_ENABLE support for motor beeps
- disallow buzzer racket and motor beeps while connected to USB
- add USB bind-storage setting
- add interrupt-safe time helpers and unify interrupt priorities
- rework sdcard writes to be async and DMA-backed
- improve blackbox/data-flash handling, rate headers, full-flash checks and USB readback
- add betaFPVF411 target and aikon_f4 build output
- add performance counters and USB reporting

## v0.6.0

- switch the project to PlatformIO/Cube LL based builds
- add a new USB driver and restore USB descriptors
- increase USB buffers and ensure all queued USB data is written
- add USB MSP commands required by the Bluejay PWA configurator
- stream OSD font data over USB
- rework soft serial
- add generic gyro interface
- port PWM, ADC and soft-SPI code to STM32 LL drivers
- add CBOR error handling and support integer-encoded float values
- prefer explicit-size integer types across the codebase
- tidy project/system defines and LED logic

## v0.5.1

- add stick calibration wizard support in OSD and USB/QUIC
- add configurable LQI source support across CRSF, DSM, SBUS, FPort, IBUS and Redpine
- add RC link OSD menu with LQI source adjustment
- add Tramp VTX support and VTX protocol auto-detection
- improve SmartAudio/Tramp serial VTX handling and reliability
- add iflight_f411_rx and zeusf4evo targets
- support CC2500 PA/LNA configurations without an LNA enable pin
- improve CRSF frame parsing and unified serial buffering
- rename and clean up several AUX functions and OSD flight-mode mappings
- update PID presets, README content, logo and community links

## v0.5.0

- migrate the build system to PlatformIO
- generate hex files through the PlatformIO build pipeline
- add PlatformIO board HSE values and advanced flags
- add PlatformIO container/drone build support
- remove the old makefiles
- automatically reset boards before upload
- improve motor-test/turtle-mode override handling when USB disconnects
- add clang-format configuration

## v0.4.2

- rework 4-way ESC passthrough to interact with GPIO directly instead of softserial
- add unified flash storage structures for profile, VTX and bind data
- expose RX bind/status information over USB/control state
- fix RX_STATUS_DETECTED handling
- move board GPIO resources to GPIO enums
- enable UART5 on matekf405
- add FPVCycleF411 and iflight_succex_e_f4 targets
- fix FPVCycleF411 gyro orientation
- update PID presets
- bump QUIC protocol version
- remove unfinished OSD setup wizard functionality

## v0.4.1

- add unified CRSF serial receiver support
- add Redpine serial receiver support
- add LQI helper functions
- harden unified serial protocol detection and CRSF header handling
- add CRSF link statistics parsing
- add Redpine CRC16 and packet flag parsing
- fix ADC divide by zero
- add nox SPI RX target variant
- remove CRSF-specific target builds
- move circular_buffer_t to its own file

## v0.4.0

- add runtime looptime autodetect
- move ADC reads to DMA and add non-blocking multi-channel ADC
- add single-DMA-stream DShot broadcasting
- guard DShot DMA transactions on SPI1/DMA2
- add interrupt-based serial RX telemetry sends
- rework OSD system status and add gyro temperature OSD element
- improve SmartAudio detection, baud handling and frame termination
- improve FPV switch behavior during failsafe and SmartAudio menu handling
- add support for non-half-duplex FPort with external bidirectional inverter
- add ICM20602 support and nfe_quicksilver target

## v0.3.4

- add ICM20608 gyro support
- add gyro_id to target info
- add bluejayf4 and infinityF4 targets
- update bluejayf4 target

## v0.3.3

- optimize BLHeli read/write timings
- exit each ESC after BLHeli read/write operations
- add ff_racepit CRSF target

## v0.3.2

- add ff_racepit target
- disallow arming during USB operation
- add config guard for USB arming safety
- reset gyro on MPU_RESET_SIGNAL_PATHWAYS
- add gyro temperature to control state
- give ESCs more time after each 4-way command

## v0.3.1

- fix motor-test override state leaking into turtle mode

## v0.3.0

- add blackbox AUX function
- add compact vector-based blackbox encoding
- record PID terms, setpoint and motor mix in blackbox
- add packed higher-rate blackbox logging
- add USB blackbox file listing and larger blackbox packet sizes
- improve data-flash safety, restart handling and chip detection
- add target feature information to target info
- add runtime DShot timing
- move core runtime values into control_state/control_flags structures
- add compile-time sixaxis calibration support
- add pedro_f4 target
- bump QUIC protocol version

## v0.2.0

- add multiple pin-controlled inverter support
- handle inverted SBUS and FPort serial receivers
- skip inverted protocols on flight controllers without inverters
- update targets for UART-specific pin inverters
- add flywoof411, ff_racepit and aikon_f4 targets
- add omnibusf4sd to the build list
- move ENABLE_SMART_AUDIO to hardware definitions
- sync Redpine with recent upstream changes
- improve bump-version behavior for lesser version components

## v0.1.1

- expose 4-way ESC passthrough over QUIC
- add BLHeli settings read/write support
- add USB motor-test command and allow USB motor test without RX input
- move motor mixer/output logic into dedicated motor code
- add basic inverted operation support
- rework SmartAudio to run asynchronously
- improve SmartAudio pit-mode handling, retries and apply behavior
- fix CBOR sizing for values and add text-string copy helper
- add QUIC exit flag for returning control
- clean up arming/motor-test state handling
- move SPI port definitions into targets

## v0.0.1

- initial tagged release after the runtime serial work
- add data-flash backed blackbox storage and USB download support
- add M25P16 flash driver and simple data-flash filesystem
- move SPI devices to the new SPI/GPIO abstraction
- add FrSky D16 and Redpine receiver support
- improve FrSky D8/D16 telemetry, bind timing and failsafe behavior
- add SmartAudio VTX control and OSD VTX menu
- add OSD font read/write support and system reset menu entry
- add runtime gyro and D-term filter configuration
- add dynamic D-term filtering and throttle D-term attenuation
- add turtle mode using DShot motor direction commands
- add DShot build support and motor-test/turtle safety improvements
- add target metadata, default builds and new targets including matekf405, alienwhoop_v3, redpine, clracing_f4 and matekf411
- improve build scripts, version handling and CI helper scripts
