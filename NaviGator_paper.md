---
title: "NaviGator AMS"
date: "December 2018"
numbersections: yes
authors:
    - name: Daniel Volya
    - name: Matthew Griessler
    - name: Kevin Allen
    - name: Kipling Cohen
    - name: Alan Albritton
    - name: Nicholas Suhlman
    - name: David Zobel
    - name: Juan Mejia
    - name: Rosemond Fabien
    - name: Daniel Olis
    - name: Marshall Rawson
    - name: Jaxon Brown
    - name: Ria Pendon
    - name: Marquez Jones
    - name: Scarlet Seymour
    - name: Dr. Eric Schwartz
    - name: Dr. Carl Crane
    - name: Dr. Ira Hill
    - name: Shannon Ridgeway
abstract:
    NaviGator ASV is a fully autonomous surface vehicle (ASV) built to compete in the Association for Unmanned Vehicle Systems International (AUVSI) Foundation’s 2016 Maritime RobotX Challenge in Oahu, Hawaii. The NaviGator ASV is part of a larger group of collaborative autonomous aerial, surface, and subsurface vehicles known as the NaviGator Autonomous Maritime System (AMS). This paper describes the NaviGator ASV’s structural design, propulsion, power system, electrical design, software infrastructure, outreach efforts, and approach to completing the challenges presented in the 2016 Maritime RobotX Challenge.
---

# Introduction

sds

# Vehicle Design

This section of the paper will describe the hardware and
software that was developed for this competition, as well as
the motivations behind these choices. This will include
descriptions of early iterations of hardware and software that
may have failed, what was learned in that process, and how
that knowledge was integrated to improve on the designs.

## Mechanical Subsystems

The mechanical platform used for the NaviGator ASV is a
modified WAM-V research vessel developed by Marine
Advanced Research. Several of the mechanical modifications
that the team has made will be detailed in this section. A
computer-aided design (CAD) render of the NaviGator ASV is
shown in Fig 1.

### Propulsion

NaviGator ASV’s propulsion system began
as two forward-facing stern thrusters, providing the ASV with
a skid-steer configuration. After a short time of testing, it
became apparent that adding more thrusters and mounting
them at an angle would simplify the vectoring of the thrust to
achieve a desired motion, as well as adding the capability of
lateral motion. The current configuration features two bow and
two stern thrusters oriented at a fixed 45 degrees. This is a
thruster configuration that the team used in the 2013
RoboBoat Competition with much success, earning first place.
In addition to improved maneuverability, using four thrusters
provides redundancy in the system, allowing the ASV to still
have maneuverability even if either both bow thrusters or both
stern thrusters fail. This feature was invaluable when a motor
driver died minutes before a qualification run in the 2013
RoboBoat Competition. With a quick modification to the
thruster mapper program, the ASV was able to operate with
just three thrusters, saving the run. Moreover, during the 2016 RobotX Maritime Challenge, NaviGator was able to repeatedly and successfully station keep and maneuver the course despite rough currents and winds. The major disadvantage of
this configuration is that the fixed angles of the thrusters
means that it is not particularly efficient moving in any
direction. However, as was demonstrated in 2016, for the tasks that the Navigator ASV is
designed to perform, maneuverability is significantly more
important than efficiency.

Mounting the thrusters posed many challenges and required
several design iterations, especially for the bow thrusters. For
the ASV to be deployed from a trailer, the bow thrusters had
to be either removed or raised during deployment so they
would not collide with the trailer structure. The transom
clamps on the trolling motors accommodated this function. 3D
printed polycarbonate clamping blocks that interfaced with the
clamps on the trolling motors kept them fixed in place. While
the mounts held the motors securely, the 3D printed parts
began to crack and eventually failed. To solve this issue, the
clamping blocks were machined from aluminum.

### Sensor Mast

The need for a stable sensor platform is
paramount in machine vision applications. The preliminary
design utilized an 80/20 aluminum rail truss, which did not
provide the required stiffness and resulted in smearing of the
vessel’s detection data. The initial sensor platform also did not
raise the LIDAR system high enough to permit detection of
obstacles in immediate proximity to the pontoons, a problem
rectified in the final design.

As previously mentioned, the cameras, LIDAR, and GPS
antenna require a rigid support. The need for an unobstructed
GPS antenna guided the design towards a mast structure. For
transport to the competition site, the assembly had to fit within
the prescribed envelope of a Pelican Products transport case,
requiring a modular assembly process. These target
specifications led to a base-and-tree assembly, where the mast
is simply welded to a plate that then fastens to the payload tray
via a superstructure. For corrosion resistance and
manufacturability, 6063 aluminum was chosen. To simplify
the assembly process, fastener types were standardized. The
mast is centered laterally on the ASV, which helps create a
well-defined coordinate system that permits simpler software
transformations. The sensor mast can be seen in Fig 2.

### Electronics Box

NaviGator ASV’s electronics are
housed in a Thule Sidekick cargo box. The team originally
considered commercial waterproof boxes, but began looking
for other options due to their high costs. One student
suggested the idea of using a cargo box after being inspired by
family road trips they had taken when they were younger.
While traditionally used to mount on the top of cars to provide
additional storage, the cargo box was an ideal electronics
enclosure due to its watertight integrity, aerodynamic form
factor, low cost, and a side-opening mechanism that makes it
very easy to access all of the electronic components.
The box’s watertight integrity prevented the team from
using air circulation for cooling. Instead, a combination of
techniques are used to cool the box. First, an adhesive
reflective covering was applied to the lid of the box to reflect
heat generated by solar radiation. Second, the box has an
active water cooling system that is used to remove the heat
generated from the electronic components inside the box.
Fiberglass inserts were used to mount all of the components
inside of the box. These inserts add rigidity to the relatively
flimsy box and make it easy to add or remove components
from the box. The components that need to be frequently
removed, e.g., the hard drives, are attached to the fiberglass
with Velcro. The rest of the components are attached with
traditional fasteners.

### Racquetball Launcher

A system for delivering the
racquetballs into the target for the Detect and Deliver task was
developed by breaking the challenge into subtasks that were
solved independently. The two main subtasks that were
considered were moving the balls into the target and feeding
the balls to the mover. Several ideas for moving the balls were
considered, ranging from a catapult to a robotic arm that
would drop the balls into the target. Prototypes of several
designs were built and tested. One design featured two
counter-rotating wheels attached to the trolling motors that
were once part of PropaGator 1, the team’s submission to the
2013 RoboBoat Competition. The trolling motors were
originally used as part of an early prototype, but since they
were already waterproof, were effective at launching the balls
consistently, and were readily available, the trolling motors
were incorporated into the final design.
The ball launching mechanism was designed so that any
type of ball feeder could be integrated into it. This allowed the
team to test multiple types of ball feeder mechanisms,
including a carousel and a linear actuator. A prototype of the
linear actuator racquetball launcher can be seen in Fig 3. After
the carousel mechanism was found to be prone to jamming,
the linear actuator design was selected. Additional design
criteria that were considered were the ease of loading balls and
how quickly all four balls could be launched. Ball loading was
addressed by designing a spring-loaded 3D printed ball
magazine that is easily detachable. To achieve rapid-fire, the
team developed a closed-bolt system. After a ball is loaded
into the chamber, it is withheld at the minimum distance from
the wheels to reduce the amount of time it takes to fire. In
order to prevent premature firing, a retention lip is used.

### Ring Challenge System

## Electrical System

Robustness and simplicity were the primary motivating
factors behind the design of the NaviGator ASV’s electrical
system. The team focused on these aspects in order to get a
testable system built quickly and minimize any downtime due
to electrical failure.

### Power System

The salient features of the power system
are the dual battery power supply and the power merge board.
The NaviGator ASV’s power requirements surpassed those of

### Power merge board

The power merge board is a
student-designed printed circuit board assembly (PCBA). It
uses two Texas Instruments LM5050 High Side OR-ing FET
controllers as ideal diode rectifiers to balance and parallel the
two batteries into one rail that supplies four output ports. This
makes the system more fault tolerant to a failing battery, a
feature used in normal operation to switch batteries out
without turning the system off. One of the strengths of MIL is
the ability to design hardware and software that can be reused
on other projects and vehicles. This is the third vehicle for
which this board design has been utilized. The design was
originally created for PropaGator 1 and then used on
PropaGator 2, both of which have competed in the RoboBoat
Competition.

### Passive sonar

### Kill system

The hardware kill system consists of two
student-designed PCBAs and four off-the-shelf twist to detent
kill switches. The kill system for the vehicle also has a
software component. The kill board monitors the status of six
kill sources. When any of the six sources request a kill, the kill
board cuts power to the thruster motor controllers. The six kill
sources are the four off-the-shelf switches that are mounted
around the vehicle, a remote kill switch, and the computer.
The remote kill switch operates over a 900 MHz radio link and
displays the hardware kill status of the vehicle. The kill board
is also used to control the NaviGator ASV’s indicator lights
and a siren used to ward off curious watercraft during testing.


## Software System

### Object Detection and Classification

The lowest level
perception service available on the NaviGator ASV is the
Occupancy Grid Server. Occupancy grids are a twodimensional grid-like representation of the environment
generated by the sensor suite available on the ASV. The
generated map contains both the occupied and unoccupied
regions in the environment. This information is provided to
the server via any range-detecting sensor onboard. On the
ASV, the primary range-detecting sensor is a Velodyne VLP16 LIDAR. A LIDAR uses lasers to provide relatively dense
range information of the environment. This information is then
segmented by regions containing dense clusters of relatively
close points. These bounding regions are treated as obstacles,
and are placed in the occupancy grid. This information is then
provided to higher level services such as the motion planner
and Classification Server.
In the Classification Server, the points generated by the
LIDAR are clustered into regions on the occupancy grid where
it decides which of these distinct regions are objects. The ASV
then looks at the bounding box of this object and classifies the
object based on the dimensions of its bounding box. The
software detects if the object has a prominent plane. If it does,
then this information is attached to the object. These objects
are then accessible to other programs through the use of a list
of detected objects.

### Motion Planning

### Motion Control

### Navigation and Odometry

The NaviGator ASV uses a
student-developed Sylphase global positioning system (GPS)
and inertial navigation system (INS) that is in the process of
being commercialized by Forrest Voight, a UF student and
member of Team NaviGator AMS. It primarily consists of a
circuit board with a Spartan-6 field programmable gate array
(FPGA), radio frequency (RF) frontend, inertial measurement
unit (IMU), magnetometer, and a barometer. The FPGA
performs the correlation operations that enable tracking of
GPS satellites. All the sensor measurements and correlations
are passed to a computer via USB, into a pipeline of software
modules that track and decode the signals from the GPS
satellites and then fuse measurements using an extended
Kalman filter into an estimate of the ASV's pose in both
absolute world and relative odometry coordinate frames. Last,
the resulting odometry is transformed so that it describes the
ASV's coordinate frame and it is then passed to ROS.
By using the sensors to aid the GPS solution and taking
advantage of GPS carrier phase measurements, extremely
precise relative odometry is possible, with noise on the order
of centimeters over periods of seconds to minutes. This is the
result of years of work, during which several iterations of the
hardware were produced. The initial version of the hardware
was a Beaglebone cape, but quickly moved to the USB/FPGA
approach for ease of development and reduced CPU load. The
current revision of the hardware is shown in Fig 9.

### State Machine

# Design Strategy

## Find Totems and Avoid Obstacles

## Identify Symbols and Dock

## Scan The Code

# Experimental Results

## Simulator

## Field Testing

In addition to testing in the simulator, NaviGator ASV
underwent significant lake testing. Over 130 hours of in-water
testing were carried out in the form of day-long tests in the
months leading up to the competition at a lake near UF. Lake
testing offered real-life environmental factors that simulation
cannot accurately provide, such as wind and current
disturbances, various lighting conditions, and inclement
weather.
Field testing also offered a chance to test the mechanical
systems of the ASV, such as actuators like the racquetball
launcher, the strength of team-manufactured components, and
the efficiency of the computer cooling system. The frequency
and duration of testing helped to expose hardware failures that
may have gone unnoticed until the competition. For example,
the original sensor mast placed the Ubiquiti omnidirectional
Wi-Fi antenna less than two inches away from the Velodyne
LIDAR. During field testing, the team found that the LIDAR
was returning noisy data. However, when testing in the lab,
the LIDAR data looked fine. Eventually the team determined
that the only difference was that a wired connection was used
to connect to the ASV while working in the lab, as opposed to
the Wi-Fi connection that was used while field testing. It turns
out that the Wi-Fi signal from the antenna was adding noise to
the LIDAR data. Moving the Wi-Fi antenna further from the
LIDAR solved the problem. This kind of issue would never
have arisen during simulation. The detection of this and other
flaws during testing prevented what would have been
catastrophic failures during the competition.
C. Field Element Construction
In order to take full advantage of the realistic testing
environment that the lake provides, field elements similar to
those that will be used in the competition were constructed.
The field elements were designed to be simple in construction
and easy to deploy. Many of the elements were made of a
PVC pipe frame that allowed for modular construction and
easy assembly and disassembly. Buoyancy was provided by
foam sheets and pool noodles fitted around the PVC pipes.
The simplicity and light weight of the course elements allowed
for quick and easy setup and teardown of the course using
only a few team members in a kayak. As an example, the
Identify Symbols and Dock platform that the team constructed
and used for testing can be seen in Fig 13.

# Acknowledgement

# References

