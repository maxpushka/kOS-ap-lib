# kOS rocket AutoPilot Library
kOS rocket Autopilot Library is a modular library created to automate the execution of routine tasks for KSP players. Compatible with [RemoteTech 2](https://remotetechnologiesgroup.github.io/RemoteTech/) mod


## Installation
1. Install latest version of [kOS](https://github.com/KSP-KOS/KOS) mod
2. Download and unzip the project folder into *Kerbal Space Program\Ships\Script* **without changing its structure**<sup>[note](#note)</sup>


## Demo missions
There's a couple of demo missions in *Examples* folder.
To run them, put `.craft` file to 


## Usage
### Import scripts
Simply add `runoncepath("0:/kOS_ap_lib/Lib/Import.ks").` to your script to add the import funtion.

Now you can import the scripts listed under *[Lib structure](#lib-structure)* section with `import("script_name").` while ommiting their extension.
For example, if you want to import `Ascend.ks`, run `import("Ascend").` and the *Ascend* function will be added to your script.

### *Lib* structure
* lib_math
	* [BisectionSolver.ks](#bisectionsolverks)
	* [Derivator.ks](#derivatorks)
* lib_orbit
	* [Change_LAN_Inc.ks](#change_lan_incks)
	* [Circularize.ks](#circularizeks)
	* [HohmannTransfer.ks](#hohmanntransferks)
	* [TransferToMoon.ks](#transfertomoonks)
* lib_phys
	* [BurnTime.ks](#burntimeks)
	* [EngThrustIsp.ks](#engthrustispks)
	* [MachNumber.ks](#machnumberks)
	* [VerticalAccelCalc.ks](#verticalaccelcalcks)
	* [VisVivaCalc.ks](#visvivacalcks)
* [Ascend.ks](#ascendks)
* [Dock.ks](#dockks)
* [Import.ks](#importks)
* [Land.ks](#landks)
* [Rendezvous.ks](#rendezvousks)


## Script guideline
### BisectionSolver.ks
> BisectionSolver(scoreFunction, point1, point2).
> 
> **Parameters:** \
> scoreFunction --> ([`delegate`](https://ksp-kos.github.io/KOS/language/delegates.html))\
> point1 --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar))\
> point2 --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar))\
> **Return type:** [`delegate`](https://ksp-kos.github.io/KOS/language/delegates.html) function\
> **[Delegate](https://ksp-kos.github.io/KOS/language/delegates.html) return type**: [`list`](https://ksp-kos.github.io/KOS/structures/collections/list.html#structure:LIST "LIST structure") of [`scalars`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar)

This script implements [bisection solver](https://en.wikipedia.org/wiki/Bisection_method). First, you need to setup the solver, then to call it just like the regular function or using `:call` suffix. You'll get a list, that contains final search range ([0] and [1] positions) and the answear ( [2] position). The final search range can be reused in loop to get more precise results (see *Usage examples* section).

**Usage examples:**
```
set solver to BisectionSolver(some_func@, 10, 15). // <- solver is now a delegate function
set points1 to solver:call(). // <- points is a list containing roots of some_func
```
```
set points2 to BisectionSolver(some_func@, 10, 15):call().
set points3 to BisectionSolver(some_func@, 10, 15)().
```
```
// Reusage example
set point_data to list(10, 15).
set solver to BisectionSolver(some_func@, point_data[0], point_data[1]).
until *some_condition* {
	set ans to solver:call().
	set point_data[0] to ans[0].
	set point_data[1] to ans[1].
}
```

### Derivator.ks
#### MakeDerivator_N
> MakeDerivator_N(init_value, N_count).
>
> **Parameters:**\
> init_value --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar))\
> N_count --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar)) set to 0 by default\
> **Return:** [`delegate`](https://ksp-kos.github.io/KOS/language/delegates.html) function\
> **[`Delegate`](https://ksp-kos.github.io/KOS/language/delegates.html) parameters**: [`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar)\
> **[`Delegate`](https://ksp-kos.github.io/KOS/language/delegates.html) return type**: the derivative value of *init_value* ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar))

First, you need to setup the derivator, then to call it with parameter just like the regular function or using `:call` suffix. You'll get the `init_value` derivative of order N.

**Usage examples:**
```
set v1 to 10. // [m/s]
set v2 to 15. // [m/s]
set derivator to MakeDerivator_dt(v1, 1).
set acceleration to derivator:call(v2).
```
---
#### MakeDerivator_dt
> MakeDerivator_dt(init_value, time_interval).
>
> **Parameters:**\
> init_value --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar))\
> time_interval --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar), seconds) set to 0 by default\
> **Return type:** [`delegate`](https://ksp-kos.github.io/KOS/language/delegates.html) function\
> **[`Delegate`](https://ksp-kos.github.io/KOS/language/delegates.html) parameters**: [`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar)\
> **[`Delegate`](https://ksp-kos.github.io/KOS/language/delegates.html) return type**: the time derivative value of *init_value* ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar))

First, you need to setup the derivator, then to call it with parameter just like the regular function or using `:call` suffix. You'll get the time derivative of `init_value`.

**Usage examples:**
```
set acceleration1 to 10. // [m/s^2]
set acceleration2 to 15. // [m/s^2]
set derivator to MakeDerivator_dt(acceleration1, 0).
set jerk to derivator:call(acceleration2).
```

### Change_LAN_Inc.ks
> Change_LAN_Inc(DesiredOrbit).
>
> **Parameters:**\
> DesiredOrbit ([`lexicon`](https://ksp-kos.github.io/KOS/structures/collections/lexicon.html) with "LAN" and "INC" fields)\
> "LAN" --> [longitude of ascending node](https://en.wikipedia.org/wiki/Longitude_of_the_ascending_node) in degrees\
> "INC" --> [orbital inclination](https://en.wikipedia.org/wiki/Orbital_inclination) in degrees\
> **Return:** function exit status ([`bool`](https://ksp-kos.github.io/KOS/structures/misc/boolean.html#boolean)); `true` if successfuly completed, otherwise `false`

Performs orbital maneuver to change longitude of ascending node and inclination of orbit. Returns `true` if it is successfuly complete.

**Usage example:**
```
set newOrbit to lexicon().
newOrbit:add("LAN", 78).
newOrbit:add("INC", 6).
Change_LAN_Inc(newOrbit). // performs maneuver to match INC and LAN with Minmus
```

### Circularize.ks
> Circularize(targetAlt).
> 
> **Parameters:**\
> targetAlt ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar) > 0)\
> **Return:** function exit status ([`bool`](https://ksp-kos.github.io/KOS/structures/misc/boolean.html#boolean)); `true` if successfuly completed, otherwise `false`

Performs circularization at specified altitude (*targetAlt* parameter). Returns `true` if maneuver is completed successfuly.
*targetAlt* value should be between periapsis and apoapsis value. If it's not, script will return `false`.

**Usage example:**
```
set newApPe to 100000.
Circularize(newApPe). // performs circularization at altitude of 100000 meters
```

### HohmannTransfer.ks
> HohmannTransfer(targetR, burnIn, autowarp).
> 
> **Parameters:**\
> targetAlt --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar), meters)\
> burnIn --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar), seconds)\
> targetR --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar)) radius of target orbit\
> burnIn --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar)) execute burn in (time:seconds+burnIn) seconds\
> autowarp --> ([`bool`](https://ksp-kos.github.io/KOS/structures/misc/boolean.html#boolean)) autowarp to burn position\
> **Return:** function exit status ([`bool`](https://ksp-kos.github.io/KOS/structures/misc/boolean.html#boolean)); `true` if successfuly completed, otherwise `false`

Executes one-burn [Hohmann transfer](https://en.wikipedia.org/wiki/Hohmann_transfer_orbit) in *burnIn* seconds. Returns `true` if maneuver is completed successfuly.

**Usage example:**
```
set targetR to 50000.
set burnIn to eta:apoapsis.
HohmannTransfer(targetR, burnIn).
```

### TransferToMoon.ks
> TransferToMoon(targetMoon, targetPe, autowarp, insertionBurn).
>
> **Parameters:**\
> targetMoon --> ([`string`](https://ksp-kos.github.io/KOS/structures/misc/string.html#string))\
> targetPe --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar) > 0)\
> autowarp --> ([`bool`](https://ksp-kos.github.io/KOS/structures/misc/boolean.html#boolean)) autowarp to burn position; set to `true` by default\
> insertionBurn --> ([`bool`](https://ksp-kos.github.io/KOS/structures/misc/boolean.html#boolean)) set to `true` by default\
> **Return:** function exit status ([`bool`](https://ksp-kos.github.io/KOS/structures/misc/boolean.html#boolean)); `true` if successfuly completed, otherwise `false`

Performs a set of maneuvers to transfer the vessel to the target Moon. Using this script one can perform Moon flyby (just set *insertionBurn* to `false`).

**Usage example:**
```
set targetMoon to "Mun".
set targetPe to 50000.
TransferToMoon(targetMoon, targetPe).
```

### BurnTime.ks
> BurnTime(dV, Rad).
>
> **Parameters:**\
> dV -->  ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar)) the amount of dV that you need to burn\
> altRad -->  ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar)) altitude above sea level\
> **Return:** seconds ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar))

Calculates the time in seconds required to burn specified amount of dV on 100% throttle. 

**Usage example:**
```
set dV to 500.
set altRad to ship:altitude.
BurnTime(dV, altRad).
```

### EngThrustIsp.ks
> EngThrustIsp().
>
> **Parameters:** takes no parameters\
> **Return:** [`list`](https://ksp-kos.github.io/KOS/structures/collections/list.html#structure:LIST "LIST structure") of [`scalars`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar)\
> **Return list structure:**\
>  [0] --> total current maximum thrust accounting for thrust limiter and fuel availability (kN)\
>  [1] --> total [*I<sub>sp</sub>*](https://en.wikipedia.org/wiki/Specific_impulse) of all active engines (seconds)

Calculates total [thrust](https://en.wikipedia.org/wiki/Thrust) (in kN) and [*I<sub>sp</sub>*](https://en.wikipedia.org/wiki/Specific_impulse) (in seconds) of all currently active engines.

**Usage example:**
```
set engines_data to EngThrustIsp().
set thrust to engines_data[0].
set isp to engines_data[1].
```

### MachNumber.ks
> MachNumber().
>
> **Parameters:** takes no parameters\
> **Return:** Mach number ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar))

Returns current [Mach number](https://en.wikipedia.org/wiki/Mach_number). Returns zero if out of atmosphere

**Usage example:**
`print MachNumber().`

### VerticalAccelCalc.ks
> VerticalAccelCalc().
>
> **Parameters:** takes no parameters\
> **Return:** vertical acceleration value ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar), m/s^2)

Returns current vertical acceleration.

**Usage example:**
`print VerticalAccelCalc().`

### VisVivaCalc.ks
> VisVivaCalc(r, a).
>
> **Parameters:**\
> r --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar)) altitude above sea level\
> a --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar)) [semi-major axis](https://en.wikipedia.org/wiki/Semi-major_and_semi-minor_axes#Energy;_calculation_of_semi-major_axis_from_state_vectors)\
> **Return:** velocity ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar), m/s)

Implementation of [vis-viva_equation](https://en.wikipedia.org/wiki/Vis-viva_equation). Note that *r* parameter is altitude *above sea level*.

**Usage example:**
```
// example dV maneuver calculation
// assuming the ship is at 100000 meter circular orbit
set v1 to velocity:orbit:mag. //current orbital velocity

set r2 to 150000.
set sma2 to 2*r2 + body:radius.
set v2 to VisVivaCalc(r2, sma2).

set dV to v2-v1.
```

### Ascend.ks
> Ascend(targetOrbit, targetIncl, finalPitch, gravTurnAlt, gravTurnV, accLimit, pre_stage, post_stage, jettisonFairing, jettisonAlt, deployAntennas, deploySolar, autowarp).
>
> **Parameters:**\
> targetOrbit --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar), meters) height of the circular target orbit (apoapsis = periapsis)\
> targetIncl --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar), degrees) inclination of the target orbit\
> finalPitch --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar), degrees) angle to the horizon when leaving the atmosphere; set to 85° by default\
> gravTurnAlt --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar), meters) altitude value at which the gravity turn begins; set to 250m by default\
> gravTurnV --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar), m/s) velocity value at which the gravity turn begins; set to 150 m/s by default\
> accLimit --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar), g-force) limits the amount of g-force that the rocket is expose to; set to 3.5 by default\
> pre_stage --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar), seconds) wait before staging; set to 0.5 sec by default\
> post_stage --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar), seconds) wait after staging; set to 1 sec by default\
> jettisonFairing --> ([`bool`](https://ksp-kos.github.io/KOS/structures/misc/boolean.html#boolean), meters) automatic fairing jettison; set to `true` by default\
> jettisonAlt --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar), meters) altitude at which fairing will be jettisoned; set to 50000m by default\
> deployAntennas --> ([`bool`](https://ksp-kos.github.io/KOS/structures/misc/boolean.html#boolean)) automatic antenna deployment; set to `true` by default\
> deploySolar --> ([`bool`](https://ksp-kos.github.io/KOS/structures/misc/boolean.html#boolean)) automatic deployment of solar panels; set to `true` by default\
> autowarp --> ([`bool`](https://ksp-kos.github.io/KOS/structures/misc/boolean.html#boolean)) autowarp to burn position; set to `true` by default\
> **Return:** function exit status ([`bool`](https://ksp-kos.github.io/KOS/structures/misc/boolean.html#boolean)); `true` if successfuly completed, otherwise `false`

Executes liftoff, ascend and circularization burn at specified apoapsis. The script works both on bodies with atmosphere and without. Make sure that the rocket is controllable and well balanced.
* **REQUIRES ACCELEROMETER ONBOARD!**
* Gravity turn begins when gravTurnAlt > current altitude OR gravTurnV  > current velocity
* Fairing jettison, deployment of antennas and solar pannels is executed AFTER the rocket reached both [maximum dynamic pressure](https://en.wikipedia.org/wiki/Dynamic_pressure "Dynamic pressure") (max Q) and an altitude of `jettisonAlt`

**Usage example:**
```
Ascend(100000, 7.5). // 100km orbit with an inclination of 7.5 degrees
```

### Dock.ks
> Dock(targetShip, targetNode, selectedNode, safeDistance).
>
> **Parameters:**\
targetShip --> ([`string`](https://ksp-kos.github.io/KOS/structures/misc/string.html#string)) the name of the target ship\
> targetNode --> ([`string`](https://ksp-kos.github.io/KOS/structures/misc/string.html#string)) tag of  target ship's node you want to dock to\
> selectedNode --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar)) tag of the current ship's node you want to "Control from"\
> safeDistance --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar)) the radius of the keep-out sphere. Set by default to max widespan of the target ship + 100 meters.
> **Return:** function exit status ([`bool`](https://ksp-kos.github.io/KOS/structures/misc/boolean.html#boolean)); `true` if successfuly completed, otherwise `false`

Executes automatic docking. The current ship should be in close proximity of the target ship (idealy less than 2 km away from the target ship's position).
The selected node and target node are identified using tag system. Make sure to tag these parts.
Note: the script uses a lot of %whatever-RCS-fuel-name%, so you can manually switch off RCS for a few seconds and then turn it on again to save some fuel.

**Usage example:**
```
// Assume there's a ship called 'Agena' you want to dock to.
// It has a docking port tagged 'ab'.
// Our ship has the same class node tagged 'n2'

Dock("Agena", "ab", "n2"). // safeDistance is calculated automatically
Dock("Agena", "ab", "n2", 250). // safeDistance is set to 250 meters
```

### Import.ks
> Import("script_name").
> Import(list("script1", "script2")).
> 
> **Parameters:** [`string`](https://ksp-kos.github.io/KOS/structures/misc/string.html#string) *OR* [`list`](https://ksp-kos.github.io/KOS/structures/collections/list.html#structure:LIST "LIST structure") of strings\
> **Return:** function exit status ([`bool`](https://ksp-kos.github.io/KOS/structures/misc/boolean.html#boolean)); `true` if all scripts were successfuly loaded, otherwise `false`

Imports scripts specified with parameters from *Lib* folder. Returns `true` if the import was successful, otherwise `false` and a warning message pops up on a terminal screen.

**Usage examples:**
`import("Ascend").`
`import(list("Ascend", "HohmannTransfer", "Land")).`

### Land.ks
> Land(targetSite, touchdownSpeed, hover_alt).
>
> **Parameters:**\
> targetSite --> ([`latlng`](https://ksp-kos.github.io/KOS/math/geocoordinates.html?highlight=latlng#LATLNG)) geocoordinates of the landsite.\
> touchdownSpeed --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar), m/s) the speed of final touchdown. Set to 1 m/s by default.\
> hover_alt --> ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar), meters) altitude at which the ship will hover, before attempting final touchdown. Set to 50 m by default.\
> **Return:** function exit status ([`bool`](https://ksp-kos.github.io/KOS/structures/misc/boolean.html#boolean)); `true` if successfuly completed, otherwise `false`

Executes landing routine at specific coordinates. The script performs quite well with error margin up to 50 meters from the *targetSite*.

**Usage example:**
```
set targetSite to latlng(154,28).
set touchdownSpeed to 3.75.
set hover_alt to 20.

Land(targetSite, touchdownSpeed, hover_alt).
```

### Rendezvous.ks
> Rendezvous(targetShip, finalDistance, autowarp).
>
> **Parameters:**\
> targetShip --> ([`string`](https://ksp-kos.github.io/KOS/structures/misc/string.html#string)) target ship's name\
> finalDistance -->  ([`scalar`](https://ksp-kos.github.io/KOS/math/scalar.html#scalar), meters) final distance to target ship. Set to 1000 by default.\
> autowarp --> ([`bool`](https://ksp-kos.github.io/KOS/structures/misc/boolean.html#boolean)) autowarp to burn position . Set to false by default.\
> **Return:** function exit status ([`bool`](https://ksp-kos.github.io/KOS/structures/misc/boolean.html#boolean)); `true` if successfuly completed, otherwise `false`

Performs orbital rendezvous. Note that target ship must be in the same sphere of influence as the current ship, otherwise exception will be risen. 
It's highly recommended to leave autowarp set to true.

**Usage example:**
```
set targetShip to "MyShipsName".
set final distance to 125.
Rendezvous(targetShip, finalDistance).
```
```
set targetShip to "ISS".
set final distance to 800.
set autowarp to false.
Rendezvous(targetShip, finalDistance, autowarp).
```

## License
[MIT](https://choosealicense.com/licenses/mit/)

---
#### Note
Though it's not recommended, it's possible to change the structure, but **the *Lib* folder should stay immutable**!
To do this, simply edit the first line of *Import.ks*. The `path` variable points to the place where the *Lib* folder is located:
`global path is "0:/your/path/to/Lib/".`
