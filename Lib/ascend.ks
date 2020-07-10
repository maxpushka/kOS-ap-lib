//Ascend script v1.0
//by maxpushka

//==================== LAUNCH PARAMETERS ===================//
//REQUIRED ACCELEROMETER ONBOARD! CHECK FOR ITS PRESENCE
set targetOrbit to 150000. //[meters] Apoapsis=Periapsis=targetOrbit
set targetIncl to 0. //[degrees] final orbit inclination
set finalPitch to 70.

set gravTurnAlt to 10000. //[meters] altitude at which vessel shall start gravity turn
set gravTurnV to 1500. //[m/s] velocity at which vessel shall start gravity turn
//gravity turn start when ship's altitute == gravTurnAlt OR ground velocity == gravTurnV
//on bodies without atmosphere these parameters have no effect.

set accLimit to false. //[g] acceleration limiter. may be set to false
set pre_stage to 0.5.
set past_stage to 1.
set deployAntennas to false.
set deploySolar to false.
set jettisonFairing to false.
set autoWarp to true.

//======================== PRELAUNCH =======================//

//IMPORTS
runoncepath("0:/kOS_ap_lib/Lib/lib_phys/VerticalAccelCalc.ks").
runoncepath("0:/kOS_ap_lib/Lib/lib_phys/MachNumber.ks").
runoncepath("0:/kOS_ap_lib/Lib/lib_math/BisectionSolver1.ks").
runoncepath("0:/kOS_ap_lib/Lib/lib_math/Derivator.ks").

//CLEARING WORKSPACE
clearscreen.
AG1 on. //open terminal
SET SHIP:CONTROL:NEUTRALIZE TO TRUE. //block user control inputs
set ship:control:pilotmainthrottle to 0. //block user throttle inputs
sas on.

//STEERING SETTING
if body:atm:exists {set pitch_ang to 0.} else {set pitch_ang to 60.} //starting pitch angle
set compass to AzimuthCalc(targetIncl).
lock steering to lookdirup(heading(compass,90-pitch_ang):vector,ship:facing:upvector).

//FLIGHT MODE PARAMETERS
set throttleStage to 1. //more details under gravity turn logic
set ascendStage to 1.

//PITCH_CALC PARAMETERS
set v_jerk_func to makeDerivator_N(0,20).

set Tpoints to lexicon().
Tpoints:add("StartPoint",100).
Tpoints:add("EndPoint",101).

set Pitch_Data to lexicon().
Pitch_Data:ADD("Time",time:seconds).
Pitch_Data:ADD("Time_to_Alt",0).
Pitch_Data:ADD("Pitch_Final",finalPitch).
if body:atm:exists {
	Pitch_Data:ADD("Alt_Final",ship:body:atm:height).
	Pitch_Data:ADD("Pitch",pitch_ang). //starting pitch angle
	
	lock throttle to 1.
}
else {
	Pitch_Data:ADD("Alt_Final",0.1*targetOrbit).
	Pitch_Data:ADD("Pitch",pitch_ang). //starting pitch angle
	
	local g_alt is body:Mu/(ship:body:radius + ship:altitude)^2. //ускорение свободного падения на текущей высоте
	if accLimit=false {lock throttle to (ship:mass^2*g_alt)/EngThrustIsp()[0].}
	else {lock throttle to MIN( (ship:mass^2*g_alt*(accLimit/2))/EngThrustIsp()[0], 1).}
}

//DeltaV_Calc PARAMETERS
set DeltaV_Data to lexicon().
DeltaV_Data:ADD("Total",0).
DeltaV_Data:ADD("Gain",0).
DeltaV_Data:ADD("Time",time:seconds).
DeltaV_Data:ADD("Thrust_Accel",throttle*availablethrust/mass).
DeltaV_Data:ADD("Accel_Vec",throttle*ship:sensors:acc).

wait 1.

//========================= ASCEND =========================//

//STAGING
local current_max to maxthrust.
when maxthrust < current_max OR availablethrust = 0 then {
	set prevThrottle to throttle.
	lock throttle to 0.
	if not(pre_stage = false) {wait pre_stage.}
	stage.
	if not(past_stage = false) {wait past_stage.}
	lock throttle to prevThrottle.
	set current_max to maxthrust.
	preserve.
}

//AUTO DEPLOY
if deployAntennas = true {
	when (MachNumber() > 1.1) AND (ship:q*constant:ATMtokPa < 3000) then {
		print "".
	}
}
if deploySolar = true {
	when (MachNumber() > 1.1) AND (ship:q*constant:ATMtokPa < 3000) then {
		PANELS ON.
	}
}
if jettisonFairing = true {
	when (MachNumber() > 1.1) AND (ship:q*constant:ATMtokPa < 3000) then {
		print "".
	}
}
	
//PRE GRAVITY TURN LOGIC	
rcs on.
if body:atm:exists {
	until (altitude > gravTurnAlt) OR (ship:verticalspeed > gravTurnV) {
		local line is 1.
		print "gravTurnAlt = " + gravTurnAlt + "   " at(0,line).
		local line is line + 1.
		print "altitude    = " + round(altitude) + "   " at(0,line).
		local line is line + 2.
		print "gravTurnV   = " + gravTurnV + "   " at(0,line).
		local line is line + 1.
		print "Velocity    = " + round(ship:verticalspeed, 1) + "   " at(0,line).
		local line is line + 1.
		print "Mach Number = " + MachNumber() + "   " at(0,line).
	}
}
clearscreen.
sas off.

//GRAVITY TURN LOGIC
until ascendStage = 3 {
	// Run Mode Logic
	//throttleStage:      //ascendStage:
	//1=engine burning    //1=powered ascend; engine burning, apoapsis<targetOrbit
	//2=apo correction    //2=flight on a ballistic trajectory in the atmosphere; apo corection, apoapsis>=targetOrbit
	//3=MECO              //3=flight on a ballistic trajectory outside the atmosphere; MECO, apoapsis>=targetOrbit
	
	//it's important to lock throttle beth [0; 1] as it's value is used in DeltaV_Calc function calculations
	
	if apoapsis > targetOrbit AND altitude > ship:body:ATM:height {
		lock throttle to 0.
		set ascendStage to 3.
		set throttleStage to 3.
		clearscreen.
	}
	
	if apoapsis < targetOrbit AND throttleStage = 1{ //while ascendStage=1
		if ship:body:atm:exists { //body has atmosphere
			if accLimit=false {
				lock throttle to MIN(MAX((targetOrbit-apoapsis)/1000, 0.005), 1).
			}
			else {
				local g_alt is body:Mu/(ship:body:radius + ship:altitude)^2.
				lock throttle to MIN(MAX( (ship:mass^2*g_alt*(accLimit/2))/EngThrustIsp()[0], 0.001 ), 1).
			}
		}
		else { //body has no atmosphere
			if accLimit=false {
				lock throttle to (ship:mass^2*g_alt)/EngThrustIsp()[0].
			}
			else {
				local g_alt is body:Mu/(ship:body:radius + ship:altitude)^2.
				lock throttle to MIN(MAX( (ship:mass^2*g_alt*(accLimit/2))/EngThrustIsp()[0], 0.001 ), 1). //(ship:mass^2*g_alt) //(targetOrbit-apoapsis)/EngThrustIsp()[0]/100
			}
		}
	}
	else if apoapsis > targetOrbit AND (throttleStage = 1 OR throttleStage = 2) { //switching to ascendStage=2
		lock throttle to 0.
		rcs off.
		set throttleStage to 2.
		set ascendStage to 2.
	}
	else if apoapsis < targetOrbit AND throttleStage = 2 { //while ascendStage=2 (apoapsis corection due to drag loss)
		rcs on.
		lock throttle to MIN(MAX((targetOrbit-apoapsis)/100, 0.01), 1).
	}	
  
	set Pitch_Data to Pitch_Calc(Pitch_Data).
	set pitch_ang to Pitch_Data["Pitch"].
	set compass to AzimuthCalc(targetIncl).	
	
	// Variable Printout
	local line is 1.
	print "ascend stage  = " + ascendStage + "   " at(0,line).
	local line is line + 1.
	print "throttleStage = " + throttleStage + "   " at(0,line).
	local line is line + 1.
	print "throttle      = " + round(throttle*100, 2) + " %   " at(0,line).
	local line is line + 1.
	print "Mach Number   = " + MachNumber() + "   " at(0, line).
	local line is line + 2.
	print "pitch angle   = " + round(pitch_ang,2) + "   " at(0,line).
	local line is line + 1.
	print "compass       = " + round(compass,2) + "   " at(0,line).
	local line is line + 2.
	print "altitude      = " + round(altitude,1) + "   " at(0,line).
	local line is line + 1.
	print "apoapsis      = " + round(apoapsis,1) + "   " at(0,line).
	local line is line + 1.
	print "target apo    = " + targetOrbit + "   " at(0,line).
	local line is line + 2.
	print "orbit incl    = " + round(orbit:inclination,5) + "   " at(0,line).
	local line is line + 1.
	print "target incl   = " + targetIncl + "   " at(0,line).
	
	// Delta V Calculations
	set DeltaV_Data to DeltaV_Calc(DeltaV_Data).
	
	// Delta V Printout
	set line to line + 2.
	print "DeltaV_total  = " + round(DeltaV_Data["Total"]) + "   " at(0,line).
	set line to line + 1.
	print "DeltaV_gain   = " + round(DeltaV_Data["Gain"]) + "   " at(0,line).
	set line to line + 1.
	print "DeltaV_Losses = " + round(DeltaV_Data["Total"] - DeltaV_Data["Gain"]) + "   " at(0,line).
	set line to line + 1.
	print "DeltaV_Eff    = " + round(100*DeltaV_Data["Gain"]/DeltaV_Data["Total"]) + "%   " at(0,line).
	
	wait 0.001. //wait for the next physics tick
}

//==================== CIRCULARIZATION =====================//

clearscreen.
if autoWarp = true {kuniverse:timewarp:warpto(time:seconds + ETA:apoapsis - 5).}
until ETA:apoapsis < 1 {
	print "ETA:apoapsis = " + round(ETA:apoapsis,1) at (0,1).
	wait 0.5.
}
sas off.
rcs on.
wait 1.

clearscreen.
set stopburn to false.
lock throttle to 1.
until stopburn {
	set data to BurnData().
	//BurnData()
	//return list(vec, pitchVec, inclVec, fi, dI, dA, Vh, Vz, Vorb, dVorb, dVincl, dVtotal).
	//            0  , 1       , 2      , 3 , 4 , 5 , 6 , 7 , 8   , 9,   , 10    , 11
	
	lock steering to data[0].
	if (data[9]<0) { //dVorb < 0
		lock throttle to 0.
		unlock throttle.
		unlock steering.
		set stopburn to true.
		sas on.
	}
	else if (data[0]:mag < 50) {
		lock throttle to MIN(MAX(data[0]:mag/1000, 0.005), 1).
	}
	
	set v1 to VECDRAW(V(0,0,0), data[2], RGB(255,0,0), "dVincl", 1.0, TRUE, 0.2, TRUE, TRUE).
	set v2 to VECDRAW(V(0,0,0), data[1], RGB(0,255,0), "dVorb", 1.0, TRUE, 0.2, TRUE, TRUE).
	set v3 to VECDRAW(V(0,0,0), data[0], RGB(255,255,255), "dV", 1.0, TRUE, 0.2, TRUE, TRUE).
	
	local line is 1.
	print "Fi      = " + data[3] + "   " at(0,line).
	local line is line + 1.
	print "dI      = " + data[4] + "   " at(0,line).
	local line is line + 1.
	print "dA      = " + data[5] + "   " at(0,line).
	local line is line + 2.
	print "Vh      = " + data[6] + "   " at(0,line).
	local line is line + 1.
	print "Vz      = " + data[7] + "   " at(0,line).
	local line is line + 2.
	print "Vorb    = " + data[8] + "   " at(0,line).
	local line is line + 1.
	print "dVorb   = " + data[9] + "   " at(0,line).
	local line is line + 1.
	print "dVincl  = " + data[10] + "   " at(0,line).
	local line is line + 1.
	print "dVtotal = " + data[11] + "   " at(0,line).
	
	wait 0.001.
	
	set v1:show to false.
	set v2:show to false.
	set v3:show to false.
}
clearscreen.
print "Circularization complete".
orbitData().

//======================= FUNCTIONS ========================//

function DeltaV_Calc {
	parameter DeltaV_Data.
	
	local thrust_accel_1 to DeltaV_Data["Thrust_Accel"].
	local thrust_accel_2 to throttle*availablethrust/mass.
	local a_vec1 to DeltaV_Data["Accel_Vec"].
	local a_vec2 to throttle*ship:sensors:acc.
	local time1 to DeltaV_Data["Time"].
	local time2 to time:seconds.
	local dt to max(0.0001,time2 - time1).
	local thrust_accel to (thrust_accel_1 + thrust_accel_2)/2.
	local a_vec to (a_vec1 + a_vec2)/2.
	local thrust_vec to thrust_accel*ship:facing:vector.
	set DeltaV_Data["Total"] to DeltaV_Data["Total"] + thrust_accel*dt.
	local obt_vel_norm to ship:velocity:orbit:normalized.
	set DeltaV_Data["Gain"] to DeltaV_Data["Gain"] + dt*(VDOT(obt_vel_norm,a_vec)).
	
	set DeltaV_Data["Time"] to time2.
	set DeltaV_Data["Accel_Vec"] to a_vec2.
	set DeltaV_Data["Thrust_Accel"] to thrust_accel_2.
	
	return DeltaV_Data.
}

function Pitch_Calc {
	parameter Pitch_Data.
	local t_1 to Pitch_Data["Time"].
	local t_2 to time:seconds.
	local dt to max(0.0001,t_2 - t_1).
	local alt_final is Pitch_Data["Alt_Final"].
	local alt_diff is alt_final - altitude.
	
	local c to -alt_diff.
	
	set solver to BisectionSolver(t_func@, Tpoints["StartPoint"], Tpoints["EndPoint"]).
	set p to solver:call().
	set Tpoints["StartPoint"] to p[0][0].	
	set Tpoints["EndPoint"] to p[1][0].	
	local time_to_alt to p[2][0].
	
	// set solver to BisectionSolver(t_func@, Tpoints["StartPoint"], Tpoints["EndPoint"]).
	// set Tpoints["StartPoint"] to solver[0].	
	// set Tpoints["EndPoint"] to solver[1].
	// local time_to_alt to solver[2].
	
	print "Leave to atmo in " + round(time_to_alt,2) + " sec" at(0,21).
	
	set Pitch_Data["Time"] to t_2.
	set Pitch_Data["Time_to_Alt"] to time_to_alt.
	if MachNumber() < 0.85 OR MachNumber() > 1.1 { //Max Q is reached when speed = 1 mach
		local pitch_des to Pitch_Data["Pitch"].
		local pitch_final to Pitch_Data["Pitch_Final"].
		local pitch_rate to max(0,(pitch_final - pitch_des)/time_to_alt).
		local pitch_des to min(pitch_final,max(0,pitch_des + dt*pitch_rate)).
		
		set Pitch_Data["Pitch"] to pitch_des.
	}
	
	return Pitch_Data.
}

function getVertJerk { //calculates jerk value
  return v_jerk_func:call(VerticalAccelCalc()).
}

function t_func {
	parameter t.

	local d is altitude.
	local c is verticalspeed.
	local b is VerticalAccelCalc()/2.
	local a is getVertJerk()/6.

	local eq is d + c*t + b*t^2 + a*t^3.
	
	return ship:body:atm:height - eq.
}

function AzimuthCalc {
	//https://www.orbiterwiki.org/wiki/Launch_Azimuth
	parameter targetIncl.
	
	// find orbital velocity for a circular orbit at the current altitude.
	local V_orb is max(ship:velocity:orbit:mag + 1,sqrt( body:mu / ( ship:altitude + body:radius))).
	
	// Use the current orbital velocity
	//local V_orb is ship:velocity:orbit:mag.
	
	// project desired orbit onto surface heading
	local az_orb is arcsin ( max(-1,min(1,cos(targetIncl) / cos(ship:latitude)))).
	if (targetIncl < 0) {
		set az_orb to 180 - az_orb.
	}
	
	// create desired orbit velocity vector
	local V_star is heading(az_orb, 0)*v(0, 0, V_orb).

	// find horizontal component of current orbital velocity vector
	local V_ship_h is ship:velocity:orbit - vdot(ship:velocity:orbit, up:vector:normalized)*up:vector:normalized.
	
	// calculate difference between desired orbital vector and current (this is the direction we go)
	local V_corr is V_star - V_ship_h.
	
	// project the velocity correction vector onto north and east directions
	local vel_n is vdot(V_corr, ship:north:vector).
	local vel_e is vdot(V_corr, heading(90,0):vector).
	
	// calculate compass heading
	local az_corr is arctan2(vel_e, vel_n).
	return az_corr.
}

function BurnData {
	set eng to EngThrustIsp.
	set inclVec to InclData(targetIncl). //return list(inclVec, dVincl).
	
	set Rad to ship:body:radius + ship:altitude.
	set g_alt to body:Mu/Rad^2. //ускорение свободного падения на текущей высоте
	set Vh to VXCL(ship:up:vector, ship:velocity:orbit):mag. //горизонтальная скорость
	set Vz to ship:verticalspeed. //вертикальная скорость
	set Vorb to sqrt(ship:body:Mu/Rad). //1 косм. на текущей высоте
	set Acentr to Vh^2/Rad. //центростремительное ускорение
	
	set AThr to eng[0]*Throttle/ship:mass. //моментальное ускорение сообщаемое движками
	if AThr = 0 {set AThr to 10^(-5).} //если двигатель выключен
	set dVorb to Vorb-Vh.
	set dA to g_alt-Acentr-Vz.
	set fi to ARCSIN(Min(Max(dA/AThr,-1), 1)).
	
	set dVtotal to dVorb + abs(inclVec:mag).
	set pitchVec to Heading(90-targetIncl, fi):vector*dVorb.
	set vec to pitchVec + inclVec.
	
	return list(vec, pitchVec, inclVec, fi, dI, dA, Vh, Vz, Vorb, dVorb, inclVec:mag, dVtotal).
}

function InclData {
	parameter targetIncl.
	
	set dI to targetIncl-orbit:inclination.
	set ecc to ship:orbit:eccentricity.
	set w to ship:orbit:argumentofperiapsis.
	set f to ship:orbit:trueanomaly.
	set n to (2*constant:pi)/ship:orbit:period. // (1/ship:orbit:period).
	set a to ship:orbit:semimajoraxis.
	
	//delta V required to change orbit inclination
	set dVincl to (2*sin(dI/2)*sqrt(1-ecc^2)*cos(w+f)*n*a)/(1+ecc*cos(f)).
	
	set normalVec to vcrs(ship:velocity:orbit,-body:position):NORMALIZED. //normal unit vector
	set inclVec to dVincl*normalVec.
	
	return inclVec.
}
	
function EngThrustIsp {
	list engines in allEngines.
	
	set ActiveEng to list().
	ActiveEng:clear.
	
	set eng_isp to 0.
	set eng_thrust to 0.
	
	for eng in allEngines {
		if eng:ignition and (not eng:flameout) {
			ActiveEng:add(eng).
		}
	}
	
	for eng in ActiveEng {
		set eng_thrust to eng_thrust + eng:availablethrust.
		set eng_isp to eng_isp + eng:isp.
	}
	
	if (ActiveEng:length=0) {return list(0,0).}
	else {return list(eng_thrust, eng_isp/ActiveEng:length).}
}

function orbitData {
	print "".
	print "ORBIT PARAMETERS".
	print "================".
	print "Apoapsis: " + round(ORBIT:APOAPSIS, 1).
	print "Periapsis: " + round(ORBIT:PERIAPSIS, 1).
	print "Eccentricity: " + ORBIT:ECCENTRICITY.
	print "Inclination: " + round(ORBIT:INCLINATION, 5).
	print "Longitude of ascending node: " + round(ORBIT:LAN, 2).
	print "================".
}
