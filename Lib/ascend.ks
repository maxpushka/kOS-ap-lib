//Ascend script v1.0
//by maxpushka
function ascend {
	//==================== LAUNCH PARAMETERS ===================//
	//REQUIRED ACCELEROMETER ONBOARD! CHECK FOR ITS PRESENCE
	set targetOrbit to 100000. //[m] Apoapsis=Periapsis=targetOrbit
	set targetIncl to 0. //[deg] final orbit inclination
	set finalPitch to 85. //[deg]

	set gravTurnAlt to 100. //[m] radar altitude at which vessel shall start gravity turn
	set gravTurnV to 150. //[m/s] velocity at which vessel shall start gravity turn
	//gravity turn start when ship's altitute == gravTurnAlt OR ground velocity == gravTurnV
	//on bodies without atmosphere these parameters have no effect.

	set accLimit to 5.5. //[g] limit acceleration. may be set to false

	set pre_stage to 0.5. //[s] pre staging delay
	set post_stage to 1. //[s] post staging delay

	set jettisonFairing to true. //auto-stage fairing when reached jettisonAlt and Q < 4 kPa
	set jettisonAlt to 50000.//[m] alt at which fairing shall be jettisoned
	set deployAntennas to true. //auto-deploy antennas
	set deploySolar to true. //auto-deploy solar pannels

	set autoWarp to true. //auto warp to apoapsis burn

	//==================== PARAMETERS CHECK ====================//

	if (targetOrbit:typename <> "Scalar") OR (targetIncl:typename <> "Scalar") OR 
	(finalPitch:typename <> "Scalar") OR (gravTurnAlt:typename <> "Scalar") OR 
	(gravTurnV:typename <> "Scalar") OR (jettisonAlt:typename <> "Scalar") OR 
	NOT((pre_stage:typename = "Scalar") OR (pre_stage:typename = "Boolean")) OR 
	NOT((post_stage:typename = "Scalar") OR (pre_stage:typename = "Boolean")) OR  
	NOT(accLimit:typename = "Scalar" OR accLimit:typename = "Boolean") OR 
	NOT(jettisonFairing:typename = "Boolean") OR NOT(deployAntennas:typename = "Boolean") OR 
	NOT(deploySolar:typename = "Boolean") OR NOT(autoWarp:typename = "Boolean") {
		print "Error: check inputs".
		print 1/0.
	}
	else if (targetOrbit < 0) OR (targetIncl < 0) OR (finalPitch < 0) OR 
	(gravTurnAlt < 0) OR (gravTurnV < 0) OR (jettisonAlt < 0) OR 
	((pre_stage:typename = "Scalar") AND (pre_stage < 0)) OR 
	((post_stage:typename = "Scalar") AND (post_stage < 0)) OR
	((accLimit:typename = "Scalar") AND (accLimit < 0)) {
		print "Error: check inputs".
		print 1/0.
	}
	else {
		print "Parameters check is passed successfuly.".
	}

	//======================== PRELAUNCH =======================//

	//IMPORTS
	runoncepath("0:/kOS_ap_lib/Lib/lib_phys/VerticalAccelCalc.ks").
	runoncepath("0:/kOS_ap_lib/Lib/lib_phys/MachNumber.ks").
	runoncepath("0:/kOS_ap_lib/Lib/lib_phys/EngThrustIsp.ks").
	runoncepath("0:/kOS_ap_lib/Lib/lib_math/BisectionSolver1.ks").
	runoncepath("0:/kOS_ap_lib/Lib/lib_math/Derivator.ks").

	//CLEARING WORKSPACE
	clearscreen.
	SET SHIP:CONTROL:NEUTRALIZE TO TRUE. //block user control inputs
	set ship:control:pilotmainthrottle to 0. //block user throttle inputs

	//STARTING PARAMETERS
	if accLimit = false {set accLimit to 50.} //max 
	else if (accLimit <> false) AND NOT(body:atm:exists) {
		set accLimit to accLimit^2.
	}

	//PITCH_CALC PARAMETERS
	set v_jerk_func to makeDerivator_N(0,20).

	set Tpoints to lexicon().
	Tpoints:add("StartPoint",110).
	Tpoints:add("EndPoint",115).

	set Pitch_Data to lexicon().
	Pitch_Data:ADD("Time",time:seconds).
	Pitch_Data:ADD("Time_to_Alt",0).
	Pitch_Data:ADD("Pitch_Final",finalPitch).
	if body:atm:exists {
		Pitch_Data:ADD("Pitch",0). //starting pitch angle
		Pitch_Data:ADD("Alt_Final",ship:body:atm:height).
	}
	else {
		Pitch_Data:ADD("Pitch",60). //starting pitch angle
		Pitch_Data:ADD("Alt_Final",0.1*targetOrbit).
	}

	//DeltaV_Calc PARAMETERS
	set DeltaV_Data to lexicon().
	DeltaV_Data:ADD("Total",0).
	DeltaV_Data:ADD("Gain",0).
	DeltaV_Data:ADD("Time",time:seconds).
	DeltaV_Data:ADD("Thrust_Accel",throttle*availablethrust/mass).
	DeltaV_Data:ADD("Accel_Vec",throttle*ship:sensors:acc).

	//AUTO DEPLOY
	local deploy_bool is false.
	if jettisonFairing = true {
		when (altitude > jettisonAlt) AND (ship:q*constant:ATMtokPa < 4000) then {
			list parts in fairing.
			for f in fairing {
				set tmp to ""+f.
				if tmp:MATCHESPATTERN("Fairing") {
					f:getmodule("ModuleProceduralFairing"):doaction("сбросить", true).
				}
			}
			set deploy_bool to true.
		}
	}
	else {
		when (altitude > 50000) AND (ship:q*constant:ATMtokPa < 4000) then {
			set deploy_bool to true.
		}
	}

	if (deployAntennas = true) {
		when deploy_bool then {
			list parts in antennas.
			for ant in antennas {
				set tmp to ""+ant.
				if tmp:MATCHESPATTERN("Antenna") OR tmp:MATCHESPATTERN("Dish") {
					if ant:HASMODULE("ModuleRTAntenna") { //for RemoteTech 2 mod
						ant:getmodule("ModuleRTAntenna"):doaction("activate", true).
						ant:getmodule("ModuleRTAntenna"):setfield("target", "Mission Control").
					}
					else if ant:HASMODULE("ModuleDeployableAntenna") {
						ant:getmodule("ModuleDeployableAntenna"):doaction("раскрыть антенну", true).
					}
				}
			}
		}
	}

	if (deploySolar = true) {
		when deploy_bool then {PANELS ON.}
	}

	wait 1.

	//========================= ASCEND =========================//

	//LIFTOFF
	if (availablethrust = 0) {stage.}
	if body:atm:exists {
		sas on.
		lock throttle to 1.
	}
	else {
		sas off.
		local AThr is EngThrustIsp().
		local g_alt is body:Mu/(ship:body:radius + ship:altitude)^2.
		lock throttle to MIN(MAX( (g_alt^2*mass*(accLimit-1))/AThr[0], 0.001 ), 1).
	}
	rcs on.
	wait 1.
		
	//STAGING LOGIC
	local current_max is maxthrust.
	when maxthrust < current_max OR availablethrust = 0 then {
		set prevThrottle to throttle.
		lock throttle to 0.
		if not(pre_stage = false) {wait pre_stage.}
		stage.
		if not(post_stage = false) {wait post_stage.}
		lock throttle to prevThrottle.
		set current_max to maxthrust.
		preserve.
	}
		
	//PRE GRAVITY TURN LOGIC
	until (alt:radar > gravTurnAlt) OR (ship:verticalspeed > gravTurnV) {
		lock steering to lookdirup(heading(90,90):vector,ship:facing:upvector).
		
		local line is 1.
		print "gravTurnAlt = " + gravTurnAlt + "   " at(0,line).
		local line is line + 1.
		print "altitude    = " + round(alt:radar) + "   " at(0,line).
		local line is line + 2.
		print "gravTurnV   = " + gravTurnV + "   " at(0,line).
		local line is line + 1.
		print "Velocity    = " + round(ship:verticalspeed, 1) + "   " at(0,line).
		local line is line + 1.
		if body:atm:exists {print "Mach Number = " + MachNumber() + "   " at(0,line).}
	}
	clearscreen.
	sas off.
	GEAR OFF.

	//GRAVITY TURN LOGIC
	set ascendStage to 1.
	set pitch_ang to Pitch_Calc(Pitch_Data)["Pitch"].
	set compass to AzimuthCalc(targetIncl).
	lock steering to lookdirup(heading(compass,90-pitch_ang):vector,ship:facing:upvector).
	until (apoapsis >= targetOrbit AND altitude > ship:body:ATM:height) {
		//Ascend stages:
		//1=powered ascend; engine burning, apoapsis<targetOrbit
		//2=flight on a ballistic trajectory in the atmosphere; apo corection, apoapsis>=targetOrbit
		//3=flight on a ballistic trajectory outside the atmosphere; MECO, apoapsis>=targetOrbit
		
		//it's important to lock throttle in range [0; 1] as it's value is used in DeltaV_Calc()
		
		if (apoapsis/targetOrbit < 0.9) { //while ascend stage = 1
			local AThr is EngThrustIsp().
			local g_alt is body:Mu/(ship:body:radius + ship:altitude)^2.
			lock throttle to MIN(MAX( (g_alt^2*mass*(accLimit-1))/AThr[0], 0.001 ), 1).
		}
		else if (apoapsis/targetOrbit > 0.9) AND (apoapsis < targetOrbit) { //while ascend stage = 2
			when 1 then {set ascendStage to 2.}
			lock throttle to MIN(MAX((targetOrbit-apoapsis)/1000, 0.001), 1).
		}
		else {lock throttle to 0.}
	  
		//set Pitch_Data to Pitch_Calc(Pitch_Data).
		set pitch_ang to Pitch_Calc(Pitch_Data)["Pitch"].
		set compass to AzimuthCalc(targetIncl).
		set DeltaV_Data to DeltaV_Calc(DeltaV_Data).

		// Variable Printout
		local line is 1.
		print "ascend stage  = " + ascendStage + "   " at(0,line).
		local line is line + 1.
		print "throttle      = " + round(throttle*100, 2) + " %   " at(0,line).
		local line is line + 1.
		
		if body:atm:exists {
			print "Mach Number   = " + MachNumber() + "   " at(0,line).
			set line to line + 2.
		}
		else {set line to line + 1.}
		
		print "pitch angle   = " + round(pitch_ang,2) + "   " at(0,line).
		set line to line + 1.
		print "compass       = " + round(compass,2) + "   " at(0,line).
		set line to line + 1.
		print "altitude      = " + round(altitude,1) + "   " at(0,line).
		set line to line + 1.
		print "apoapsis      = " + round(apoapsis,1) + "   " at(0,line).
		set line to line + 1.
		print "target apo    = " + targetOrbit + "   " at(0,line).
		set line to line + 2.
		print "orbit incl    = " + round(orbit:inclination,5) + "   " at(0,line).
		set line to line + 1.
		print "target incl   = " + targetIncl + "   " at(0,line).
		// Delta V Printout
		set line to line + 2.
		print "DeltaV_total  = " + round(DeltaV_Data["Total"]) + "   " at(0,line).
		set line to line + 1.
		print "DeltaV_gain   = " + round(DeltaV_Data["Gain"]) + "   " at(0,line).
		set line to line + 1.
		print "DeltaV_Losses = " + round(DeltaV_Data["Total"] - DeltaV_Data["Gain"]) + "   " at(0,line).
		set line to line + 1.
		print "DeltaV_Eff    = " + round(100*DeltaV_Data["Gain"]/DeltaV_Data["Total"]) + "%   " at(0,line).
		
		wait 0. //wait for the next physics tick
	}
	lock throttle to 0.
	rcs off.
	clearscreen.
	wait 1.

	//==================== CIRCULARIZATION =====================//

	if autoWarp = true {kuniverse:timewarp:warpto(time:seconds + ETA:apoapsis - 5).}
	until ETA:apoapsis < 1 {
		print "ETA:apoapsis = " + round(ETA:apoapsis,1) at (0,1).
		wait 0.5.
	}
	sas off.
	rcs on.
	wait 1.
	
	clearscreen.
	lock throttle to 1.
	set data to BurnData().
	until (data[9]<0) { //dVorb < 0
		set data to BurnData().
		//BurnData()
		//return list(vec, pitchVec, inclVec, fi, dI, dA, Vh, Vz, Vorb, dVorb, dVincl, dVtotal).
		//            0  , 1       , 2      , 3 , 4 , 5 , 6 , 7 , 8   , 9,   , 10    , 11
		
		lock steering to data[0].
		if (data[0]:mag < 50) {
			lock throttle to MIN(MAX(data[0]:mag/1000, 0.005), 1).
		}
		
		set v1 to VECDRAW(V(0,0,0), data[2], RGB(255,0,0), "dVincl", 1.0, TRUE, 0.2, TRUE, TRUE).
		set v2 to VECDRAW(V(0,0,0), data[1], RGB(0,255,0), "dVorb", 1.0, TRUE, 0.2, TRUE, TRUE).
		set v3 to VECDRAW(V(0,0,0), data[0], RGB(255,255,255), "dVtotal", 1.0, TRUE, 0.2, TRUE, TRUE).
		
		local line is 1.
		print "Fi      = " + data[3] + "   " at(0,line).
		local line is line + 1.
		print "dI      = " + data[4] + "   " at(0,line).
		local line is line + 1.
		print "dA      = " + data[5] + "   " at(0,line).
		local line is line + 1.
		print "AThr    = " + round(EngThrustIsp()[0]*Throttle/ship:mass, 2) + "   " at(0,line).
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
	}
	lock throttle to 0.
	sas on.
	unlock throttle.
	unlock steering.

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
		
		if body:atm:exists {
			print "Leaving atmo in " + round(time_to_alt,1) + " sec" at(0,21).
		}
		
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
		local eng is EngThrustIsp(). //return list(thrust, average_isp).
		local inclVec is InclData(targetIncl). //return list(inclVec, dVincl).
		
		local Rad is ship:body:radius + ship:altitude.
		local g_alt is body:Mu/Rad^2. //ускорение свободного падения на текущей высоте
		local Vh is VXCL(ship:up:vector, ship:velocity:orbit):mag. //горизонтальная скорость
		local Vz is ship:verticalspeed. //вертикальная скорость
		local Vorb is sqrt(ship:body:Mu/Rad). //1 косм. на текущей высоте
		local Acentr is Vh^2/Rad. //центростремительное ускорение
		
		local AThr is eng[0]*Throttle/ship:mass. //моментальное ускорение сообщаемое движками
		local dVorb is Vorb-Vh.
		local dA is g_alt-Acentr-Vz.
		local fi is ARCSIN(Min(Max(dA/AThr,-1), 1)).
		
		local dVtotal is dVorb + abs(inclVec:mag).
		set pitchVec to Heading(90-targetIncl, fi):vector*dVorb.
		local vect is pitchVec + inclVec.
		
		return list(vect, pitchVec, inclVec, fi, dI, dA, Vh, Vz, Vorb, dVorb, inclVec:mag, dVtotal).
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
}
