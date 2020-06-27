print "Ascend script v1.0".

//==================== LAUNCH PARAMETERS ===================//
set targetOrbit to 100000. //target orbit in meters, Apoapsis=Periapsis=targetOrbit
set targetIncl to 90. //final orbit inclination in degrees

//gravity turn start when ship's altitute == gravTurnAlt and/or velocity == gravTurnV
set gravTurnAlt to 250. //[meters] altitude at which vessel shall start gravity turn
set gravTurnV to 150. //[m/s] velocity at which vessel shall start gravity turn
//========================= ASCEND =========================//

//PRELAUCNH
SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
set ship:control:pilotmainthrottle to 0.
sas off.
rcs on.
clearscreen.
AG1 on. //open terminal
lock throttle to 1.
set pitch_ang to 0.
set compass to AzimuthCalc(targetIncl).
lock steering to lookdirup(heading(compass,90-pitch_ang):vector,ship:facing:upvector).
wait 1.

//FLIGHT MODE PARAMETERS
set throttleStage to 1.
set ascendStage to 1.

//STAGING
local current_max to maxthrust.
when maxthrust < current_max OR availablethrust = 0 then {
	set prevThrottle to throttle.
	lock throttle to 0.
	stage.
	lock throttle to prevThrottle.
	set current_max to maxthrust.
	preserve.
}

//PITCH_CALC PARAMETERS
set Pitch_Data to lexicon().
Pitch_Data:ADD("Time",time:seconds).
Pitch_Data:ADD("Pitch",0).
Pitch_Data:ADD("Pitch_Final",85).
Pitch_Data:ADD("Vz",verticalspeed).
Pitch_Data:ADD("Alt_Final",0.7*ship:body:atm:height).

//DeltaV_Calc PARAMETERS
set DeltaV_Data to lexicon().
DeltaV_Data:ADD("Total",0).
DeltaV_Data:ADD("Gain",0).
DeltaV_Data:ADD("Time",time:seconds).
DeltaV_Data:ADD("Thrust_Accel",throttle*availablethrust/mass).
DeltaV_Data:ADD("Accel_Vec",throttle*ship:sensors:acc).

wait until (altitude > gravTurnAlt). // OR ship:velocity > gravTurnV).

local line is 1.
until ascendStage = 3 {
	// Run Mode Logic
	//throttleStage:      //ascendMode:
	//1=engine burning   //1=powered ascend; engine burning, apoapsis<targetOrbit
	//2=MECO             //2=flight on a ballistic trajectory in the atmosphere; MECO, apoapsis>=targetOrbit
	//                   //3=flight on a ballistic trajectory outside the atmosphere; MECO, apoapsis>=targetOrbit
	
	if apoapsis > targetOrbit AND altitude > ship:body:ATM:height {
		lock throttle to 0.
		set ascendStage to 3.
	}
	
	if apoapsis < targetOrbit AND throttleStage = 1 {
		lock throttle to Max((targetOrbit-apoapsis)/1000, 0.005).
	}
	else if apoapsis > targetOrbit AND throttleStage = 1 {
		lock throttle to 0.
		when 1 then {rcs off.}
		set throttleStage to 2. //MECO
		set ascendStage to 2.
	}
	else if apoapsis < targetOrbit AND throttleStage = 2 {
		set throttleStage to 1.
		set ascendStage to 1.
		lock throttle to Max((targetOrbit-apoapsis)/100, 0.005).
	}	
  
	set Pitch_Data to Pitch_Calc(Pitch_Data).
	set pitch_ang to Pitch_Data["Pitch"].
	set compass to AzimuthCalc(targetIncl).	
	
	// Variable Printout
	set line to 1.
	print "throttleStage = " + throttleStage + "   " at(0,line).
	set line to line + 1.
	print "ascendStage   = " + ascendStage + "   " at(0,line).
	set line to line + 2.
	print "pitch_ang     = " + round(pitch_ang,2) + "   " at(0,line).
	set line to line + 1.
	print "compass       = " + round(compass,2) + "   " at(0,line).
	set line to line + 1.
	print "gravTurnAlt   = " + round(gravTurnAlt) + "   " at(0,line).
	set line to line + 2.
	print "altitude      = " + round(altitude) + "   " at(0,line).
	set line to line + 1.
	print "apoapsis      = " + round(apoapsis) + "   " at(0,line).
	set line to line + 1.
	print "target apo    = " + targetOrbit + "   " at(0,line).
	set line to line + 1.
	print "orbit incl    = " + round(orbit:inclination,5) + "   " at(0,line).
	set line to line + 1.
	print "target incl   = " + targetIncl + "   " at(0,line).
	
	// Delta V Calculations
	set DeltaV_Data to DeltaV_Calc(DeltaV_Data).
	
	// Delta V Printout
	set line to line + 3.
	print "DeltaV_total  = " + round(DeltaV_Data["Total"]) + "   " at(0,line).
	set line to line + 1.
	print "DeltaV_gain   = " + round(DeltaV_Data["Gain"]) + "   " at(0,line).
	set line to line + 1.
	print "DeltaV_Losses = " + round(DeltaV_Data["Total"] - DeltaV_Data["Gain"]) + "   " at(0,line).
	set line to line + 1.
	print "DeltaV_Eff    = " + round(100*DeltaV_Data["Gain"]/DeltaV_Data["Total"]) + "%   " at(0,line).
	
	wait 0.001. //wait for the next physics tick
}

//===================== CIRCULARIZATION =====================

sas on.
unlock steering.
until ETA:apoapsis < 1 {
	clearscreen.
	print "ETA:apoapsis = " + round(ETA:apoapsis,1).
	wait 0.25.
}
sas off.
rcs on.
wait 1.

set stopburn to false.
set dIp to targetIncl-orbit:inclination.
set past_dVincl to 0.
lock throttle to 1.
until stopburn {
	clearscreen.
	set data to burndata.
	lock steering to data[0].
	//return list(vec, fi, inclVec, dA, Vh, Vz, Vorb, dVorb, dI, dVincl).
	//            0  , 1 , 2      , 3 , 4 , 5 , 6   , 7    , 8 , 9
	
	if (data[7]<0) {
		set stopburn to true.
	}
	else if (data[0]:mag < 50) {
		lock throttle to Max(data[0]:mag/100, 0.01).
	}
	
	if (past_dVincl > data[9]) AND (abs(data[8]) < 0.085) { //fine tuning starts when deltaInclination comes to 0.085 degrees
		print "1.".
		lock steering to data[0].
		set past_dVincl to data[9].
		set v1 to VECDRAW(V(0,0,0), data[2], RGB(255,0,0), "dI", 1.0, TRUE, 0.2, TRUE, TRUE).
		set v2 to VECDRAW(V(0,0,0), Heading(90-targetIncl,data[1]):vector*data[7], RGB(0,255,0), "dVorb", 1.0, TRUE, 0.2, TRUE, TRUE).
		set v3 to VECDRAW(V(0,0,0), data[0], RGB(0,0,255), "final", 1.0, TRUE, 0.2, TRUE, TRUE).
	}
	else if (past_dVincl < data[9]) AND (abs(data[8]) < 0.085) {
		print "2". 
		lock steering to data[0]-2*data[2].
		set past_dVincl to data[9].
		set v1 to VECDRAW(V(0,0,0), data[2], RGB(255,0,0), "dI", 1.0, TRUE, 0.2, TRUE, TRUE).
		set v2 to VECDRAW(V(0,0,0), Heading(90-targetIncl,data[1]):vector*data[7], RGB(0,255,0), "dVorb", 1.0, TRUE, 0.2, TRUE, TRUE).
		set v3 to VECDRAW(V(0,0,0), data[0]-2*data[2], RGB(0,0,255), "final", 1.0, TRUE, 0.2, TRUE, TRUE).
	}
	else {
		print "3".
		lock steering to data[0].
		set v1 to VECDRAW(V(0,0,0), data[2], RGB(255,0,0), "dI", 1.0, TRUE, 0.2, TRUE, TRUE).
		set v2 to VECDRAW(V(0,0,0), Heading(90-targetIncl,data[1]):vector*data[7], RGB(0,255,0), "dVorb", 1.0, TRUE, 0.2, TRUE, TRUE).
		set v3 to VECDRAW(V(0,0,0), data[0], RGB(0,0,255), "final", 1.0, TRUE, 0.2, TRUE, TRUE).

	}
	
	//print "Fi     =" + data[1].
	//print "dA     =" + data[3].
	//print "Vh     =" + data[4].
	//print "Vz     =" + data[5].
	//print "Vorb   =" + data[6].
	print "dVorb  =" + data[7].
	print "dI     =" + data[8].
	print "dVincl =" + data[9].
	
	wait 0.001.
	
	set v1:show to false.
	set v2:show to false.
	set v3:show to false.
}

lock throttle to 0.
sas on.
clearscreen.
print "Circularization complete".
orbitData.



//========================FUNCTIONS========================

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
	local v_speed1 to Pitch_Data["Vz"].
	local t_1 to Pitch_Data["Time"].
	local v_speed2 to verticalspeed.
	local t_2 to time:seconds.
	local dt to max(0.0001,t_2 - t_1).
	local v_accel to max(0.001,(v_speed2 - v_speed2)/dt).
	local alt_final is Pitch_Data["Alt_Final"].
	local alt_diff is alt_final - altitude.
	
	local a to .5*v_accel.
	local b to verticalspeed.
	local c to -alt_diff.
	
	local time_to_alt to ((-b) + sqrt(max(0,b^2 - 4*a*c)))/(2*a).
	local pitch_des to Pitch_Data["Pitch"].
	local pitch_final to Pitch_Data["Pitch_Final"].
	local pitch_rate to max(0,(pitch_final - pitch_des)/time_to_alt).
	
	local pitch_des to min(pitch_final,max(0,pitch_des + dt*pitch_rate)).
	
	set Pitch_Data["Pitch"] to pitch_des.
	set Pitch_Data["Time"] to t_2.
	set Pitch_Data["Vz"] to v_speed2.
	
	return Pitch_Data.
}

function AzimuthCalc {
//https://www.orbiterwiki.org/wiki/Launch_Azimuth

	parameter targetIncl.
	local beta_ang is arcsin(MAX(-1, MIN(1, cos(targetIncl)/cos(ship:latitude)))).
	if beta_ang < 0 {
		set beta_ang to 180-beta_ang.
	}
	
	local R is ship:body:radius + targetOrbit.
	local Vorb is sqrt(ship:body:Mu/R). //1 косм. скорость на текущей высоте
	
	local Veqrot is (2*constant:pi*ship:body:radius)/(ship:body:rotationperiod).
	
	local Vxrot is Vorb*sin(beta_ang)-Veqrot*cos(ship:latitude).
	local Vyrot is Vorb*cos(beta_ang).
	
	local beta_rot_ang is (arctan2(Vxrot,Vyrot)).
	return beta_rot_ang.
}

function burndata {
	set eng to englist.

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
	
	local dI is targetIncl-orbit:inclination.
	local dVincl is 2*Vorb*sin(abs(dI/2)). //delta V required to change orbit inclination
	local normalVec is vcrs(ship:velocity:orbit,-body:position). //normal vector
	local norm is normalVec/normalVec:mag. //normal unit vector
	if dI > 0 {set inclVec to dVincl*norm.}
	else {set inclVec to -dVincl*norm.}
	
	set vec to Heading(90-targetIncl, fi):vector*dVorb + inclVec.
	
	return list(vec, fi, inclVec, dA, Vh, Vz, Vorb, dVorb, dI, dVincl).
}
	
function englist {
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
	
	if (ActiveEng:length=0) {return list(0,0,false).}
	else {return list(eng_thrust, eng_isp/ActiveEng:length, true).}
}

function orbitData {
	print "".
	print "ORBIT PARAMETERS".
	print "================".
	print "Apoapsis: " + ORBIT:APOAPSIS.
	print "Periapsis: " + ORBIT:PERIAPSIS.
	print "Eccentricity: " + ORBIT:ECCENTRICITY.
	print "Inclination: " + ORBIT:INCLINATION.
	print "================".
}
