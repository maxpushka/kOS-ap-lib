print "Ascend script v1.0".

//==================== LAUNCH PARAMETERS ===================//
set targetOrbit to 100000. //target orbit in meters, Apoapsis=Periapsis=targetOrbit
set targetIncl to 0. //final orbit inclination in degrees

set gravityTurnAlt to 250. //altitude at which vessel shall start gravity turn

//========================= FLIGHT =========================//

set ship:control:pilotmainthrottle to 0.

set throttleStage to 0.
set ascendStage to 0.

set Pitch_Data to lexicon().
Pitch_Data:ADD("Time",time:seconds).
Pitch_Data:ADD("Pitch",0).
Pitch_Data:ADD("Pitch_Final",85).
Pitch_Data:ADD("Vz",verticalspeed).
Pitch_Data:ADD("Alt_Final",0.7*ship:body:atm:height).

clearscreen.
AG1 on. //open terminal
lock throttle to 1.
lock steering to heading(90,90).

local current_max to maxthrust.
when maxthrust < current_max OR availablethrust = 0 then {
	lock throttle to 0.
	stage.
	lock throttle to 1.
	set current_max to maxthrust.
	preserve.
}




//========================FUNCTIONS========================

function DeltaV_calc {
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

function Pitch_Rate {
	parameter Pitch_Data.
	local v_speed1 to Pitch_Data["V_Speed"].
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
	set Pitch_Data["V_Speed"] to v_speed2.
	
	return Pitch_Data.
}

function AzimuthCalc {
//https://www.orbiterwiki.org/wiki/Launch_Azimuth

	local beta_ang is arcsin(cos(targetIncl)/cos(ship:latitude)). //targetIncl is set in launch parameters.
	
	local R is ship:body:radius + targetOrbit.
	local Vorb is sqrt(ship:body:Mu/R). //1 косм. скорость на текущей высоте
	
	local Veqrot is (2*constant:pi*ship:body:radius)/(ship:body:rotationperiod).
	
	local Vxrot is Vorb*sin(beta_ang)-Veqrot*cos(ship:latitude).
	local Vyrot is Vorb*cos(beta_ang).
	
	local beta_rot_ang is (tan(Vxrot/Vyrot))^(-1).
	return beta_rot_ang.
}