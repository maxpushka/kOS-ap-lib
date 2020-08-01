function TransferToMoon {
	parameter targetMoon, targetPe, autowarp is true, insertionBurn is true.
	
	//PARAMETERS CHECK
	local validBodies is list("Kerbol","Eve","Kerbin","Duna","Jool").
	if (targetPe <= 0) AND (NOT validBodies:contains(body:name)) {return false.}
	// if round(orbit:inclination) <> round(body(targetMoon):orbit:inclination) OR
	// round(orbit:lan) <> round(body(targetMoon):orbit:lan) {
		// print "Error: target moon and the vessel must be in the same plane".
		// return false.
	// }
	
	//STAGING
	set current_max to maxthrust.
	when maxthrust < current_max OR availablethrust = 0 then {
		set prevThrottle to throttle.
		lock throttle to 0.
		stage.
		lock throttle to prevThrottle.
		set current_max to maxthrust.
		preserve.
	}
	
	clearscreen.
	sas off.
	rcs on.
	
	lock steering to prograde.
	wait until CheckAngle().
	
	//====================== TRANSFER BURN ======================//
	
	clearscreen.
	set stopburn to false.
	set burnAp to body(targetMoon):altitude - targetPe.
	until stopburn {
		
		if apoapsis >= burnAp {set stopburn to true.}
		
		local eng is EngThrustIsp().
		local AThr is eng[0]/ship:mass.
		lock throttle to MIN(MAX( 1-(apoapsis/burnAp), 0.1*AThr), 1).
		
		print "Target apoapsis  = " + round(burnAp,1) at(0,0).
		print "Current apoapsis = " + round(apoapsis,1) at(0,1).
		
		wait 0.
	}

	clearscreen.
	print "Target apoapsis  = " + round(burnAp,1).
	print "Current apoapsis = " + round(apoapsis,1).
	
	//===================== INSERTION BURN ======================//
	
	if (autowarp = true) {kuniverse:timewarp:warpto(time:seconds + ETA:transition - 3).}
	wait until (ship:body = body(targetMoon)).
	wait 1.
	ApChange(targetPe, 1).
	
	//======================== FUNCTIONS ========================//
	
	function CalculateAngle {
		local an1 is (2*body:radius + ship:altitude + body(targetMoon):altitude - targetPe)/2.
		local an2 is body:radius+body(targetMoon):altitude.
		return 180*(1-(an1/an2)^1.5).
	}

	function CheckAngle {
		local vecS is ship:position - body:position.
		local vecM is body(targetMoon):position - body:position.
		local vecHV is VXCL(ship:up:vector, ship:velocity:orbit).
		local vecSM is body(targetMoon):position-ship:position.
		local mun_ang is CalculateAngle().
		local curr_ang is VANG(vecM, vecS).
		if VANG(vecHV, vecSM) > 90 {
			set curr_ang to -curr_ang.
		}
		
		print targetMoon + " angle: " + mun_ang at(0,0).
		print "Current angle: " + curr_ang at(0,1).
		
		if abs(curr_ang - mun_ang) < 6 {
			set kuniverse:timewarp:rate to 0.
		}
		return abs(curr_ang - mun_ang) < 3.
	}
}