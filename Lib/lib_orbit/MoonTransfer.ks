function MoonTransfer {
	parameter targetMoon, targetPe.
	
	//PARAMETERS CHECK
	local validBodies is list("Kerbol","Eve","Kerbin","Duna","Jool").
	if (targetPe <= 0) AND (NOT validBodies:contains(body:name)) {return false.}
	if round(orbit:inclination, 1) <> round(body(targetMoon):orbit:inclination, 1) OR
	round(orbit:lan, 1) <> round(body(targetMoon):orbit:lan, 1) {
		print "Error: target moon and the vessel must be in the same plane".
		return false.
	}
	
	//SETTING STAGING
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
	
	
	clearscreen.
	set stopburn to false.
	set targetAp to body("Mun"):altitude - body("Mun"):soiradius + pe.
	until stopburn {
		
		if apoapsis >= targetAp {set stopburn to true.}
		
		local eng is EngThrustIsp().
		local AThr is eng[0]/ship:mass.
		lock throttle to MIN(MAX( 1-(apoapsis/targetAp), 0.1*AThr), 1).
		//lock throttle to 1-(ship:orbit:apoapsis/body("Mun"):altitude).
		
		print "Target apoapsis  = " + round(targetAp,1) at(0,0).
		print "Current apoapsis = " + round(apoapsis,1) at(0,1).
		
		wait 0.
	}

	clearscreen.
	print "Target apoapsis  = " + round(targetAp,1).
	print "Current apoapsis = " + round(apoapsis,1).
	
	//===========================================
	
	function CalculateAngle {
		local an1 is (2*body:radius + ship:altitude + ( body("Mun"):altitude - body("Mun"):soiradius ) + pe)/2.
		local an2 is body:radius+body("Mun"):altitude.
		return 180*(1-(an1/an2)^1.5).
	}

	function CheckAngle {
		local vecS is ship:position - body:position.
		local vecM is body("Mun"):position - body:position.
		local vecHV is VXCL(ship:up:vector, ship:velocity:orbit).
		local vecSM is body("Mun"):position-ship:position.
		local mun_ang is CalculateAngle().
		local curr_ang is VANG(vecM, vecS).
		if VANG(vecHV, vecSM) > 90 {
			set curr_ang to -curr_ang.
		}
		
		print "Mun angle: " + mun_ang at(0,0).
		print "Current angle: " + curr_ang at(0,1).
		
		if abs(curr_ang - mun_ang) < 6 {
			set kuniverse:timewarp:rate to 0.
		}
		return abs(curr_ang - mun_ang) < 3.
	}
}