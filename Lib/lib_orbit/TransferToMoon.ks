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
	local targetAp is body(targetMoon):altitude - targetPe.
	set stopburn to false.
	lock throttle to 1.
	wait until ship:orbit:hasnextpatch.
	until (ship:orbit:nextpatch:periapsis <= targetPe) {
		local eng is EngThrustIsp().
		local AThrLim is ship:mass/eng[0].
		lock throttle to MIN(MAX(1-targetPe/ship:orbit:nextpatch:periapsis, 0.1*AThrLim), 1).
	}
	
	lock throttle to 0.
	unlock steering.
	sas on.
	wait 1.
	clearscreen.
	
	//===================== INSERTION BURN ======================//
	
	if (insertionBurn) {
		if (autowarp = true) {kuniverse:timewarp:warpto(time:seconds + ETA:transition - 1).}
		wait until (ship:body = body(targetMoon)).
		wait 5.
		rcs on.
		HoffmanTransfer(targetPe, ETA:periapsis). //orbit insertion
		HoffmanTransfer(targetPe, ETA:apoapsis). //periapsis correction of moonar orbit
	}
	
	//======================== FUNCTIONS ========================//
	
	function CalculateAngle {
		local a1 is (2*body:radius + ship:altitude + body(targetMoon):altitude - targetPe)/2. //ship's target semimajor axis
		local a2 is body:radius+body(targetMoon):altitude. //moonar semimajor axis
		return 180*(1-(a1/a2)^1.5).
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
		
		print targetMoon + " angle: " + mun_ang + "     " at(0,0).
		print "Current angle: " + curr_ang + "     " at(0,1).
		print "Burn start angle: " + (mun_ang+8) + "     " at(0,2).
		
		if abs(curr_ang - mun_ang) < 10 {
			set kuniverse:timewarp:rate to 0.
		}
		return abs(curr_ang - mun_ang) < 8.
	}
}