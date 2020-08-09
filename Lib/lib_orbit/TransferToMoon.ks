function TransferToMoon {
	parameter targetMoon, targetPe, autowarp is true, insertionBurn is true.
	
	//PARAMETERS CHECK
	if (NOT body:orbitingchildren:contains(body(targetMoon))) {
		print "Error: target moon's name is incorrect or planet has no moon with such name.".
		return false.
	}
	else if (targetPe <= 0) {
		print "Error: target periapsis should be > 0.".
		return false.
	}
	else if (autowarp:typename <> "Boolean") OR (insertionBurn:typename <> "Boolean") {
		print "Error: autowarp and insertionBurn type should be Boolean.".
		return false.
	}
	// if round(orbit:inclination) <> round(targetMoon:orbit:inclination) OR
	// round(orbit:lan) <> round(targetMoon:orbit:lan) {
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
	
	set targetMoon to body(targetMoon).
	set angle_dt to makeDerivator_dt(0,1).
	
	if (altitude > targetMoon:altitude) {
		lock steering to retrograde.
	}
	else {
		lock steering to prograde.
	}
	
	wait until CheckAngle(angle_dt)[0].
	
	//====================== TRANSFER BURN ======================//
	
	clearscreen.
	local targetR is targetMoon:altitude - 1.25*targetMoon:soiradius.
	lock throttle to 1.
	print "Throttle = " + round(throttle*100,5) + + " %" + "       " at (0,0).
	wait until (apoapsis > targetR).
	
	lock throttle to 0.1.
	print "Throttle = " + round(throttle*100,5) + + " %" + "       " at (0,0).
	wait until ship:orbit:hasnextpatch.
	
	
	local frac is targetMoon:radius/targetPe.
	if (frac < 1) {set frac to frac*10^((""+frac):length()-2).}
	else if (frac > 1) AND (frac < 2) {set frac to frac*2.}
	
	local currPe is ship:orbit:nextpatch:periapsis.
	until (currPe <= targetPe) {
		local eng is EngThrustIsp().
		local AThrLim is ship:mass/eng[0].
		lock throttle to MIN(MAX(1-frac*targetPe/currPe, 0.1*AThrLim), 0.1).
		set currPe to ship:orbit:nextpatch:periapsis.
		
		print "Throttle = " + round(throttle*100,5) + + " %" + "       " at (0,0).
		print "current periapsis = " + round(ship:orbit:nextpatch:periapsis,1) + "     " at(0,1).
		print "target periapsis  = " + targetPe + "     " at(0,2).
	}
	
	lock throttle to 0.
	unlock steering.
	sas on.
	wait 1.
	clearscreen.
	
	//===================== INSERTION BURN ======================//
	
	if (insertionBurn) {
		if (autowarp = true) {kuniverse:timewarp:warpto(time:seconds + ETA:transition).}
		until (ship:body = targetMoon) {
			print "ETA " + targetMoon + " SOI: " + round(ETA:transition,1) + " sec" + "     " at(0,0).
		}
		clearscreen.
		print "Calculating insertion maneuver...".
		wait 5.
		rcs on.
		HohmannTransfer(targetPe, ETA:periapsis, autowarp). //orbit insertion
	}
	
	//======================== FUNCTIONS ========================//
	
	function CalculateAngle {
		//ship's target semimajor axis
		local a1 is (2*body:radius + ship:altitude + targetMoon:altitude - targetPe)/2.
		local a2 is body:radius+targetMoon:altitude. //moonar semimajor axis
		return 180*(1-(a1/a2)^1.5).
	}

	function CheckAngle {
		parameter angle_dt_derivator.
		
		local vecS is ship:position - body:position.
		local vecM is targetMoon:position - body:position.
		local curr_ang is VANG(vecM, vecS).
		local moon_ang is CalculateAngle().
		local vecHV is VXCL(ship:up:vector, ship:velocity:orbit).
		local vecSM is targetMoon:position-ship:position.
		
		if VANG(vecHV, vecSM) > 90 {
			set curr_ang to -curr_ang.
		}
		
		local targetSMA is (2*body:radius + ship:altitude + targetMoon:altitude - targetPe)/2.
		local v0 is ship:velocity:orbit:mag.
		local v1 is VisVivaCalc(altitude, targetSMA).
		local dV is v1-v0.
		local t_burn is BurnTime(dV, altitude).
		
		local angle_dt is angle_dt_derivator:call(curr_ang).
		set warpstop to 3+abs(angle_dt)*20.
		
		print targetMoon:name + " angle: " + moon_ang + "     " at(0,0).
		print "Ship angle: " + curr_ang + "     " at(0,1).
		print "Warp stop angle: " + (moon_ang + warpstop) at(0,2).
		print "Burn start angle: " + (moon_ang+3) + "     " at(0,3).
		
		
		print "v0 = " + v0 at(0,5).
		print "v1 = " + v1 at(0,6).
		print "dV = " + dV at(0,7).
		print "Burn time = " + t_burn at(0,8).
		
		print "angle_dt = " + angle_dt at(0,10).
		
		if (abs(curr_ang - moon_ang) < warpstop) {
			set kuniverse:timewarp:rate to 0.
		}
		// else {
			// set kuniverse:timewarp:rate to 0.
		// }
		return list(abs(curr_ang - moon_ang) < 3, curr_ang, moon_ang).
	}
}