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
	set angle_dt to makeDerivator_dt(0,1).
	wait until CheckAngle(angle_dt)[0].
	
	//====================== TRANSFER BURN ======================//
	
	clearscreen.
	lock throttle to 1.
	print "Throttle = " + round(throttle*100,5) + + " %" + "       " at (0,0).
	wait until (ship:orbit:hasnextpatch) AND (ship:orbit:nextpatch:body = body(targetMoon)).
	lock throttle to 0.
	wait 1.
	
	local targetAp is body(targetMoon):altitude - targetPe.
	local frac is body(targetMoon):radius/targetPe.
	local currPe is ship:orbit:nextpatch:periapsis.
	until (currPe <= targetPe) {
		local eng is EngThrustIsp().
		local AThrLim is ship:mass/eng[0].
		lock throttle to MIN(MAX(1-frac*targetPe/currPe, 0.1*AThrLim), 1).
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
		until (ship:body = body(targetMoon)) {
			print "ETA " + targetMoon + " SOI: " + round(ETA:transition,1) + " sec" + "     " at(0,0).
		}
		clearscreen.
		wait 2.
		rcs on.
		HoffmanTransfer(targetPe, ETA:periapsis, autowarp). //orbit insertion
	}
	
	//======================== FUNCTIONS ========================//
	
	function CalculateAngle {
		//ship's target semimajor axis
		local a1 is (2*body:radius + ship:altitude + body(targetMoon):altitude - targetPe)/2.
		
		//moonar semimajor axis
		local a2 is body:radius+body(targetMoon):altitude.

		return 180*(1-(a1/a2)^1.5).
	}

	function CheckAngle {
		parameter angle_dt_derivator.
		
		local vecS is ship:position - body:position.
		local vecM is body(targetMoon):position - body:position.
		local vecHV is VXCL(ship:up:vector, ship:velocity:orbit).
		local vecSM is body(targetMoon):position-ship:position.
		local moon_ang is CalculateAngle().
		local curr_ang is VANG(vecM, vecS).
		
		if VANG(vecHV, vecSM) > 90 {
			set curr_ang to -curr_ang.
		}
		
		local targetSMA is (2*body:radius + ship:altitude + body(targetMoon):altitude - targetPe)/2.
		local v0 is ship:velocity:orbit:mag.
		local v1 is VisVivaCalc(altitude, targetSMA).
		local dV is v1-v0.
		local t_burn is BurnTime(dV, altitude).
		
		local angle_dt is angle_dt_derivator:call(curr_ang).
		
		print targetMoon + " angle: " + moon_ang + "     " at(0,0).
		print "Current angle: " + curr_ang + "     " at(0,1).
		print "Burn start angle: " + (moon_ang+1) + "     " at(0,2).
		
		print "v0 = " + v0 at(0,4).
		print "v1 = " + v1 at(0,5).
		print "dV = " + dV at(0,6).
		print "t_burn = " + t_burn at(0,7).
		
		print "angle_dt = " + angle_dt at(0,9).
		
		if abs(curr_ang - moon_ang) < angle_dt*60 { 
			set kuniverse:timewarp:rate to 0.
		}
		return list(abs(curr_ang - moon_ang) < 3, curr_ang, moon_ang).
	}
}