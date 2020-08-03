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
	set pastdV to 0.
	set stopburn to false.
	until stopburn {
		
		local v0 is ship:velocity:orbit:mag.
		local targetSMA is (2*body:radius + ship:altitude + body(targetMoon):altitude - targetPe)/2.
		local v1 is VisVivaCalc(altitude, targetSMA).
		local dV is v1-v0.
		local t_burn is BurnTime(dV, altitude).
		
		if ((pastdV > 0) AND (dv<0)) {
			lock throttle to 0.
			unlock steering.
			sas on.
			set stopburn to true.
		}
		else {
			local eng is EngThrustIsp().
			local AThr is eng[0]/ship:mass.
			local AThrLim is ship:mass/eng[0].
			lock throttle to MIN(MAX(abs(dV)/AThr, 0.1*AThrLim), 1).
		}
		
		print "dV: " + round(dV,5) + "     " at(0,0).
		print "Burn time: " + round(t_burn,5) + " sec" + "     " at(0,1).
		
		wait 0.
		set pastdV to dV.
	}
	
	wait 1.
	clearscreen.
	
	//===================== INSERTION BURN ======================//
	
	if (insertionBurn) {
		if (autowarp = true) {kuniverse:timewarp:warpto(time:seconds + ETA:transition - 3).}
		wait until (ship:body = body(targetMoon)).
		wait 5.
		rcs on.
		HoffmanTransfer(targetPe, ETA:periapsis). //orbit insertion
		wait 2.
		HoffmanTransfer(targetPe, ETA:periapsis). //apoapsis correction of moonar orbit
	}
	
	//======================== FUNCTIONS ========================//
	
	function CalculateAngle {
		local a1 is (2*body:radius + ship:altitude + body(targetMoon):altitude - targetPe)/2.
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
		print "Burn start angle: " + (mun_ang+3) + "     " at(0,2).
		
		if abs(curr_ang - mun_ang) < 6 {
			set kuniverse:timewarp:rate to 0.
		}
		return abs(curr_ang - mun_ang) < 3.
	}
}