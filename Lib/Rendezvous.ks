function Rendezvous {
	parameter targetShip, d_final is 1000, autowarp is true.
	set targetShip to vessel(targetShip). //converting str to Vessel type object
	
	clearscreen.
	set ship:control:neutralize to true. //block user control inputs
	set ship:control:pilotmainthrottle to 0. //block user throttle inputs
	if (targetShip:typename <> "Vessel") {
		print "Error: the selected target is not a vessel.".
		return false.
	}
	else if (ship:body <> targetShip:body) {
		print "Error: the selected target must be in the same SOI.".
		return false.
	}
	
	sas off.
	if (targetShip:orbit:semimajoraxis > ship:orbit:semimajoraxis) {
		lock steering to prograde.
	}
	else {lock steering to retrograde.}
	
	set placeholder to "                          ".
	set t_wait to 0.
	set prev_t_wait to 0.
	set TOF to 0.
	set t_wait_dt to makeDerivator_dt(0,1).
	
	when (t_wait > 0) AND (t_wait_dt:call(t_wait) < 0) then {
		set kuniverse:timewarp:rate to 0.
		if true {wait 1.}
		kuniverse:timewarp:warpto(time:seconds + t_wait - 60).
		unset t_wait_dt.
	}
	
	set startburn to false.
	until startburn {
		local pi is constant:pi.
		local w_target is sqrt(body:Mu/targetShip:orbit:semimajoraxis^3).
		local w_ship is sqrt(body:Mu/orbit:semimajoraxis^3).
		
		local a_tranfer is (targetShip:orbit:semimajoraxis+orbit:semimajoraxis)/2.
		set TOF to pi*sqrt(a_tranfer^3/body:Mu).
		local alpha_lead is constant:radtodeg*(w_target*TOF).
		local fi_final is 180-alpha_lead.
		
		local vecS is ship:position - body:position.
		local vecM is targetShip:position - body:position.
		local fi_init is VANG(vecM, vecS).
		
		set t_wait to constant:degtorad*(fi_final-fi_init)/(w_target-w_ship).
		
		if (t_wait < 0) AND (prev_t_wait > 0) {set startburn to true.}
		
		local line is 0.
		print "w_target = " + w_target at(0,line).
		local line is line+1.
		print "w_ship   = " + w_ship at(0,line).
		local line is line+1.
		print "tof = " + ceiling(tof) + placeholder at(0,line).
		local line is line+1.
		print "alpha_lead = " + alpha_lead at(0,line).
		local line is line+1.
		print "fi_init = " + fi_init at(0,line).
		local line is line+1.
		print "fi_final = " + fi_final at(0,line).
		local line is line+1.
		print "t_wait = " + ceiling(t_wait) + placeholder at(0,line).
		local line is line+1.
		print "prev_t_wait = " + prev_t_wait at(0,line).
		
		wait 1.
		set prev_t_wait to t_wait.
	}
	
	//===================== RENDEZVOUS BURN =====================//
	
	local t_intercept is time:seconds + TOF.
	local transferR is (positionat(targetShip, t_intercept)-body:position):mag - body:radius.
	HoffmanTransfer(transferR).
	
	//============= FLYING TO INTERSECTION POSITION =============//
	
	rcs off.
	unlock steering.
	sas on.
	
	set dV to (velocityat(targetShip, t_intercept):orbit - velocityat(ship, t_intercept):orbit):mag.
	set predR to (positionat(ship, t_intercept)-body:position):mag - body:radius.
	set t_burn to BurnTime(dV, predR).
	set t_intercept to t_intercept - (t_burn/2).
	kuniverse:timewarp:warpto(t_intercept - 60).
	
	clearscreen.
	print "CANCELLING OUT RELATIVE VELOCITY." at(0,0).
	until (time:seconds >= t_intercept) {
		set relativeVelocityVec to target:velocity:orbit - ship:velocity:orbit.
		lock steering to relativeVelocityVec.
		
		print "Burn in " + (t_intercept - time:seconds) + placeholder at(0,1).
		
		wait 0.
	}
	
	//============ CANCELLING OUT RELATIVE VELOCITY =============//
	
	sas off.
	rcs on.
	clearscreen.
	set stopburn to false.
	until stopburn {
		set relativeVelocityVec to target:velocity:orbit - ship:velocity:orbit.
		set relativeSpeed to relativeVelocityVec:mag.
		
		lock steering to relativeVelocityVec.
		print "Relative speed = " + round(relativeSpeed,2) at(0,0).
		print "Throttle = " + round(throttle*100,2) at(0,1).
		
		if (relativeSpeed < 0.05) {
			lock trottle to 0.
			set stopburn to true.
		}
		else if (relativeSpeed < 10) {
			local eng is EngThrustIsp().
			local AThr is eng[0]/ship:mass.
			local AThrLim is ship:mass/eng[0].
			lock throttle to MIN(MAX(abs(dV)/AThr, 0.001*AThrLim), 2*AThrLim). // [0.001; 2] m/s^2
		}
		else {
			local eng is EngThrustIsp().
			local AThr is eng[0]/ship:mass.
			local AThrLim is ship:mass/eng[0].
			lock throttle to MIN(MAX(abs(dV)/AThr, 0.001*AThrLim), 1).
		}
		
	}
	
	clearscreen.
	unlock steering.
	sas on.
	rcs off.
	print "Distance to target = " + (ship:position - targetShip:position):mag.
}