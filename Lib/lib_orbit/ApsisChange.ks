function ApChange {	
    parameter targetAp, dVtol is 10^(-2), AccelTol is 10^(-5), useRCS is false, autowarp is true.
	//[m/s] ammout of dV at which vessel would stop the burn; the lower, the more precise burn
	//[m/s^2] minimal vessel acceleration; the lower, the more precise burn
	//[bool] useRCS for rotation
	//[bool] autowarp to burn
	
	if targetAp < periapsis {
		print "Error: new apoapsis can not be lower than current periapsis".
		return false.
	}
	
	set ship:control:neutralize to true. //block user control inputs
	set ship:control:pilotmainthrottle to 0. //block user throttle inputs
	
    local v0 is ship:velocity:orbit:mag.
	local targetSMA is (2*body:radius+periapsis+targetAp)/2.
    local v1 is VisVivaCalc(periapsis, targetSMA).
    local dV is v1-v0.
	local t_burn is BurnTime(dV, periapsis).
	
	set burnNode to node(time:seconds+ETA:periapsis-(t_burn/2),0,0,dV).
	add burnNode.
	
	clearscreen.
    if autowarp = true {kuniverse:timewarp:warpto(time:seconds + ETA:periapsis - (t_burn/2+30)).}
	when (ETA:periapsis < t_burn/2+30) then {
		sas off.
		if useRCS {rcs on.}
		lock steering to prograde:vector:normalized*dV.
	}
	local burn is time:seconds+ETA:periapsis-ceiling(t_burn/2).
    until (time:seconds >= burn) {
		local v0 is ship:velocity:orbit:mag.
		local dV is v1-v0.
		set burnNode to node(burn,0,0,dV).
		
    	print "Burn time = " + BurnTime(dV, periapsis) + "   " at(0,0).
		print "Burn dV   = " + dV + "   " at(0,1).
        print "Burn starts in " + round(burn-time:seconds) + " sec" + "   " at(0,2).
    }
    clearscreen.
	
    local stopburn is false.
	until stopburn {
		local v0 is ship:velocity:orbit:mag.
		local targetSMA is (2*body:radius+periapsis+targetAp)/2.
		local v1 is VisVivaCalc(altitude, targetSMA).
		local dV is v1-v0.
		local t_burn is BurnTime(dV, altitude).
		
		set burnNode to node(0,0,0,dV).
		
		lock steering to prograde:vector:normalized*dV.
		if (abs(dV) < dVtol) {
			lock throttle to 0.
			unlock steering.
			sas on.
			rcs off.
			set stopburn to true.
		}
		else {
			local eng is EngThrustIsp().
			local AThr is eng[0]/ship:mass.
			lock throttle to MIN(MAX(abs(dV)/AThr, AccelTol*AThr), 1).
		}
		
		print "Throttle: " + round(throttle*100,5) + " %" + "     " at(0,0).
		print "dV: " + round(dV,5) + "     " at(0,1).
		print "Burn time: " + round(t_burn,5) + " sec" + "     " at(0,2).
		
		wait 0.
	}
	
	wait 1.
	if allnodes:length > 0 {remove allnodes[0].}
	clearscreen.
	print "Target apoapsis  = " + targetAp.
	print "Current apoapsis = " + round(apoapsis, 1).
}

function PeChange {	
    parameter targetPe, dVtol is 10^(-2), AccelTol is 10^(-5), useRCS is false, autowarp is true.
	//[m/s] ammout of dV at which vessel would stop the burn; the lower, the more precise burn
	//[m/s^2] minimal vessel acceleration; the lower, the more precise burn
	//[bool] useRCS for rotation
	//[bool] autowarp to burn
    
	if targetPe > apoapsis {
		print "Error: new periapsis can not be higher than current apoapsis".
		return false.
	}
	
	set ship:control:neutralize to true. //block user control inputs
	set ship:control:pilotmainthrottle to 0. //block user throttle inputs
	
    local v0 is ship:velocity:orbit:mag.
	local targetSMA is (2*body:radius+apoapsis+targetPe)/2.
    local v1 is VisVivaCalc(apoapsis,targetSMA).
    local dV is v1-v0.
	local t_burn is BurnTime(dv, apoapsis).
	
	set burnNode to node(time:seconds+ETA:apoapsis-(t_burn/2),0,0,dV).
	add burnNode.
	
	clearscreen.
    if autowarp = true {kuniverse:timewarp:warpto(time:seconds + ETA:apoapsis - (t_burn/2+30)).}
	when (ETA:apoapsis < t_burn/2+30) then {
		sas off.
		if useRCS {rcs on.}
		lock steering to prograde:vector:normalized*dV.
	}
	local burn is time:seconds+ETA:apoapsis-ceiling(t_burn/2).
    until (time:seconds >= burn) {
    	local v0 is ship:velocity:orbit:mag.
		local dV is v1-v0.
		set burnNode to node(burn,0,0,dV).
		
    	print "Burn time = " + BurnTime(dV, apoapsis) + "   " at(0,0).
		print "Burn dV   = " + dV + "   " at(0,1).
        print "Burn starts in " + round(burn-time:seconds) + " sec" + "   " at(0,2).
    }
    clearscreen.
	
    local stopburn is false.
	until stopburn {
		local v0 is ship:velocity:orbit:mag.
		local targetSMA is (2*body:radius+apoapsis+targetPe)/2.
		local v1 is VisVivaCalc(altitude,targetSMA).
		local dV is v1-v0.
		local t_burn is BurnTime(dV, altitude).
		
		set burnNode to node(0,0,0,dV).
		
		lock steering to prograde:vector:normalized*dV.
		if (abs(dV) < dVtol) {
			lock throttle to 0.
			unlock steering.
			sas on.
			rcs off.
			set stopburn to true.
		}
		else {
			local eng is EngThrustIsp().
			local AThr is eng[0]/ship:mass.
			lock throttle to MIN(MAX(abs(dV)/AThr, AccelTol*AThr), 1).
		}
		
		print "Throttle: " + round(throttle*100,5) + " %" + "     " at(0,0).
		print "dV: " + round(dV,5) + "     " at(0,1).
		print "Burn time: " + round(t_burn,5) + " sec" + "     " at(0,2).
		
		wait 0.
	}
	
	wait 1.
	if allnodes:length > 0 {remove allnodes[0].}
	clearscreen.
	print "Target periapsis  = " + targetPe.
	print "Current periapsis = " + round(periapsis, 1).
}