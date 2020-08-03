function HoffmanTransfer {	
    parameter targetR, burnIn is 0, autowarp is true.
	//targetR = [m] radius of target orbit
	//burnIn = [s] execute burn in (time:seconds+burnIn) seconds
	//autowarp = [bool] autowarp to burn position
	
	clearscreen.
	set ship:control:neutralize to true. //block user control inputs
	set ship:control:pilotmainthrottle to 0. //block user throttle inputs
	
	local burnTimestamp is time:seconds+burnIn. //time in seconds when maneuver occurs
    local v0 is velocityat(ship, burnTimestamp):orbit:mag. //orbit velocity in burnIn seconds
	local predR is (positionat(ship, burnTimestamp)-body:position):mag - body:radius. //altitude in burnIn seconds
	local targetSMA is (2*body:radius+predR+targetR)/2.
    local v1 is VisVivaCalc(predR, targetSMA).
    local dV is v1-v0.
	
	local t_burn is BurnTime(dV, predR). //time of burn
	local burnTimestamp is burnTimestamp-ceiling(t_burn/2). //corrected time in seconds when maneuver occurs
	
	set burnNode to node(burnTimestamp,0,0,dV).
	add burnNode.
	
	sas off.
	
    if (autowarp = true) {kuniverse:timewarp:warpto(burnTimestamp-30).}
    until (time:seconds >= burnTimestamp) {
		set v0 to velocityat(ship, burnTimestamp):orbit:mag.
		set predR to (positionat(ship, burnTimestamp)-body:position):mag - body:radius.
		set targetSMA to (2*body:radius+predR+targetR)/2.
		set v1 to VisVivaCalc(predR, targetSMA).
		set dV to v1-v0.
		lock steering to prograde:vector:normalized*dV.
		
		print "Burn time = " + round(t_burn,5) + " sec" + "     " at(0,0).
		print "Burn dV   = " + round(dV,10) + "     " at(0,1).
        print "Burn starts in " + round(burnTimestamp-time:seconds) + " sec" + "     " at(0,2).
		wait 0.
    }
    clearscreen.
	
	set pastdV to 0.
    local stopburn is false.
	until stopburn {
		local v0 is ship:velocity:orbit:mag.
		local v1 is VisVivaCalc(altitude, targetSMA).
		local dV is v1-v0.
		local t_burn is BurnTime(dV, altitude).
		
		set burnNode to node(0,0,0,dV).
		
		lock steering to prograde:vector:normalized*dV.
		if ((pastdV < 0) AND (dv>0)) OR ((pastdV > 0) AND (dv<0)) {
			lock throttle to 0.
			unlock steering.
			sas on.
			rcs off.
			set stopburn to true.
		}
		else {
			local eng is EngThrustIsp().
			local AThr is eng[0]/ship:mass.
			local AThrLim is ship:mass/eng[0].
			lock throttle to MIN(MAX(abs(dV)/AThr, 0.001*AThrLim), 1).
		}
		
		print "Throttle: " + round(throttle*100,5) + " %" + "     " at(0,0).
		print "dV: " + round(dV,10) + "     " at(0,1).
		print "Burn time: " + round(t_burn,5) + " sec" + "     " at(0,2).
		
		wait 0.
		set pastdV to dV.
	}
	
	if (allnodes:length > 0) {remove allnodes[0].}
	clearscreen.
}
