function ApChange {	
    parameter targetAp, useRCS is false, autowarp is true.
    //it's highly recommended not to use RCS during the precise burns 
	//when difference between apo/pe is very small
	
	if targetAp < periapsis {
		print "Error: new apoapsis can not be lower than periapsis".
		return false.
	}
	
    local v0 is VisVivaCalc(apoapsis,ship:orbit:semimajoraxis).
	local targetSMA is (2*body:radius+periapsis+targetAp)/2.
    local v1 is VisVivaCalc(targetAp,(targetSMA+ship:orbit:semimajoraxis)/2).
    local dV is v0-v1.
	local t_burn is BurnTime(dv, periapsis).
	
	clearscreen.
    if autowarp = true {kuniverse:timewarp:warpto(time:seconds + ETA:periapsis - (t_burn/2+30)).}
	when (ETA:periapsis < t_burn/2+30) then {
		sas off.
		if useRCS {rcs on.}
		lock steering to prograde:vector:normalized*dV.
	}
    until (ETA:periapsis < t_burn/2+1) {
    	print "Burn time = " + t_burn + "   " at(0,0).
		print "Burn dV   = " + dV + "   " at(0,1).
        print "Burn starts in " + round(ETA:periapsis-(t_burn/2+1)) + " sec" + "   " at(0,2).
    }
    clearscreen.
	
    local stopburn is false.
	until stopburn {
		local v0 is VisVivaCalc(apoapsis,ship:orbit:semimajoraxis).
		local targetSMA is (2*body:radius+periapsis+targetAp)/2.
		local v1 is VisVivaCalc(targetAp,(targetSMA+ship:orbit:semimajoraxis)/2).
		local dV is v0-v1.
		
		lock steering to prograde:vector:normalized*dV..
		if abs(dV) < 10^(-5) {
			lock throttle to 0.
			set stopburn to true.
		}
		else {
			local eng is EngThrustIsp().
			local AThr is eng[0]/ship:mass.
			lock throttle to MIN(MAX(abs(dV)/AThr, 10^(-3)/eng[0]), 1).
		}
		
		print "dV: " + round(abs(dV),5) + "   " at(0,0).
		print "Burn time: " + round(t_burn,5) + "   " at(0,1).
		
		wait 0.
	}
	
	clearscreen.
	print "Target apoapsis  = " + targetAp.
	print "Current apoapsis = " + round(apoapsis, 1).
}

function PeChange {	
    parameter targetPe, useRCS is false, autowarp is true.
	//it's highly recommended not to use RCS when the difference 
	//between apo/pe is very small
    
	if targetPe > apoapsis {
		print "Error: new periapsis can not be higher than apoapsis".
		return false.
	}
	
    local v0 is VisVivaCalc(periapsis,ship:orbit:semimajoraxis).
	local targetSMA is (2*body:radius+apoapsis+targetPe)/2.
    local v1 is VisVivaCalc(targetPe,(targetSMA+ship:orbit:semimajoraxis)/2).
    local dV is v0-v1.
	local t_burn is BurnTime(dv, apoapsis).
	
	clearscreen.
    if autowarp = true {kuniverse:timewarp:warpto(time:seconds + ETA:apoapsis - (t_burn/2+30)).}
	when (ETA:apoapsis < t_burn/2+30) then {
		sas off.
		if useRCS {rcs on.}
		lock steering to prograde:vector:normalized*dV.
	}
    until (ETA:apoapsis < t_burn/2+1) {
    	print "Burn time = " + t_burn + "   " at(0,0).
		print "Burn dV   = " + dV + "   " at(0,1).
        print "Burn starts in " + round(ETA:apoapsis-(t_burn/2+1)) + " sec" + "   " at(0,2).
    }
    clearscreen.
	
    local stopburn is false.
	until stopburn {
		local v0 is VisVivaCalc(periapsis,ship:orbit:semimajoraxis).
		local targetSMA is (2*body:radius+apoapsis+targetPe)/2.
		local v1 is VisVivaCalc(targetPe,(targetSMA+ship:orbit:semimajoraxis)/2).
		local dV is v0-v1.
	
		lock steering to prograde:vector:normalized*dV.
		if abs(dV) < 10^(-5) {
			lock throttle to 0.
			set stopburn to true.
		}
		else {
			local eng is EngThrustIsp().
			local AThr is eng[0]/ship:mass.
			lock throttle to MIN(MAX(abs(dV)/AThr, 10^(-3)/eng[0]), 1).
		}
		
		print "dV: " + round(dV,5) + "   " at(0,0).
		print "Burn time: " + round(t_burn,5) + "   " at(0,1).
		
		wait 0.
	}
	
	clearscreen.
	print "Target periapsis  = " + targetPe.
	print "Current periapsis = " + round(periapsis, 1).
}