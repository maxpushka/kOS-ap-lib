function ApChange {
    runoncepath("0:/kOS_ap_lib/Lib/lib_phys/VisVivaCalc.ks").
    runoncepath("0:/kOS_ap_lib/Lib/lib_phys/EngThrustIsp.ks").
    runoncepath("0:/kOS_ap_lib/Lib/lib_phys/BurnTime.ks").
	
    parameter targetAp, autowarp is false.
    
    local v0 is VisVivaCalc(apoapsis,ship:orbit:semimajoraxis).
    local v1 is VisVivaCalc(targetAp,(2*body:radius+periapsis+targetAp)/2).
    local dV is v0-v1.
    local t_burn is BurnTime(dv, periapsis).

	set apNode to node(time:seconds + ETA:periapsis, 0, 0, dV).
	add apNode.
	sas on.
	set sasmode to "maneuver".
	
	clearscreen.
    if autowarp = true {kuniverse:timewarp:warpto(time:seconds + ETA:periapsis - (t_burn/2+20)).}
    until (ETA:periapsis < t_burn/2+1) {
    	print "Burn time = " + t_burn + "   " at(0,0).
		print "Burn dV   = " + dV + "   " at(0,1).
        print "Burn starts in " + round(ETA:periapsis-(t_burn/2+1)) + " sec" + "   " at(0,2).
    }
    clearscreen.
	
    local stopburn is false.
	local burnVec is apNode:deltav.
	until stopburn {
		lock steering to burnVec.
		if apNode:deltav:mag < 10^(-5) {
			lock throttle to 0.
			set stopburn to true.
		}
		else {
			local eng is EngThrustIsp().
			local AThr is eng[0]/ship:mass.
			lock throttle to MIN(MAX(apNode:deltav:mag/AThr, 10^(-3)/eng[0]), 1).
		}
		
		print "dV: " + round(dv,1) + "   " at(0,0).
		print "Burn time: " + round(t_burn,1) + "   " at(0,1).
	}
	
	clearscreen.
	print "Target apo  = " + targetAp.
	print "Current apo = " + round(apoapsis, 1).
}
