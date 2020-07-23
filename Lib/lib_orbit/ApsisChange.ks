function ApChange {
    runoncepath("0:/kOS_ap_lib/Lib/lib_phys/VisVivaCalc.ks").
    runoncepath("0:/kOS_ap_lib/Lib/lib_phys/EngThrustIsp.ks").
    runoncepath("0:/kOS_ap_lib/Lib/lib_phys/BurnTime.ks").
	
    parameter targetAp, autowarp is true.
    
    local v0 is VisVivaCalc(targetAp,ship:orbit:semimajoraxis).
    local v1 is VisVivaCalc(targetAp,ship:orbit:semimajoraxis+(targetAp-apoapsis)).
    local dV is v1-v0.
    
    local t_burn is BurnTime(dv, periapsis).
    
    clearscreen.
    sas on.

    if autowarp = true {kuniverse:timewarp:warpto(time:seconds + t_burn/2+15).}

    when ETA:periapsis < t_burn/2+10 then {
	sas off.
	rcs on.
	lock steering to prograde:vector:normalized*dV.
    }

    until ETA:periapsis < t_burn/2+1 {
    	print "Burn time = " + t_burn + "   " at(0,0).
	print "Burn dV   = " + dV + "   " at(0,1).
        print "Burn in " + round(ETA:periapsis) + " sec" + "   " at(0,2).
    }
    clearscreen.
    local stopburn is false.
    until stopburn {
        local v0 is VisVivaCalc(targetAp,ship:orbit:semimajoraxis).
        local v1 is VisVivaCalc(targetAp,ship:orbit:semimajoraxis+(targetAp-apoapsis)).
        local dV is v1-v0.
        
        lock steering to prograde:vector:normalized*dV.
        if abs(dV) < 1 {
            lock throttle to 0.
            set stopburn to true.
        }
        else {
            lock throttle to abs(dV)/EngThrustIsp()[0].
        }
		
		print "dV: " + round(dv,1) + "   " at(0,0).
		print "Burn time: " + round(t_burn,1) + "   " at(0,1).
    }
	
	print " Target apo: " + targetAp at(0,3).
	print "Current apo: " + apoapsis at(0,4).
}
