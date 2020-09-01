function Circularize {
	parameter targetAlt.
	if (targetAlt < 0) OR ((NOT (apoapsis >= targetAlt)) AND (NOT (periapsis <= targetAlt))) {
  		return false.
	}

	SET SHIP:CONTROL:NEUTRALIZE TO TRUE. //block user control inputs
	set ship:control:pilotmainthrottle to 0. //block user throttle inputs
	
	until (altitude < targetAlt+100) AND (altitude > targetAlt-100) {
		print "Current alt     = " + round(altitude,1) at(0,0).
		print "Burn start alt  = " + targetAlt at(0,1).
		wait 1.
	}
	
	clearscreen.
	sas off.
	rcs on.
	
	until (altitude < targetAlt+1) AND (altitude > targetAlt-1) {
		set v1 to ship:velocity:orbit.
		set v2 to VXCL(ship:up:vector, ship:velocity:orbit):normalized*sqrt(body:Mu/(ship:body:radius+targetAlt)).
		set v3 to v2-v1.
		print "Correction dV = " + round(v3:mag,5) at(0,0).
		lock steering to v3.
		SET corr_vec TO VECDRAW(V(0,0,0), v3, RGB(255,0,0), round(v3:mag,1), 1.0, TRUE, 0.2, TRUE, TRUE).
		wait 0. //wait for the next physics tick
	}
	
	set stopburn to false.
	until stopburn {
		set v1 to ship:velocity:orbit.
		set v2 to VXCL(ship:up:vector, ship:velocity:orbit):normalized*sqrt(body:Mu/(ship:body:radius+targetAlt)).
		set v3 to v2-v1.
		
		clearscreen.
		print "Correction dV = " + round(v3:mag,5) at(0,0).
		SET corr_vec TO VECDRAW(V(0,0,0), v3, RGB(255,0,0), round(v3:mag,1), 1.0, TRUE, 0.2, TRUE, TRUE).
		
		lock steering to v3.
		local eng is EngThrustIsp().
		local AThr is eng[0]/ship:mass.
		local AThrLim is ship:mass/eng[0].
		lock throttle to MIN(MAX( v3:mag/AThr, 0.1*AThrLim), 1).
		
		if (v3:mag < 0.01) {
			lock throttle to 0.
			set stopburn to true.
		}
	}
	
	SET corr_vec:show to false.
	clearscreen.
	print "Circularization complete.".
	print "Apoapsis     = " + round(apoapsis, 1).
	print "Periapsis    = " + round(periapsis, 1).
	print "Apo-Per      = " + (round(apoapsis, 1) - round(periapsis, 1)).
	print "Eccentricity = " + round(orbit:eccentricity, 5).
	
	return true.
}
