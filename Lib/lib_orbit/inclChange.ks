function InclChange {
	parameter targetIncl. //[deg] target orbit inclination
	parameter tolerance is 10^(-5). //[deg] maximum deviation from target inclination.
	if targetIncl*tolerance < 0 {return false.} //check if both parameters > 0
	
	SET SHIP:CONTROL:NEUTRALIZE TO TRUE. //block user control inputs
	set ship:control:pilotmainthrottle to 0. //block user throttle inputs
	
	//ORIENTATE VESSEL
	sas off.
	rcs on.
	local normalVec is vcrs(ship:velocity:orbit,-body:position).
	if (ship:geoposition:lat < 0) {
		if (targetIncl-orbit:inclination > 0) {lock steering to normalVec.}
		else {lock steering to -normalVec.}
	}
	else {
		if (targetIncl-orbit:inclination < 0) {lock steering to normalVec.}
		else {lock steering to -normalVec.}
	}
	wait 1.
	
	//WAITING FOR ARRIVAL AT ASCENDING/DESCENDING NODE
	clearscreen.
	print "Estimated dV = " + InclData(targetIncl):mag at(0,0).
	set startburn to false.
	set prev_lat to 0.
	until startburn {
		set lat to ship:geoposition:lat.
		print "lat = " + lat at(0,1).
		print "prev_lat = " + prev_lat at(0,2).
		
		if (lat*prev_lat < 0) { //AND (lan_ang > lan) {
			set startburn to true.
		}
		
		wait 0.5.
		set prev_lat to lat.
	}
	clearscreen.
	
	//========================== BURN ==========================//
	sas off.
	rcs on.
	set stopburn to false.
	lock steering to InclData(targetIncl).
	//lock throttle to 1.
	until stopburn {
		local inclVec is InclData(targetIncl).
		lock steering to inclVec.
		
		if inclVec:mag < tolerance {
			lock throttle to 0.
			set stopburn to true.
			sas on.
			rcs off.
		}
		else {
			local eng is EngThrustIsp().
			local AThr is eng[0]/ship:mass.
			local AThrLim is ship:mass/eng[0].
			lock throttle to MIN(MAX(inclVec:mag/AThr, 0.1*AThrLim), 1).
		}
		print "dVincl = " + inclVec:mag at (0,1).
	}
	
	wait 1.
	clearscreen.
	print "Current inclination: " + round(ship:orbit:inclination,5).
}

function InclData {
	parameter targetIncl.
	
	set dI to targetIncl-orbit:inclination.
	set ecc to ship:orbit:eccentricity.
	set w to ship:orbit:argumentofperiapsis.
	set f to ship:orbit:trueanomaly.
	set n to (2*constant:pi)/ship:orbit:period.
	set a to ship:orbit:semimajoraxis.
	
	//delta V required to change orbit inclination
	set dVincl to (2*sin(dI/2)*sqrt(1-ecc^2)*cos(w+f)*n*a)/(1+ecc*cos(f)).
	
	set normalVec to vcrs(ship:velocity:orbit,-body:position):normalized. //normal unit vector
	set inclVec to normalVec*dVincl.
	
	return inclVec.
}