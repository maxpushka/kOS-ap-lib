//==================== LIST OF FUNCTIONS ====================//

// InclChange(targetIncl)
// OrbitPatch(targetOrbit) //targetOrbit=Ap=Pe, incl is not changed

// EngThrustIsp()
// InclData(targetIncl)

//===================== IMPLEMENTATION =====================//

function InclChange {
	parameter targetIncl.
	
	until (abs(ship:latitude) < 0.5) {
		print "burn at lat = 0" at(0,1).
		print "current lat = " + round(ship:latitude, 3) at(0,2).
	}
	clearscreen.
	
	sas off.
	rcs on.
	set stopburn to false.
	lock steering to InclData(targetIncl).
	wait 5.
	until stopburn {
		
		local inclVec is InclData(targetIncl).
		set AThr to EngThrustIsp()[1]/(ship:mass).
		lock steering to inclVec.
		
		if inclVec:mag < 10^(-4) {
			lock throttle to 0.
			set stopburn to true.
			sas on.
			rcs off.
		}
		else if inclVec:mag < 50 AND inclVec:mag > 0.25 {
			lock throttle to MIN(MAX(inclVec:mag/AThr*1000, 0.05), 1).
		}
		else if inclVec:mag < 0.1 AND inclVec:mag > 0.05 {
			lock throttle to MIN(MAX(inclVec:mag/AThr*100, 0.01), 1).
		}
		else if inclVec:mag < 0.05 {
			lock throttle to MIN(MAX(inclVec:mag/AThr*10, 0.001), 1).
		}
		print "dVincl = " + round(inclVec:mag, 5) at (0,1).
	}
	
	wait 1.
	clearscreen.
	print "Current inclination: " + ship:orbit:inclination.
}

function OrbitPatch {
	parameter targetOrbit.
	
}

//==================== UTILITY FUNCTIONS ====================//

function EngThrustIsp {
	list engines in allEngines.
	
	set ActiveEng to list().
	ActiveEng:clear.
	
	set eng_isp to 0.
	set eng_thrust to 0.
	
	for eng in allEngines {
		if eng:ignition and (not eng:flameout) {
			ActiveEng:add(eng).
		}
	}
	
	for eng in ActiveEng {
		set eng_thrust to eng_thrust + eng:availablethrust.
		set eng_isp to eng_isp + eng:isp.
	}
	
	if (ActiveEng:length=0) {return list(0,0).}
	else {return list(eng_thrust, eng_isp/ActiveEng:length).}
}

function InclData {
	parameter targetIncl.
	
	set dI to targetIncl-orbit:inclination.
	set ecc to ship:orbit:eccentricity.
	set w to ship:orbit:argumentofperiapsis.
	set f to ship:orbit:trueanomaly.
	set n to 1/ship:orbit:period.
	set a to ship:orbit:semimajoraxis.
	
	//delta V required to change orbit inclination
	set dVincl to (2*sin(dI/2)*sqrt(1-ecc^2)*cos(w+f)*n*a)/(1+ecc*cos(f)).
	
	set normalVec to vcrs(ship:velocity:orbit,-body:position):NORMALIZED. //normal unit vector
	set inclVec to dVincl*normalVec.
	
	return inclVec.
}