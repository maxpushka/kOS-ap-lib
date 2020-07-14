//Inclination changing script

function InclChange {
	parameter targetIncl. //[degrees] target orbit inclination
	parameter tolerance. //[degrees] maximum deviation from target inclination.
	if targetIncl*tolerance < 0 {return false.} //check if both parameters > 0
	
	runoncepath("0:/kOS_ap_lib/Lib/lib_phys/EngThrustIsp.ks").
	
	sas off.
	rcs on.
	clearscreen.
	
	set diff to ship:orbit:inclination/180.
	set steerlock to 3*diff.
	
	until (abs(ship:latitude) < diff) {
		print "burn at lat = 0" at(0,1).
		print "wait for lan " + diff at(0,2).
		print "current lat = " + ship:latitude at(0,3).
		if abs(ship:latitude) < steerlock {
			lock steering to InclData(targetIncl).}
	}
	until (ship:latitude > diff/180) {
		print "burn at lat = 0" at(0,1).
		print "wait for lan " + diff at(0,2).
		print "current lat = " + ship:latitude at(0,3).
	}
	clearscreen.
	
	sas off.
	rcs on.
	set stopburn to false.
	lock steering to InclData(targetIncl).
	lock throttle to 1.
	until stopburn {
		
		local inclVec is InclData(targetIncl).
		local AThr is EngThrustIsp()[0]/(ship:mass).
		
		lock steering to inclVec.
		
		if inclVec:mag < tolerance {
			lock throttle to 0.
			set stopburn to true.
			sas on.
			rcs off.
		}
		else {
			lock throttle to MIN(MAX(inclVec:mag/AThr, 10^(-5)), 1).
		}
		print "dVincl = " + inclVec:mag at (0,1).
	}
	
	wait 1.
	clearscreen.
	print "Current inclination: " + ship:orbit:inclination.
}

//==================== UTILITY FUNCTIONS ====================//

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
	
	set normalVec to vcrs(ship:velocity:orbit,-body:position):NORMALIZED. //normal unit vector
	set inclVec to dVincl*normalVec.
	
	return inclVec.
}