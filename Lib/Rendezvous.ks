function Rendezvous {
	parameter targetShip.
	set targetShip to vessel(targetShip). //converting str to Vessel type object
	
	clearscreen.
	if (targetShip:typename <> "Vessel") {
		print "Error: the selected target is not a vessel.".
		return false.
	}
	
	local fi is (orbit:trueanomaly - targetShip:orbit:trueanomaly)*constant:DegToRad.
	local e1 is orbit:eccentricity.
	local E is 2*arctan(sqrt( (1-e1)/(1+e1) )*tan(fi/2)).
	
	local t is (orbit:period/2*constant:pi)*(E-e1*sin(E)).
	
	print fi.
	print e1.
	print E.
	print t.
	
	local timestamp is time:seconds + t.
	kuniverse:timewarp:warpto(timestamp).
	
	
}