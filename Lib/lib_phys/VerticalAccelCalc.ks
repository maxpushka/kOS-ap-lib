function VerticalAccelCalc {
	local V is ship:velocity:orbit.
	local R is -ship:body:position.
	
	local temp_vec is VCRS(VCRS(R,V),R):normalized.
	local centri_acc to VDOT(temp_vec,V)^2/R:mag.
	
	local g is ship:body:mu/(ship:body:position:mag^2).
	return g - centri_acc.
}