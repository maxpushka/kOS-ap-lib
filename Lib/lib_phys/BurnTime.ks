function BurnTime {
	parameter dV, altRad. //alt of place where burn occurs
	if (dV < 0) {set dV to abs(dV).}
	else if (altRad < 0) {return false.}
	
	local eng is EngThrustIsp().
	
	local f is eng[0].
	local m is ship:mass.
	local e is constant:e.
	local p is eng[1].
	local g is body:Mu/(body:radius+altRad)^2 * 9.81.
	
	return g * m * p * (1 - e^(-dV/(g*p))) / f.
}
