function BurnTime {
	parameter dV, R. //alt of place where burn occurs
	if (dV < 0) {set dV to abs(dV).}
	else if (R < 0) {return false.}
	
	local f is EngThrustIsp().
	local m is ship:mass.
	local e is constant:e.
	local p is f[1].
	local g is body:Mu/(body:radius+R)^2.
	
	return g * m * p * (1 - e^(-dV/(g*p))) / f[0].
}
