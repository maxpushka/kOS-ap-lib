function VisVivaCalc {
    parameter r, a.
    
	set r to body:radius+r.
	
    if (r*a < 0) {return false.} //check if parameters both > 0
    else {return sqrt(ship:body:mu*(2/r - 1/a)).}
}