function Change_LAN_Inc {
	parameter DesiredOrbit. // lexicon("LAN", "INC")
	
	clearscreen.
	SET SHIP:CONTROL:NEUTRALIZE TO TRUE. //block user control inputs
	set ship:control:pilotmainthrottle to 0. //block user throttle inputs
	
	local body_pos to ship:body:position.
	local INC_ship to ship:orbit:inclination.
	local R to -body_pos.
	local SMA_ship to ship:orbit:semimajoraxis.
	local LAN_des to DesiredOrbit["LAN"].
	local LAN_VEC to  solarprimevector*(SMA_ship)*R(0,-LAN_des,0).
	local Inc_Rotate to ANGLEAXIS(-1*DesiredOrbit["INC"],LAN_VEC).
	local Inc_Normal to Inc_Rotate*(V(0,-1,0):direction).
	local Inc_Normal to SMA_ship*Inc_Normal:vector.

	local AngVel_ship to SMA_ship*VCRS(R,ship:velocity:orbit):normalized.

	local LAN_relative_vec to SMA_ship*VCRS(AngVel_ship,Inc_Normal):normalized.

	local LAN_relative_theta to FindTheta_Vec(LAN_relative_vec).
	local LAN_eta to ETA_to_theta(LAN_relative_theta).

	local delta_inc to VANG(AngVel_ship,Inc_Normal).
	local Vel_at_LAN to velocityat(ship,time:seconds + LAN_eta):orbit.
	local temp_dir to Vel_at_LAN:direction.
	local rotate_dir to ANGLEAXIS(delta_inc,LAN_relative_vec).
	local vel_rotated to rotate_dir*temp_dir.
	local New_Vel_at_LAN to (Vel_at_LAN:mag)*vel_rotated:vector.

	local LAN_node to SetNode_BurnVector(time:seconds + LAN_eta,New_Vel_at_LAN).
	add LAN_node.

	if (LAN_node:burnvector:mag > 0.05) {
		local Delta_V is LAN_node:deltav:mag.
		local BurnTime is Delta_V*mass/EngThrustIsp()[0].
		local BurnTimestamp is time:seconds + LAN_node:eta - BurnTime/2.
		
		print "Aligning with Maneuver Node".
		until VANG(ship:facing:vector, LAN_node:burnvector) < 1 {
			lock steering to lookdirup(LAN_node:burnvector,ship:facing:upvector).
			print "Direction Angle Error = " + round(VANG(ship:facing:vector,nextnode:burnvector),1) + "   "at(0,1).
		}
		print "Alignment Complete" at(0,2).
		
		clearscreen.
		print "Warping to Burn Point".
		kuniverse:timewarp:warpto(BurnTimestamp - 10).
		until (time:seconds >= BurnTimestamp) {
			print "ETA burn: " + round(BurnTimestamp - time:seconds) + " sec" + "        " at(0,1).
			lock steering to lookdirup(LAN_node:burnvector,ship:facing:upvector).
		}
		clearscreen.
		until (LAN_node:burnvector:mag < 0.05) {
			print "dV: " + round(LAN_node:burnvector:mag, 2) + "     " at(0,0).
			print "Throttle: " + round(throttle*100) + "%     " at(0,1).
			
			lock steering to lookdirup(LAN_node:burnvector,ship:facing:upvector).
			local eng is EngThrustIsp()[0].
			local AThr is eng/ship:mass.
			local AThrLim is ship:mass/eng.
			lock throttle to MIN(MAX(LAN_node:burnvector:mag/AThr, 0.001*AThrLim), 1).
		}				
	}
	lock throttle to 0.
	set ship:control:pilotmainthrottle to 0.
	clearscreen.
	remove LAN_node.
	wait 0.
	return true.
	
	function SetNode_BurnVector {
		parameter timeat,V_New.

		local V_timeat to velocityat(ship,timeat):orbit.

		local node_normal_vec to vcrs(ship:body:position,ship:velocity:orbit):normalized.
		local node_prograde_vec to V_timeat:normalized.
		local node_radial_vec to VCRS(node_normal_vec,node_prograde_vec).

		local burn_vector to (V_New - V_timeat).
		local burn_prograde to VDOT(node_prograde_vec,burn_vector).
		local burn_normal to VDOT(node_normal_vec,burn_vector).
		local burn_radial to VDOT(node_radial_vec,burn_vector).

		return NODE(timeat,burn_radial,burn_normal,burn_prograde).
	}
	
	function ETA_to_theta {
		parameter theta_test.

		local T_orbit to ship:orbit:period.
		local theta_ship to ship:orbit:trueanomaly.
		local e to ship:orbit:eccentricity.
		local GM to ship:body:mu.
		local a to ship:orbit:semimajoraxis.

		local EA_ship to 2*ARCTAN((TAN(theta_ship/2))/sqrt((1+e)/(1-e))).
		local MA_ship to EA_ship*constant:pi/180 - e*SIN(EA_ship).
		local EA_test to 2*ARCTAN((TAN(theta_test/2))/sqrt((1+e)/(1-e))).
		local MA_test to EA_test*constant:pi/180 - e*SIN(EA_test).
		local n to sqrt(GM/(a)^3).
		local eta_to_testpoint to (MA_test - MA_ship)/n.
		if eta_to_testpoint < 0 {
			set eta_to_testpoint to T_orbit + eta_to_testpoint.
		}

		return eta_to_testpoint.
	}
	
	function FindTheta_Vec {
		parameter test_vector is -ship:body:position.

		local body_pos to ship:body:position.
		local R to -body_pos.
		local AngVel_ship to VCRS(R,ship:velocity:orbit):normalized.
		local theta_test to VANG(test_vector,R).
		local cross_test to VCRS(R,test_vector):normalized.

		local check_vec to cross_test + AngVel_ship.
		local theta_ship is ship:orbit:trueanomaly.
		local theta is theta_ship.

		if check_vec:mag > 1 {set theta to theta_ship + theta_test.}
		else {set theta to theta_ship - theta_test.}

		if theta < 0 {set theta to 360 + theta.}
		if theta > 360 {set theta to theta - 360.}

		return theta.
	}
}