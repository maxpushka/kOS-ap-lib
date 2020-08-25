function Land {
	parameter targetSite, touchdownSpeed is 1.0, hover_alt is 50.

	//============== LANDING PARAMETERS GUIDELINE ==============//
	
	// targetSite --> intended to be set using latlng(0,0) structure
	// touchdownSpeed --> [m/s]
	// hover_alt --> [m] altitude at which drone will hover before attempting final touchdown
	
	//======================== MAIN BODY =======================//
	
	//STAGING
	set current_max to maxthrust.
	when maxthrust < current_max OR availablethrust = 0 then {
		set prevThrottle to throttle.
		lock throttle to 0.
		stage.
		lock throttle to prevThrottle.
		set current_max to maxthrust.
		preserve.
	}
	
	//MAIN BODY
	clearscreen.
	set md to 1.
	local tmax is 0.9.
	local Isp_data is EngThrustIsp()[1].
	
	landing( targetSite, tmax, hover_alt, touchdownSpeed, Isp_data ).

	//======================= FUNCTIONS ========================//

	function nextmode {set md to md+1.}
	function landing {
		parameter landsite, ahmaxfrac, hover_alt, touchdownSpeed, Isp_data.
  
		local uf is V(0,1,0).
		local stoptime is 0.

		if (md = 1) {
			print "Mode 1" at (0,0).
			waitorient(landsite, 5). //outputs on string 1
			nextmode().
		}

		if (md = 2) {
			print "Mode 2" at (0,2).
			set tmax to min(ahmaxfrac, 3*mass*body:mu/(body:radius^2*availablethrust)).
			rotateorbit(landsite).
			nextmode().
		}

		if (md = 3) {
			print "Mode 3" at (0,3).
			set stoptime to waitdownrange(landsite, tmax).
			nextmode().
		}

		if (md = 4) {
			print "Mode 4" at (0,4).
			if (not gear) { gear on. }
			Descent(landsite, stoptime, hover_alt, Isp_data). //outputs on strings 5-11
			nextmode().
		}

		if (md = 5) {
			print "Mode 5" at (0,12).
			VertDescent(hover_alt, touchdownSpeed). //outputs on string 13
			nextmode().
		}
		set ship:control:pilotmainthrottle to 0.
		unlock throttle.
		unlock steering.
		print "Touchdown confirmed" at (0,14).
		wait 1.
		return true.
	}

	function warpfor {
		parameter dt.
		// warp    (0:1) (1:5) (2:10) (3:50) (4:100) (5:1000) (6:10000) (7:100000)
		set t1 to time:seconds+dt.
		// if (dt < 0) {
			// print "WARNING: wait time " + round(dt) + " is in the past.".
		// }
		until time:seconds >= t1 {
			local rt is t1-time:seconds.
			set warp to min(7,max(round(log10(min((rt*0.356)^2,rt*100))), 0)).
			wait 0.
		}
	}

	function GeoDist {
		parameter geocoord.
		parameter normvec is V(0,1,0).
		return abs( vdot(geocoord:altitudeposition(0), normvec:normalized) ).
	}

	function OrbAngTo {
		parameter pos.
		local nvec is vcrs( body:position, velocity:orbit ):normalized.
		local proj is vxcl(nvec, pos - body:position).
		local angl is arctan2( vdot(nvec, vcrs(body:position, proj)), -vdot(body:position, proj) ).
		if (angl < 0) {set angl to 360 + angl.}

		return angl.
	}

	function WaitOrient {
		parameter tgtcoord.
		parameter maxangle is arcsin(50/velocity:orbit:mag).

		if (abs(tgtcoord:lat) > orbit:inclination + maxangle or abs(tgtcoord:lat) > 180 - orbit:inclination + maxangle) {return 1/0.}
		local corrlng is tgtcoord:lng + OrbAngTo(tgtcoord:position)*orbit:period/body:rotationperiod.
		local corrtgt is latlng(tgtcoord:lat, corrlng).
		local nvec is vcrs( body:position, velocity:orbit ).
		until GeoDist(corrtgt, nvec) < body:radius * sin(maxangle) and OrbAngTo(corrtgt:position) > 90 {
			set warp to 4.
			wait 10.
			set corrlng to tgtcoord:lng + OrbAngTo(tgtcoord:position)*orbit:period/body:rotationperiod.
			set corrtgt to latlng(tgtcoord:lat, corrlng).
			set nvec to vcrs( body:position, velocity:orbit ).
			print "Angle to target projection: " + round(OrbAngTo(tgtcoord:position)) + "   " at (0,1).
		}
		set warp to 0.
	}

	function RotateOrbit {
		parameter tgtcoord.
		parameter newpe is 0.
		
		local dt is (OrbAngTo(burn_data()[1]:position) - 90)/360*orbit:period.
		if (dt < 0) {set dt to dt + orbit:period.}
		warpfor(dt).
		
		lock steering to lookdirup(burn_data()[0], up:vector).
		wait until vang(facing:vector, burn_data()[0]) < 1.
		until GeoDist(burn_data()[1], vcrs(body:position, velocity:orbit)) < 50 {
			local data is burn_data()[0].
			lock steering to lookdirup(data, up:vector).
			
			local eng is EngThrustIsp().
			local AThr is eng[0]/ship:mass.
			local AThrLim is ship:mass/eng[0].
			lock throttle to MIN(MAX((data:mag)/AThr, 0.1*AThrLim), 1).
		}
		lock throttle to 0.
		unlock steering.
		
		function burn_data {
			local corrlng is tgtcoord:lng + 90*orbit:period/body:rotationperiod.
			local corrtgt is latlng(tgtcoord:lat, corrlng).

			local newvdir is heading(corrtgt:heading, 0):vector.

			local newsma is body:radius + (newpe + altitude)*0.5.
			local newvmag is sqrt( body:mu * (2/body:position:mag - 1/newsma) ).
			local newv is newvmag*newvdir.
			local dV is newv - velocity:orbit.
			
			return list(dV, corrtgt).
		}
	}

	function waitdownrange {
		// assumed acceleration: maxah from 0 to ftfrac*th, change to 0 during last th
		parameter tgtcoord.
		parameter maxahfrac is 0.95.
		parameter ftfrac is min(4,availablethrust*body:radius^2/(mass*body:mu)).

		local maxah is maxahfrac*availablethrust/mass.
		local hland is tgtcoord:terrainheight.
		local lock vh to vxcl(up:vector,velocity:orbit - tgtcoord:altitudevelocity(hland):orbit):mag.
		local th is vh/(maxah*(ftfrac + 0.5)).

		lock steering to lookdirup(srfretrograde:vector,up:vector).
		until false {
			wait 0.
			set th to vh/(maxah*(ftfrac + 0.5)).
			local stopdist is maxah*th^2*0.5*(1/3 + ftfrac * (1+ftfrac) ).
			local tgtdist is (body:radius + (altitude - hland)/3)*vang(up:vector,tgtcoord:position - body:position)*constant:degtorad.
			if (tgtdist < stopdist) {break.}
			set kuniverse:timewarp:rate to (tgtdist - stopdist)/(vh*10).
		}
		return th.
	}

	function GetVA {
		// acceleration changes from a0 to a1 at tau, then to a2 at ts
		parameter hland, v0, geff.
		parameter ts, Isp_data.

		local dh is altitude - hland.

		local g1 is body:mu/(body:radius + hland + dh/3)^2.
		local dv is sqrt(velocity:surface:sqrmagnitude + 2*g1*dh ) + g1*ts/3.
		local endmass is mass*constant:e^(-dv/(Isp_data*9.80665)).
		local a2 is availablethrust/endmass - body:mu/(body:radius + hland)^2.

		local a0 is -geff.
		local tau is ( ts*(a2*ts - 2*v0) - 6*dh ) / ( 2*v0 + (a0 + a2)*ts ).
		local a1 is ( tau*(a2 - a0) - 2*v0 ) / ts - a2.

		print "ts = " + round(ts) + " ; tau = " + round(tau) + " ;" at (0,8).
		print "a0 = " + round(a0,2) + " ;" at(0,9).
		print "a1 = " + round(a1,2) + " ;" at(0,10).
		print "a2 = " + round(a2,2) at (0,11).
		return list(a1, a2, tau).
	}

	function Descent {
		parameter tgtcoord, th, hover_alt, Isp_data.
		parameter ftfrac is min(4,availablethrust*body:radius^2/(mass*body:mu)).

		local tburn is th*(1+ftfrac).
		local tend is time:seconds + tburn.
		local hland is tgtcoord:terrainheight + hover_alt.

		local vv0 is verticalspeed.
		local vh is vxcl(up:vector,velocity:orbit - tgtcoord:altitudevelocity(hland):orbit).
		local ah0 is vh:mag/(th * (ftfrac + 0.5)).

		local hpid is pidloop(0.04, 0, 0.4).
		local vpid is pidloop(0.04, 0, 0.4).
		set hpid:setpoint to 0.
		set vpid:setpoint to 0.
		set hpid:minoutput to -ah0/2.
		set hpid:maxoutput to ah0/2.
		set vpid:minoutput to -body:mu/(body:radius^2*2).

		local uf is facing:upvector.
		local av is 0.
		local expdr is ah0*th^2*0.5*(1/3 + ftfrac * (1+ftfrac) ).
		local expalt is altitude.

		local alt0 is expalt.
		local tleft is tburn.

		local dcenter is body:position:mag.
		local geff is (body:mu/dcenter^2 - vxcl(up:vector,velocity:orbit):sqrmagnitude)/dcenter.

		local a1a2tau is getva(hland, vv0, geff, tleft, Isp_data).
		local a0 is -geff.
		local a1 is a1a2tau[0].
		local a2 is a1a2tau[1].
		local tau is a1a2tau[2].

		until tleft <= 0 {
			local hdir is vxcl(up:vector,tgtcoord:position):normalized.
			set vh to vxcl(up:vector,velocity:orbit - tgtcoord:altitudevelocity(hland):orbit).
			set dcenter to body:position:mag.
			set geff to (body:mu/dcenter^2 - vxcl(up:vector,velocity:orbit):sqrmagnitude)/dcenter.
			local maxa is EngThrustIsp()[0] / mass.// * (body:mu/body:radius^2).

			set tleft to tend - time:seconds.
			local td is tburn - tleft.
			if (td < tau) {set av to a0 + (a1 - a0)*td/tau.}
			else {set av to a2 + (a1 - a2)*tleft/(tburn - tau).}
			local ah is ah0 * min(1, tleft/th).

			// expected downrange and altitude
			if (tleft > th) {set expdr to ah0*th^2/6 + ah0 * th/2*(tleft - th) + ah0*(tleft - th)^2/2.}
			else {set expdr to ah0*tleft^3/(6*th).}

			if (td < tau) {set expalt to alt0 + vv0*td + a0*td^2 / 2 + (a1 - a0)*td^3/(6*tau).}
			else {set expalt to hland + a2*tleft^2 / 2 + (a1 - a2)*tleft^3/(6*(tburn - tau)).}

			// real values
			local hasl is altitude.
			local dr is (body:radius + (hasl - hland)/3)*vang(up:vector, tgtcoord:position - body:position)*constant:degtorad.

			// side velocity component
			local latvel is vxcl(hdir, vh).
			if (vdot(hdir, vh)) < 0 {
				set dr to -dr.
				set latvel to V(0,0,0).
				set hdir to -hdir.
			}
			local alatvec is -latvel*5/max(tleft,1).
			local ahvec is -hdir*(ah + hpid:update(time:seconds, dr - expdr)).
			local avvec is up:vector*(av + geff + vpid:update(time:seconds, hasl - expalt)).

			print "Hacc: " + round(ahvec:mag,2) + "     " at(0,5).
			print "Vacc: " + round(avvec:mag,2) + "     " at (0,6).
			print "Lacc: " + round(alatvec:mag,2) + "     " at (0,7).
			print "Downrange current / predicted: " + round(dr/1000, 2) + " / " + round(expdr/1000, 2) + "      " at (0,8).
			print "Altitude current / predicted: " + round(hasl/1000, 2) + " / " + round(expalt/1000, 2) + "      " at (0,9).

			print "HPID output: " + round(hpid:output, 2) + "     " at (0,10).
			set avec to ahvec + avvec + alatvec.

			local dup is up:vector.
			if (dr <= 0 or vang(facing:vector, up:vector) <= 1) {set dup to uf.}
			lock steering to lookdirup(avec,dup).
			lock throttle to avec:mag/maxa.
			if (avec:mag/maxa > 1.005) { print " WARNING: not enough TWR" at (0,11). }
			else {print "                        " at (0,11).}
			if (dr > 0 and vang(facing:vector, up:vector) > 1) {set uf to facing:upvector.}
			if (alt:radar < 2*hover_alt and verticalspeed > -2) {break.}
			wait 0.
			
			if (dr < 25) {break.}
		}
		//return uf.
	}

	function VertDescent {
		parameter hover_alt. // сюда нужно передать высоту kOS модуля над поверхностью, когда корабль посажен
		parameter endspeed is 1.5. // желаемая скорость касания

		unlock steering.
		wait 0.
		local av is ship:verticalspeed^2 * 0.5 / max(alt:radar - hover_alt, 0.1).

		// выполняем цикл до мягкой посадки
		until status = "Landed" {
			local vh is vxcl(up:vector, velocity:surface).
			set vh to vh / max(1, vh:mag).
			lock steering to -velocity:surface.
			
			print "Vy: " + round(verticalspeed,2) + "     " at(0,13).
			if (verticalspeed < -abs(endspeed)) {
				set av to ship:verticalspeed^2 * 0.5 / max(alt:radar - hover_alt, 0.1).
			}
			else {set av to -abs(endspeed).}
			
			local dcenter is body:position:mag.
			local geff is (body:mu/dcenter^2 - vxcl(up:vector,velocity:orbit):sqrmagnitude)/dcenter.
			lock throttle to (mass * (av + geff) / (availablethrust * vdot(facing:vector, up:vector))).
			wait 0.
		}
		lock throttle to 0.
	}
}