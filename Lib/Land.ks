function Land {
	parameter targetSite, touchdownSpeed is 0.5, hover_alt is 50.
	
	//============== LANDING PARAMETERS GUIDELINE ==============//
	
	// targetSite --> intended to be set using latlng(0,0) structure
	// touchdownSpeed --> [m/s]
	// hover_alt --> [m] altitude at which drone will hover before attempting final touchdown
	
	//======================== MAIN BODY =======================//
	
	clearscreen.
	set md to 1.	
	set Isp_data to EngThrustIsp()[1].
	print "Isp_data: " + Isp_data.
	landing( targetSite, 3.0, hover_alt, Isp_data ).
	
	//======================= FUNCTIONS ========================//
	
	function nextmode {set md to md+1.}
	function landing {
		parameter landsite, ahmax, hover_alt, Isp_data.
		
		local stoptime is 0.
		if md = 1 {
			print "Mode 1".
			waitorient(landsite).//, 5*constant:degtorad).
			nextmode().
		}
		if md = 2 {
			print "Mode 2".
			//until ship:availablethrust > 0 {stage.}
			rotateorbit(landsite).
			
			sas on.
			wait 1.
			set sasmode to "radialin".
			
			nextmode().
		}
		if md = 3 {
			print "Mode 3".
			set stoptime to waitdownrange(landsite, ahmax).
			nextmode().
		}
		if md = 4 {
			print "Mode 4".
			local av is getvertacc(landsite, stoptime, Isp_data).
			sas off.
			gear on.
			Descent(landsite, av).//Descent(landsite, stoptime, av).
			nextmode().
		}
		if md = 5 {
			print "Mode 5".
			VertDescent(hover_alt, touchdownSpeed).
			lock steering to "kill".
			wait 5.
			nextmode().
		}
		set ship:control:pilotmainthrottle to 0.
		unlock throttle.
		unlock steering.
	}
	
	function GeoDist {
		parameter geocoord.
		parameter normvec is V(0,1,0).
		return abs( vdot(geocoord:position, normvec:normalized) ).
	}
	
	function WaitOrient {
		parameter tgtcoord.
		parameter maxangle is arcsin(50/velocity:orbit:mag). // по умолчанию разрешаем коррекцию плоскости, если она не больше 50 м/с
		
		if abs(tgtcoord:lat) > orbit:inclination + maxangle or 
			abs(tgtcoord:lat) > 180 - orbit:inclination + maxangle {
				print "Error: latitude differs from inclination by more than maxangle, correction is impossible".
				return false.
		} // если широта отличается от наклонения больше, чем на maxangle, коррекция невозможна
		
		local lngcorr is OrbAngTo(tgtcoord:position)*orbit:period/body:rotationperiod. // поправка на вращение планеты
		local corrtgt is latlng(tgtcoord:lat, tgtcoord:lng + lngcorr).
		local nvec is vcrs( body:position, velocity:orbit ).
		until GeoDist(corrtgt, nvec) < body:radius * sin(maxangle) {
			set warp to 4. // 100x, если позволяет высота
			wait 10.
			set lngcorr to OrbAngTo(tgtcoord:position)*orbit:period/body:rotationperiod.
			set corrtgt to latlng(tgtcoord:lat, tgtcoord:lng + lngcorr).
			set nvec to vcrs( body:position, velocity:orbit ).
		}
		set warp to 0.
	}
	
	function OrbAngTo {
		//возвращает угол между орбитальным положением корабля и проекцией другого вектора на орбитальную плоскость
		parameter pos.
		
		local nvec is vcrs( body:position, velocity:orbit ):normalized.
		local proj is pos - body:position - vxcl(nvec, pos - body:position).
		local angl is arctan2( vdot(nvec, vcrs(body:position, proj)), -vdot(body:position, proj) ).
		if angl < 0 {set angl to 360 + angl.}

		return angl.
	}
	
	function RotateOrbit {
		// совмещает поворот орбиты со сбросом траектории на суборбитальную
		parameter tgtcoord.
		parameter newpe to 0.
		
		local corrlng is tgtcoord:lng + 90*orbit:period/body:rotationperiod.
		local corrtgt is latlng(tgtcoord:lat, corrlng). // направление на точку посадки с поправкой на вращение планеты
		
		warpfor((OrbAngTo(corrtgt:position) - 90)/360*orbit:period).
		local lngcorr is 90*orbit:period/body:rotationperiod.
		local corrtgt is latlng(tgtcoord:lat, tgtcoord:lng + lngcorr).
		local lock tgtdir to corrtgt:heading. // направление на точку посадки с поправкой на вращение планеты
		local lock newvdir to heading(tgtdir, 0):vector. // направление скорости, которое должно быть, чтобы орбита прошла над заданной точкой
		local lock newsma to body:radius + (newpe + altitude)*0.5.
		local lock newvmag to sqrt( body:mu * (2/(body:radius + altitude) - 1/newsma) ).
		local newv is newvmag*newvdir.
		local dV is newv - velocity:orbit.
		lock steering to lookdirup(dV, up:vector).
		wait until vang(facing:vector, dV) < 1.
		lock throttle to 1.
		until GeoDist(corrtgt, vcrs(body:position, velocity:orbit)) < 50 {
			set lngcorr to 90*orbit:period/body:rotationperiod.
			set corrtgt to latlng(tgtcoord:lat, tgtcoord:lng + lngcorr).
			lock tgtdir to corrtgt:heading. // направление на точку посадки с поправкой на вращение планеты
			lock newvdir to heading(tgtdir, 0):vector. // направление скорости, которое должно быть, чтобы орбита прошла над заданной точкой
			lock newsma to body:radius + (newpe + altitude)*0.5.
			lock newvmag to sqrt( body:mu * (2/(body:radius + altitude) - 1/newsma) ).
			set newv to newvmag*newvdir.
			set dV to newv - velocity:orbit.
			lock steering to lookdirup(dV, up:vector).
		}
		lock throttle to 0.
		unlock steering.
	}
	
	function waitdownrange {
		parameter tgtcoord.
		parameter maxah.
		local lock vh to ship:groundspeed. // groundspeed - скорость относительно поверхности
		local hland is tgtcoord:terrainheight.
		local stoptime is vh/maxah.
		until false {
			wait 0.
			set stoptime to vh/maxah.
			local stopdist is vh^2/(2*maxah).
			// расстояние по поверхности равно R_ave * alpha,
			// где alpha - угол в радианах между векторами
			// из центра притяжения на корабль и на цель,
			// R_ave - среднее расстояние от центра притяжения
			// до корабля за время снижения.
			// При равноускоренном снижении R_ave будет находиться
			// на 1/3 пути от поверхности до высоты аппарата
			local tgtdist is (body:radius + (altitude -hland)/3)*vang(up:vector,tgtcoord:position - body:position)*constant:degtorad.
			if tgtdist < stopdist + vh*0.2 {break.} // 0.2 секунды - время запуска двигателя в стоке
			set kuniverse:timewarp:rate to (tgtdist - stopdist)/(vh*5).
		}
		return stoptime.
	}
	
	function GetVertAcc {
		parameter tgtcoord.
		parameter stoptime, Isp_data. // обе величины в секундах, stoptime - время гашения горизонтальной скорости
		local hstart is altitude.
		local vv is ship:verticalspeed.
		local hland is tgtcoord:terrainheight.
		// оценка скорости, которую нужно стравить
		// складывается из непосредственно скорости и гравипотерь;
		// потери оценены как g*stoptime/5
		local dv is sqrt(velocity:surface:sqrmagnitude + 2*body:mu*(1/(body:radius + hland) - 1/(body:radius + hstart)) ) + body:mu/body:position:sqrmagnitude*stoptime/5.
		// оценка массы и максимально достижимого вертикального ускорения перед переходом к вертикальному спуску
		local endmass is mass*constant:e^(-dv/(Isp_data*9.80665)).
		local endav is availablethrust/endmass - body:mu/(body:radius + hland)^2.
		local a is stoptime^2/(2*endav).
		local b is stoptime * ( vv/endav - stoptime/2 ).
		local c is vv^2/(2*endav) - vv*stoptime - hstart + hland.
		return (-b - sqrt(b^2 - 4*a*c) ) / (2*a).
	}
		
	function Descent1 {
		parameter tgtcoord, stoptime, av.
		
		local tstart is time:seconds.
		local hland is tgtcoord:terrainheight.
		local vh0 is ship:groundspeed.
		local ah is vh0/stoptime.
		local endvv is ship:verticalspeed + av*stoptime.
		// высота перехода к вертикальному спуску
		local vdstarth is altitude + stoptime * (ship:verticalspeed + av * stoptime / 2).
		local hpid is pidloop(0.04, 0, 0.4).
		local vpid is pidloop(0.04, 0, 0.4).
		set hpid:setpoint to 0.
		set vpid:setpoint to 0.
		set hpid:minoutput to -ah/2.
		set hpid:maxoutput to ah/2.
		set vpid:minoutput to -body:mu/(body:radius^2*2).
		until altitude < vdstarth {
			local hdir is vxcl(up:vector,tgtcoord:position):normalized.
			local dcenter is body:radius + altitude.
			// ускорение свободного падения с поправкой на центробежную силу
			local geff is (body:mu/dcenter - vxcl(up:vector,velocity:orbit):sqrmagnitude)/dcenter.
			local maxa is ship:availablethrust / mass.
			local tleft is tstart + stoptime - time:seconds.

			// ожидаемые значения высоты и расстояния до цели
			local expdh is expvh^2/(2*ah).
			local expalt is vdstarth + endvv*(time:seconds - tstart - stoptime) + av/2*(stoptime - time:seconds + tstart)^2.
			//реальные значения
			local vh is ship:groundspeed.
			local vv is ship:verticalspeed.
			local hasl is altitude.
			local dh is (body:radius + (hasl - hland)/3)*vang(up:vector,tgtcoord:position-body:position)*constant:degtorad.
			set ahvec to -hdir*(ah + hpid:update(time:seconds,dh - expdh)).
			set avvec to up:vector*(av + geff + vpid:update(time:seconds,hasl - expalt)).
			// боковая скорость и настройка компенсации
			local latvel is vxcl(hdir,vxcl(up:vector,velocity:surface)).
			local alatvec is -latvel*5/max(tleft,1).
			print "Hacc: " + round(ahvec:mag,2) + "  Vacc: " + round(avvec:mag,2) + "  Lacc: " + round(alatvec:mag,2) + "     " at (0,13).
			set avec to ahvec + avvec + alatvec.
			lock steering to lookdirup(avec,up:vector).
			lock throttle to avec:mag/maxa.
			if avec:mag/maxa > 1.005 { print " WARNING: not enough TWR" at (25,33). }
			wait 0.
			if dh < 25 {break.} // за 25 м до точки посадки переходим к следующему этапу
		}
	}
	
	function Descent {
		parameter tgtcoord.
		parameter av.

		local tstart is time:seconds.
		local hland is tgtcoord:terrainheight.
		local vh is vxcl(up:vector, velocity:surface):mag.
		local vv is ship:verticalspeed.
		local dh0 is (body:radius + (altitude - hland)/3)*vang(up:vector,tgtcoord:position-body:position)*constant:degtorad.
		local ah is vh^2/(2*dh0).
		local stoptime is vh/ah.
		local endvv is ship:verticalspeed + av*stoptime.

		// высота перехода к вертикальному спуску
		local vdstarth is altitude + stoptime * (vv + av * stoptime / 2).

		local hpid is pidloop(0.04, 0, 0.4).
		local vpid is pidloop(0.04, 0, 0.4).

		set hpid:setpoint to 0.
		set vpid:setpoint to 0.
		set hpid:minoutput to -ah/2.
		set hpid:maxoutput to ah/2.
		set vpid:minoutput to -body:mu/(body:radius^2*2).

		until altitude <= vdstarth {
			local hdir is vxcl(up:vector,tgtcoord:position):normalized.
			local dcenter is body:radius + altitude.

			// ускорение свободного падения с поправкой на центробежную силу
			local geff is (body:mu/dcenter^2 - vxcl(up:vector,velocity:orbit):sqrmagnitude)/dcenter.
			local maxa is ship:availablethrust / mass.
			local tleft is max(0,tstart + stoptime - time:seconds).

			// ожидаемые значения высоты и расстояния до цели
			local expdh is ah*tleft^2/2.
			local expalt is vdstarth - endvv*tleft + av/2*tleft^2.

			//реальные значения
			set vh to vxcl(up:vector, velocity:surface):mag.
			set vv to ship:verticalspeed.
			local hasl is altitude.
			local dh is (body:radius + (hasl - hland)/3)*vang(up:vector,tgtcoord:position-body:position)*constant:degtorad.

			//необходимая горизонтальная и вертикальная тяга
			set ahvec to -hdir*max(0,ah + hpid:update(time:seconds,dh - expdh)).
			set avvec to up:vector*(av + geff + vpid:update(time:seconds,hasl - expalt)).

			// боковая скорость и настройка компенсации
			local latvel is vxcl(hdir,vxcl(up:vector,velocity:surface)).
			local alatvec is -latvel*5/max(tleft,1).

			print "Hacc: " + round(ahvec:mag,2) + "  Vacc: " + round(avvec:mag,2) + "  Lacc: " + round(alatvec:mag,2) + "     " at (0,13).
			print "Downrange current / predicted: " + round(dh/1000, 2) + " / " + round(expdh/1000, 2) + "      " at (0,14).
			print "Altitude current / predicted: " + round(hasl/1000, 2) + " / " + round(expalt/1000, 2) + "      " at (0,15).

			set avec to ahvec + avvec + alatvec.
			lock steering to lookdirup(avec,up:vector).
			lock throttle to avec:mag/maxa.
			if avec:mag/maxa > 1.005 { print " WARNING: not enough TWR" at (25,33). }
			if dh < 25 break. // за 25 м от точки посадки переходим к вертикальному снижению
			wait 0.
		}
	}
	
	function VertDescent {
		parameter hover_alt. // vessel hovers at this alt before attempting touchdown
		parameter touchdownSpeed.

		unlock steering.
		wait 0.
		local av is ship:verticalspeed^2 * 0.5 / max(alt:radar - hover_alt, 0.1).
		// выполняем цикл до мягкой посадки
		until (status = "Landed") {
			local vh is vxcl(up:vector, velocity:surface).
			set vh to vh / max(1, vh:mag).
			lock steering to -VELOCITY:SURFACE.
			if verticalspeed < -abs(touchdownSpeed) {
				set av to ship:verticalspeed^2 * 0.5 / max(alt:radar - hover_alt, 0.1).
			}
			else {set av to -abs(touchdownSpeed).}
			
			print "av = " + av at(0,8).
			local dcenter is body:position:mag.
			local geff is (body:mu/dcenter^2 - vxcl(up:vector,velocity:orbit):sqrmagnitude)/dcenter.
			lock throttle to mass * (av + geff) / (availablethrust * vdot(facing:vector, up:vector)).
			wait 0.
		}
		lock throttle to 0.
	}
	
	function warpfor {
		parameter dt.
		// warp    (0:1) (1:5) (2:10) (3:50) (4:100) (5:1000) (6:10000) (7:100000)
		set t1 to time:seconds+dt.
		if dt < 0 {
			print "WARNING: wait time " + round(dt) + " is in the past.".
			set t1 to t1 + orbit:period.
		}
		until time:seconds >= t1 {
			local rt is t1-time:seconds.
			set warp to min(7,max(round(log10(min((rt*0.356)^2,rt*100))), 0)).
			wait 0.
		}
	}
}