function Dock {
	parameter targetShip, safeDistance is 25.
	set targetShip to vessel(targetShip). //converting str to Vessel type object
	
	clearscreen.
	set ship:control:neutralize to true. //block user control inputs
	set ship:control:pilotmainthrottle to 0. //block user throttle inputs
	if (targetShip:typename <> "Vessel") {
		print "Error: the selected target is not a vessel.".
		return false.
	}
	else if (ship:body <> targetShip:body) {
		print "Error: the selected target must be in the same SOI.".
		return false.
	}
	
	//==========================================================//
	
	rcs on.
	sas off.
	set tgtport to target:dockingports[0].
	lock v_relative to ship:velocity:orbit - tgtport:ship:velocity:orbit.

	if tgtport:ship:position:mag < safedistance {
		local facing0 to ship:facing.
		lock steering to facing0.
		// Отходим от цели по прямой со скоростью 2 м/с
		local lock v_relative to ship:velocity:orbit - tgtport:ship:velocity:orbit.
		local lock tgtpos to tgtport:ship:position.
		until tgtpos:mag > safedistance*1.5 {
			local tgtvel to -tgtpos:normalized * 2.
			translatevec(tgtvel - v_relative).
			wait 0.
		}
		unlock v_relative.
		unlock tgtpos.
		// не забудем затормозиться, когда отощли на безопасное расстояние
		kill_relative_velocity(tgtport).
	}
	
	unlock steering.
	wait 0.
	lock steering to lookdirup(-tgtport:facing:forevector, tgtport:facing:topvector).
	approach(tgtport, safeDistance).
	dock_finalize(tgtport).
	
	wait until ship:elements:length > 1.
	
	function translatevec {
		parameter vec.
		local vec_copy to vec.
		//нормируем вектор, чтобы от него осталось только направление смещения
		if vec:sqrmagnitude > 1 {
			set vec_copy to vec:normalized.
		}
		local facing to ship:facing.
		local tf to vdot(vec_copy, facing:vector).
		local ts to vdot(vec_copy, facing:starvector).
		local tt to vdot(vec_copy, facing:topvector).
		set ship:control:translation to V(ts,tt,tf).
	}
	
	function kill_relative_velocity {
		parameter tgtport, thr to 0.1.
		// thresh - относительная скорость, по достижении которой считаем цель неподвижной 
		// (по умолчанию ставим на 0.1 м/с)
		local v_relative to ship:velocity:orbit - tgtport:ship:velocity:orbit.
		until v_relative:sqrmagnitude < thr*thr {
			translatevec(-v_relative).
			wait 0.
			set v_relative to ship:velocity:orbit - tgtport:ship:velocity:orbit.
		}
	}

	function v_safe {
		parameter dist.
		return sqrt(dist) / 2. // даёт 5 м/с на расстоянии 100 метров - вроде разумно
	}

	function moveto {
		parameter origin. // объект, относительно которого задаётся положение
		parameter pos. // где нужно оказаться относительно положения origin
		parameter vel is v_safe(origin:position:mag). // с какой скоростью двигаться
		parameter tol to 0.5 * vel. // в какой окрестности "засчитывается" попадание
		local lock v_wanted to vel * (origin:position + pos):normalized.
		local lock v_relative to ship:velocity:orbit - origin:velocity:orbit.
		// "попали" тогда, когда ship:position - origin:position = pos
		// ship:position = pos + origin:position
		// а ship:position всегда равно V(0,0,0)
		until (origin:position + pos):mag < tol {
			set vel to v_safe(origin:position:mag).
			local lock v_wanted to vel * (origin:position + pos):normalized.
			local lock v_relative to ship:velocity:orbit - origin:velocity:orbit.
			translatevec(v_wanted - v_relative).
		}
		unlock v_wanted.
		unlock v_relative.
	}
	
	function approach {
		parameter tgtport, rsafe.
		local lock vec_i to tgtport:facing:vector.
		local lock vec_j to vxcl(vec_i, -tgtport:ship:position):normalized.
  
		// фаза I
		if vxcl(vec_i, -tgtport:ship:position):mag < rsafe {
			print "Going around the target".
			moveto(tgtport:ship, vxcl(vec_j, -tgtport:ship:position) + vec_j*rsafe).
		}
		// фаза II
		if vdot(-tgtport:ship:position, vec_i) < rsafe {
			print "Getting in front of target".
			moveto(tgtport:ship, (vec_i + vec_j)*rsafe).
		}
		// фаза III
		// выравниваемся уже не по центру масс корабля-цели, а по оси стыковочного узла
		print "Getting in front of target docking port".
		moveto(tgtport:ship, tgtport:position - tgtport:ship:position + vec_i*rsafe).
		print "Ready for final approach".
		unlock vec_i.
		unlock vec_j.
	}
	
	function dock_finalize {
		parameter tgtport.
		print "Starting final docking approach".
		local dist to tgtport:nodeposition:mag * 0.75.
		// положение, в котором должен находиться центр масс активного аппарата относительно цетра масс цели, чтобы стыковочные 
		local newposition to tgtport:facing:vector * dist + tgtport:nodeposition - ship:controlpart:position - tgtport:ship:position.
		until (tgtport:nodeposition - ship:controlpart:position):mag < tgtport:acquirerange*1.25 {
			local vel is min(max(v_safe((tgtport:nodeposition - ship:controlpart:position):mag) / 2, 0.25), 1).
			moveto(tgtport:ship, newposition, vel, dist * 0.25).
			set dist to dist*0.75.
			set newposition to tgtport:facing:vector * dist + tgtport:nodeposition - ship:controlpart:position - tgtport:ship:position.
		}
		unlock all.
	}
}