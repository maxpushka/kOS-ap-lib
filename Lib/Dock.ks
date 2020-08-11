function Dock {
	parameter targetShip, targetNode, selectedNode, safeDistance is minSafeDist().

	//==================== PARAMETERS CHECK ====================//	
	
	clearscreen.
	set targetShip to vessel(targetShip). //converting str to Vessel type object
	if (targetShip:typename <> "Vessel") {
		print "Error: the selected target is not a vessel.".
		return false.
	}
	else if (ship:body <> targetShip:body) {
		print "Error: the selected target must be in the same SOI.".
		return false.
	}
	
	//====================== MAIN SEQUENCE =====================//
	
	//STARTING PARAMETERS
	set ship:control:neutralize to true. //block user control inputs
	set ship:control:pilotmainthrottle to 0. //block user throttle inputs
	rcs on.
	sas off.
	set prev_control_part to ship:controlpart.
	set elem to targetShip:elements:length.
	
	set selectedNode to ship:partstagged(selectedNode)[0].
	set targetNode to targetShip:partstagged(targetNode)[0].
	
	//CHECKING IF THE NODES ARE VALID
	if (NOT ship:dockingports:contains(selectedNode)) {
		print "Error: the part selected as the ship's node is not docking port or the ship has no docking ports.".
		return false.
	}
	else if (NOT targetShip:dockingports:contains(targetNode)) {
		print "Error: the part selected as target node is not docking port or target ship has no docking ports.".
		return false.
	}	
	else if (targetNode:nodetype <> selectedNode:nodetype) {
		print "Error: the size of target node does not match with the ship's selected node.".
		return false.
	}
	
	//DATA PRINTOUT
	print "Safety sphere radius: " + safeDistance + " m".
	print " ". print " ". print " ". print " ". //leaving two lines free for data printout
	print "Log:".
	print "[MET " + round(missiontime) + "s]: " + "Sequence started".
	local placeholder is "          ".
	when 1 then {
		lock steering to lookdirup(-targetNode:facing:forevector, targetNode:facing:topvector).
		local v_relative is ship:velocity:orbit - targetNode:ship:velocity:orbit.
		local dist is (targetNode:nodeposition - ship:controlpart:position):mag.
		
		print "Relative velocity   = " + round(v_relative:mag, 5) + " m/s" + placeholder at(0,2).
		print "Dist to target node = " + round(dist, 2) + " m" + placeholder at(0,3).
		preserve.
	}
	
	//LEAVING SAFERY SPHERE IF IN IT
	if targetNode:ship:position:mag < safedistance {
		print "[MET " + round(missiontime) + "s]: " + "Leaving safety sphere".
		
		local facing0 is ship:facing.
		lock steering to facing0.
		//Отходим от цели по прямой со скоростью 2 м/с
		local lock v_relative to ship:velocity:orbit - targetNode:ship:velocity:orbit.
		local lock tgtpos to targetNode:ship:position.
		until tgtpos:mag > safedistance {
			local tgtvel is -tgtpos:normalized * 2.
			translatevec(tgtvel - v_relative).
			wait 0.
		}
		unlock v_relative.
		unlock tgtpos.
	}
	
	kill_relative_velocity(targetNode).
	approach(targetNode, safeDistance).
	selectedNode:controlfrom.
	dock_finalize(targetNode, selectedNode).
	prev_control_part:controlfrom.
	
	return true.
	
	//======================= FUNCTIONS ========================//
	
	function minSafeDist { //intended to use only on docking init
		set box to vessel(targetShip):bounds.
		set absmin to box:relmin:mag.
		set absmax to box:relmax:mag.
		
		//max widespan of ship + 100 meters "keep-out sphere" radius
		return ceiling(max(absmin, absmax)+100).
	}
	
	function translatevec {
		parameter vector.
		local vec_copy is vector.
		//нормируем вектор, чтобы от него осталось только направление смещения
		if vector:sqrmagnitude > 1 {
			set vec_copy to vector:normalized.
		}
		local facing is ship:facing.
		local tf is vdot(vec_copy, facing:vector).
		local ts is vdot(vec_copy, facing:starvector).
		local tt is vdot(vec_copy, facing:topvector).
		set ship:control:translation to V(ts,tt,tf).
	}
	
	function kill_relative_velocity {
		parameter targetNode, threshold is 0.1.
		// threshold - относительная скорость, по достижении которой считаем цель неподвижной 
		// (по умолчанию ставим на 0.1 м/с)
		print "[MET " + round(missiontime) + "s]: " + "Killing relative velocity".
		local v_relative is ship:velocity:orbit - targetNode:ship:velocity:orbit.
		until v_relative:sqrmagnitude < threshold^2 {
			translatevec(-v_relative).
			wait 0.
			set v_relative to ship:velocity:orbit - targetNode:ship:velocity:orbit.
		}
		set ship:control:translation to V(0,0,0).
	}

	function v_safe {
		parameter dist.
		return sqrt(dist) / 2. // даёт 5 м/с на расстоянии 100 метров - вроде разумно
	}

	function move {
		parameter origin. // объект, относительно которого задаётся положение
		parameter pos. // где нужно оказаться относительно положения origin
		parameter vel is default_vel@. // с какой скоростью двигаться
		parameter tol is 0.5 * vel:call(). // в какой окрестности "засчитывается" попадание
		
		local lock v_wanted to vel:call() * (origin:position + pos:call()):normalized.
		local lock v_relative to ship:velocity:orbit - origin:velocity:orbit.
		local elem is origin:elements:length.
		// "попали" тогда, когда ship:position - origin:position = pos
		// ship:position = pos + origin:position
		// а ship:position всегда равно V(0,0,0)
		until ((origin:position + pos:call()):mag < tol) OR (origin:elements:length > elem) {	
			local lock v_wanted to vel:call() * (origin:position + pos:call()):normalized.
			local lock v_relative to ship:velocity:orbit - origin:velocity:orbit.
			translatevec(v_wanted - v_relative).
		}
		unlock v_wanted.
		unlock v_relative.
		
		function default_vel {
			return v_safe(origin:position:mag-safeDistance). // +10).
		}
	}
	
	function approach {
		parameter targetNode, rsafe.
		local lock vec_i to targetNode:facing:vector.
		local lock vec_j to vxcl(vec_i, -targetNode:ship:position):normalized.
		
		// фаза I
		if vxcl(vec_i, -targetNode:ship:position):mag < rsafe {
			print "[MET " + round(missiontime) + "s]: " + "Going around the target".
			move(targetNode:ship, phase1@).
			kill_relative_velocity(targetNode).
		}
		// фаза II
		if vdot(-targetNode:ship:position, vec_i) < rsafe {
			print "[MET " + round(missiontime) + "s]: " + "Getting in front of target".
			move(targetNode:ship, phase2@).
			kill_relative_velocity(targetNode).
		}
		// фаза III
		// выравниваемся уже не по центру масс корабля-цели, а по оси стыковочного узла
		print "[MET " + round(missiontime) + "s]: " + "Getting in front of target docking port".
		move(targetNode:ship, phase3@).
		kill_relative_velocity(targetNode).
		print "[MET " + round(missiontime) + "s]: " + "Ready for final approach sequence".
		unlock vec_i.
		unlock vec_j.
		
		function phase1 {
			return vxcl(vec_j, -targetNode:ship:position) + vec_j*rsafe.
		}
		function phase2 {
			return (vec_i + vec_j)*rsafe.
		}
		function phase3 {
			return targetNode:position - targetNode:ship:position + vec_i*rsafe.
		}
	}
	
	function dock_finalize {
		parameter targetNode, selectedNode.
		
		kill_relative_velocity(targetNode).
		print "[MET " + round(missiontime) + "s]: " + "Starting final docking approach".
		local dist is targetNode:nodeposition:mag.
		
		when 1 then {
			local dist is (targetNode:nodeposition - ship:controlpart:position):mag.
			
			set selectedNode_draw to VECDRAW(selectedNode:nodeposition, selectedNode:facing:vector*5, RGB(0,255,0), "", 1, true, 0.2, true).
			set targetNode_draw to VECDRAW(targetNode:nodeposition, newposition():normalized*dist, RGB(255,0,0), round(dist,2)+" m", 1, true, 0.2, false).
			
			preserve.
		}
		
		until ((targetNode:nodeposition - ship:controlpart:position):mag < targetNode:acquirerange*1.25) 
		OR (targetShip:elements:length > elem) {			
			move(targetNode:ship, newposition@, dock_vel@, dist * 0.1).
			set dist to dist*0.9.
		}
		
		print "[MET " + round(missiontime) + "s]: " + "Docking complete".
		set ship:control:translation to V(0,0,0).
		unlock all.
		clearvecdraws().
		
		function dock_vel {
			return min(max(v_safe((targetNode:nodeposition - ship:controlpart:position):mag) / 5, 0.1), 2).
		}
		function newposition {
			// положение, в котором должен находиться центр 
			// масс активного аппарата относительно цетра масс 
			// цели, чтобы стыковочные узлы притянулись
			return targetNode:facing:vector * dist + targetNode:nodeposition - ship:controlpart:position - targetNode:ship:position.
		}
	}
}