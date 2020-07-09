function BisectionSolver {
	parameter ScoreFunction, StartPoint, EndPoint, Nmax is 1.
	
	set table to list().
	if StartPoint > EndPoint {
		local tmp is EndPoint.
		set EndPoint to StartPoint.
		set StartPoint to tmp.
	}
	set MidPoint to (StartPoint + EndPoint)/2.
	
	set Score1 to ScoreFunction:call(StartPoint).
	set Score2 to ScoreFunction:call(EndPoint).
	set Score3 to ScoreFunction:call(MidPoint).
	
	local n is 1.
	until n > Nmax {	
		if Score1*Score2 < 0 { //sign(StartPoint) != sign(EndPoint)
			//reduce range
			set MidPoint to (StartPoint + EndPoint)/2.
			set Score3 to ScoreFunction:call(MidPoint).
			if Score3 = 0 {break.}
			
			if Score1*Score3 > 0 { //sign(StartPoint) == sign(MidPoint)
				set StartPoint to MidPoint.
				set Score1 to Score3.
			}
			else { //sign(EndPoint) == sign(MidPoint)
				set EndPoint to MidPoint.
				set Score2 to Score3.
			}
		}
		else { //sign(StartPoint) == sign(EndPoint)
			//expand range
			if Score1 > 0 { 
				set StartPoint to EndPoint.
				set Score1 to Score2.
				
				set EndPoint to StartPoint + 1.5*StartPoint.
				set Score2 to ScoreFunction:call(EndPoint).
			}
			else {
				set EndPoint to StartPoint.
				set Score2 to Score1.
				
				set StartPoint to EndPoint + 1.5*EndPoint.
				set Score1 to ScoreFunction:call(StartPoint).
			}
		}
		set n to n+1.
	}
	
	return list(StartPoint, EndPoint, MidPoint).
}