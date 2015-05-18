(define (problem Problema1)

	(:domain FACTORY)

	(:objects Piece1 Piece2 - piece)

	(:INIT
		(robotPos storehouse)
		(ofType Piece1 typeA)
		(ofType Piece2 typeA)
		(at workstation1 workshop1)
		(stored Piece1)
		(in Piece2 workstation1)
		(workstationOcupied workstation1)
		(accepts workstation1 typeA)
		(transforms workstation1 typeB)
	)

	(:goal
		(work Piece2 typeB)
		(work Piece1 typeB)
	)
)
 
