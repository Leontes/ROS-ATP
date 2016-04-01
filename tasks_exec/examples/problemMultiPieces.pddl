(define (problem Problema1)

	(:domain FACTORY)

	(:objects Piece1 Piece2 Piece3 - piece)

	(:INIT
		(robotPos storehouse)

		(ofType Piece1 typeA)
		(ofType Piece2 typeA)
		(ofType Piece3 typeB)

		(at workstation1 workshop1)
		(at workstation2 workshop2)

		(stored Piece1)
		(stored Piece3)

		(in Piece2 workstation1)

		(workstationOcupied workstation1)

		(accepts workstation1 typeA)
		(accepts workstation2 typeB)

		(transforms workstation1 typeB)
		(transforms workstation2 typeC)
	)

	(:goal
		(work Piece2 typeC)
		(work Piece1 typeB)
		(work Piece3 typeC)
	)
)
 
