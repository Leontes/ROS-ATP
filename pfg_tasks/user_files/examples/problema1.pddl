(define (problem Problema1)

	(:domain FACTORY)

	(:objects Piece1 - piece)

	(:INIT
		(robotPos storehouse)
		(ofType Piece1 typeA)
		(at workstation1 workshop1)
		(stored Piece1)
		(accepts workstation1 typeA)
		(transforms workstation1 typeB)
	)

	(:goal
		(work Piece1 typeB)
	)
)
 
