;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Robot y Bloques
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain BLOCKSANDROBOT2)
  (:requirements :strips :typing)
  (:types block room table robot)
  (:predicates (on ?x - block ?y - block)
	       (ontable ?x - block)
	       (clear ?x - block)
	       (holding ?x - block ?r - robot)
	       (conected ?x - room ?y -room)
	       (in ?x - table ?y - room)
	       (robotPos ?x - room ?r - robot)
	       (blockInTable ?x - block ?y-table)
  )

  (:action pick-up
	     :parameters (?x - block ?y - table ?z - room ?r - robot)
	     :precondition (and (clear ?x) (ontable ?x) 
			 (blockInTable ?x ?y) (in ?y ?z) (robotPos ?z ?r))
	     :effect
	     (and (not (ontable ?x))
		   (not (clear ?x))
		   (not (blockInTable ?x ?y))
		   (holding ?x ?r)
	     )
  )

  (:action put-down
	     :parameters (?x - block ?y - table ?z - room ?r -robot)
	     :precondition (and (holding ?x ?r) (in ?y ?z)(robotPos ?z ?r))
	     :effect
	     (and (not (holding ?x ?r))
		   (clear ?x)
		   (ontable ?x)
		   (blockInTable ?x ?y)
	     )
  )
 
 (:action stack
	     :parameters (?x - block ?y - block ?z - table ?u - room ?r - robot)
	     :precondition (and (holding ?x ?r) (clear ?y)
			(blockInTable ?y ?z) (in ?z ?u) (robotPos ?u ?r))
	     :effect
	     (and (not (holding ?x ?r))
		   (not (clear ?y))
		   (clear ?x)		
		   (on ?x ?y)
           (blockInTable ?x ?z)
	     )
 )
  (:action unstack
	     :parameters (?x - block ?y - block ?z - table ?u - room ?r - robot)
	     :precondition (and (on ?x ?y) (clear ?x) (blockInTable ?x ?z)
				(blockInTable ?y ?z)  (in ?z ?u) (robotPos ?u ?r) )
	     :effect
	     (and (holding ?x ?r)
		   (clear ?y)
		   (not (clear ?x))
		   (not (blockInTable ?x ?z))
		   (not (on ?x ?y))
	     )
  )
 
 (:action go
	     :parameters (?x - room ?y - room ?r - robot)
	     :precondition (and (robotPos ?x ?r) (conected ?x ?y))
	     :effect
	     (and (not(robotPos ?x ?r))
		  (robotPos ?y ?r)
	     )
 )
)
