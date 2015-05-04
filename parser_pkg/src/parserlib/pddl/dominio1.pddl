;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Robot y Bloques
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain BLOCKSANDROBOT1)
  (:requirements :strips :typing)
  (:types block room table robot)
  (:predicates (on ?x - block ?y - block)
	       (ontable ?x - block)
	       (clear ?x - block)
	       (handempty ?r - robot)
	       (holding ?x - block ?r - robot)
	       (conected ?x - room ?y - room)
	       (in ?x - table ?y - room)
	       (robotPos ?x - room ?r - robot)
	       (blockInTable ?x - block ?y - table)
  )

  (:action pick-up
	     :parameters (?x - block ?y - table ?z - room ?r - robot)
	     :precondition (and (clear ?x) (ontable ?x) 
			(handempty ?r) (blockInTable ?x ?y) (in ?y ?z) (robotPos ?z ?r))
	     :effect
	     (and (not (ontable ?x))
		   (not (clear ?x))
		   (not (handempty ?r))
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
		   (handempty ?r)
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
		   (handempty ?r)
		   (on ?x ?y)
                   (blockInTable ?x ?z)
	     )
 )
  (:action unstack
	     :parameters (?x - block ?y - block ?z - table ?u - room ?r - robot)
	     :precondition (and (on ?x ?y) (clear ?x) (blockInTable ?x ?z)
				(blockInTable ?y ?z)  (in ?z ?u) (robotPos ?u ?r) (handempty ?r))
	     :effect
	     (and (holding ?x ?r)
		   (clear ?y)
		   (not (clear ?x))
		   (not (handempty ?r))
		   (not (blockInTable ?x ?z))
		   (not (on ?x ?y))
	     )
  )

  (:action tururu
	     :parameters (?x - block ?r - robot)
	     :precondition ((handempty ?r))
	     :effect
	     ((holding ?x ?r))
  )
 
 (:action go
	     :parameters (?x - room ?y - room ?r - robot)
	     :precondition (and (robotPos ?x ?r) (conected ?x ?y))
	     :effect
	     (and (not(robotPos ?x ?r))
		  (robotPos ?y ?r)
	     )
 )

 (:task move_robot
	:parameters (?rob - robot ?r - room)
	
  (:method Case1 ;;si el robot esta en la habitacion no se hace nada
	 :precondition (robotPos ?r ?rob)
	 :tasks ()
   )
	 
   
   (:method Case2 ;;si no esta en la habitacion destino
	  :precondition (robotPos ?r2 ?rob)
	  :tasks ( 
	  (go ?r2 r rob)
	  )
	)
)
)