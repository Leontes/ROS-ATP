;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Factory
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain FACTORY)
  (:requirements :strips :typing :adl)
  (:types piece workstation place type robot)
  (:constants storehouse workshop1 workshop2 - place workstation1 workstation2 - workstation typeA typeB typeC - type)
  (:predicates 
	(robotPos ?place - place)
	(ofType ?piece - piece ?type - type )
	(at ?workstation - workstation ?workshop - place)
	(in ?piece - piece ?workstation - workstation)
	(stored ?piece - piece)
	(robotOcupied)
	(workstationOcupied ?workstation - workstation)
	(inRobot ?piece - piece)
	(accepts ?workstation - workstation ?type - type)
	(transforms ?workstation - workstation ?type - type)
  )

  (:action pick_up
	     :parameters (?piece - piece)
	     :precondition 
	     	(and 
	     		(not(robotOcupied)) 
	     		(stored ?piece)
	     		(robotPos storehouse)
	     	)
	     :effect
	     	(and
				(robotOcupied)
				(not(stored ?piece))
				(inRobot ?piece)
		    )
  )

  (:action put_down
	     :parameters (?piece - piece)
	     :precondition 
	     	(and 
	     		(robotOcupied)
	     		(inRobot ?piece)
	     		(robotPos storehouse)
	     	)
	     :effect
	     	(and
		     	(not(robotOcupied))
		     	(stored ?piece)
		     	(not(inRobot ?piece))
		     )
  )
 
 (:action load
		:parameters (?piece - piece ?workstation - workstation ?workshop - place)
		:precondition 
			(and 
				(robotOcupied)
				(inRobot ?piece)
				(not (workstationOcupied ?workstation))
				(robotPos ?workshop)
				(at ?workstation ?workshop)
			)
		:effect
			(and 
				(not (robotOcupied))
				(not (inRobot ?piece))
				(in ?piece ?workstation)
				(workstationOcupied ?workstation)
			)
 )
  (:action unload
  	:parameters (?piece - piece ?workstation - workstation ?workshop - place)
	:precondition 
		(and 
			(not(robotOcupied))
			(in ?piece ?workstation)
			(workstationOcupied ?workstation)
			(robotPos ?workshop)
			(at ?workstation ?workshop)
		)
	:effect
		(and 
			(robotOcupied)
			(inRobot ?piece)
			(not(in ?piece ?workstation))
			(not(workstationOcupied ?workstation))
		)
	)
 
 (:action go
	:parameters (?x - place ?y - place)
	:precondition
		(and
			(robotPos ?x)
		)
	:effect
		(and
			(not(robotPos ?x))
			(robotPos ?y)
		)
 )

 (:action transform
 	:parameters (?piece - piece ?workstation - workstation ?type1 - type ?type2 - type ?workshop - place)
 	:precondition
 		(and
 			(in ?piece ?workstation)
 			(robotPos ?workshop)
			(at ?workstation ?workshop)
			(ofType ?piece ?type1)
			(accepts ?workstation ?type1)
			(transforms ?workstation ?type2)
 		)
 	:effect
 	 	(and
			(not(ofType ?piece ?type1))
			(ofType ?piece ?type2)
 		)
 )

 (:task move_robot
	:parameters (?r - place)

	(:method Case1
		:precondition (robotPos ?r)
		:tasks ()
	)


	(:method Case2
		:precondition (and (robotPos ?r2))
		:tasks ( 
			(go ?r2 ?r)
		)
	)
 )

 (:task get
 	:parameters (?piece - piece)
 	(:method Case1
 		:precondition 
 			(stored ?piece)
 		:task (
 			(move_robot storehouse)
 			(pick_up ?piece)
 		)
 	)
 	(:method Case2
 		:precondition 
 			(and
 				(in ?piece ?wstation)
 				(at ?wstation ?wshop)
 			)
 		:task (
 			(move_robot ?wshop)
 			(unload ?piece ?wstation ?wshop)
 		)
 	)
 )
  (:task put
 	:parameters (?piece - piece)
 	(:method Case1
 		:precondition 
 			(robotPos storehouse)
 		:task (
 			(put_down ?piece)
 		)
 	)
 	(:method Case2
 		:precondition 
 			(and
 				(robotPos ?wshop)
 				(at ?wstation ?wshop)
 				(ofType ?piece ?tp)
 				(accepts ?wstation ?tp)
 			)
 		:task (
 			(load ?piece ?wstation ?wshop)
 		)
 	)
 )

 (:task work
 	:parameters (?piece - piece ?newType - type)
 	(:method Case1
 		:precondition
 			(ofType ?piece ?newType)
 		:task(
 			(get ?piece)
 			(move_robot storehouse)
 			(put ?piece)
 		)
 	)
 	(:method Case2
 		:precondition
 			(and
 				(ofType ?piece ?tp)
 				(accepts ?wstation ?tp)
 				(transforms ?wstation ?tp2)
 				(at ?wstation ?wshop)
 			)
 		:task(
 			(get ?piece)
 			(move_robot ?wshop)
 			(put ?piece)
 			(transform ?piece ?wstation ?tp ?tp2 ?wshop)
 			(work ?piece ?newType)
 		)
 	)
 )

)
