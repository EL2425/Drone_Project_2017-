(define (domain automated-drone-delivery)
	
	(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

	(:types drone package location)


	(:predicates
		(drone-empty ?d - drone)
		(holding ?d - drone ?p - package)
		(at-drone ?d - drone ?l - location)
		(at-package ?p - package ?l - location)
		(drone-ground ?d - drone) ;;tag to make drone land prior to load and etc
		(drone-fly ?d - drone) ;;tag to make drone takeoff prior to move
		(can-move ?d - drone)
		
		
		
	)

	(:durative-action TAKEOFF
		:parameters (?d - drone)
		:duration ( = ?duration 5)
		:condition (at start (drone-ground ?d)) 
		:effect (and (at start(not (drone-ground ?d)))
					 (at end(drone-fly ?d))
					 (at end(can-move ?d)))
	)
	
	(:durative-action LAND
		:parameters (?d - drone)
		:duration ( = ?duration 5)
		:condition (at start(drone-fly ?d)) 
		:effect (and (at end(drone-ground ?d))
					 (at start(not(can-move ?d))))
	)

	
	(:durative-action LOAD
		:parameters (?d - drone ?p - package ?l - location)
		:duration (= ?duration 5)
		:condition (and (over all(at-drone ?d ?l))
						(at start(drone-empty ?d))
						(at start(at-package ?p ?l))
						(over all(drone-ground ?d)))
		:effect (and (at start(holding ?d ?p))
					 (at start(not (drone-empty ?d)))
					 (at end(not (at-package ?p ?l))))
	)

	(:durative-action UNLOAD
		:parameters (?d - drone ?p - package ?l - location)
		:duration (= ?duration 5)
		:condition (and (over all(at-drone ?d ?l))
						(at start(holding ?d ?p))
						(over all(drone-ground ?d)))
		:effect (and (at end(not (holding ?d ?p)))
					 (at end(drone-empty ?d))
					 (at end(at-package ?p ?l)))
	)

	(:durative-action MOVE
		:parameters (?d - drone ?from ?to - location)
		:duration (= ?duration 5)
		:condition (and (at start(at-drone ?d ?from))
							(over all(drone-fly ?d))
							(over all(can-move ?d)))
		:effect (and (at start(not(at-drone ?d ?from)))
					 (at end(at-drone ?d ?to))
					 (at start(not (at-drone ?d ?from)))
					 )
	)
)
