(define (domain automated-drone-delivery)
	
	(:requirements :typing)

	(:types drone package location)

	(:constants warehouse1 warehouse2 - location)

	(:predicates
		(drone-empty ?d - drone)
		(holding ?d - drone ?p - package)
		(at-drone ?d - drone ?l - location)
		(at-package ?p - package ?l - location)
		(drone-ground ?d - drone) ;;tag to make drone land prior to load and etc
		(drone-fly ?d - drone) ;;tag to make drone takeoff prior to move
		
		
		
	)

	(:action TAKEOFF
		:parameters (?d - drone)
		:precondition (drone-ground ?d) 
		:effect (and (not (drone-ground ?d))
					 (drone-fly ?d))
	)
	
	(:action LAND
		:parameters (?d - drone)
		:precondition (drone-fly ?d) 
		:effect (and (drone-ground ?d)
					 (not (drone-fly ?d)))
	)
	
	(:action LOAD
		:parameters (?d - drone ?p - package ?l - location)
		:precondition (and (at-drone ?d ?l)
						(drone-empty ?d)
						(at-package ?p ?l)
						(drone-ground ?d))
		:effect (and (holding ?d ?p)
					 (not (drone-empty ?d))
					 (not (at-package ?p ?l)))
	)

	(:action UNLOAD
		:parameters (?d - drone ?p - package ?l - location)
		:precondition (and (at-drone ?d ?l)
						(holding ?d ?p)
						(drone-ground ?d))
		:effect (and (not (holding ?d ?p))
					 (drone-empty ?d)
					 (at-package ?p ?l))
	)

	(:action MOVE
		:parameters (?d - drone ?from ?to - location)
		:precondition (and (at-drone ?d ?from)
							(drone-fly ?d))
		:effect (and (at-drone ?d ?to)
					 (not (at-drone ?d ?from)))
	)
)
