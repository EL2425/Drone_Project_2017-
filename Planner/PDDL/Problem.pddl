(define (problem two_drones)

	(:domain automated-drone-delivery)

	(:objects drone1 drone2 - drone
			  package1 package2 package3 - package
			  customer1 customer2 customer3 - location
	)

	(:init
		(at-drone drone1 warehouse1)
		(at-drone drone2 warehouse2)
		(at-package package1 warehouse1)
		(at-package package2 warehouse2)
		(at-package package3 warehouse1)
		(drone-empty drone1)
		(drone-ground drone1)
		(drone-empty drone2)
		(drone-ground drone2)
	)

	(:goal (and
		(at-package package1 customer1)
		(at-package package2 customer2)
		(at-package package3 customer3)
		(at-drone drone1 warehouse1)
		(at-drone drone2 warehouse2)
		(drone-ground drone1)
		(drone-ground drone2)
		)
	)
)
