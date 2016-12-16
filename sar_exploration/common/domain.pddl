; WASP Autonomous Systems Course
; Domain for Search and Rescue (SAR) Autonomy
; KTH Team 3 (Team AWESome)

(define (domain sar_autonomy)

(:requirements :strips :typing :durative-actions :fluents :action-costs)

(:types
        turtlebot
        drone
        waypoint 
        person
        crate
        content
)

(:predicates
	(tbot-at ?tbot - turtlebot ?wp - waypoint)
	(drone-at ?dr - drone ?wp - waypoint)
	(per-at ?per - person ?wp - waypoint)
	(crate-at ?cr - crate ?wp - waypoint)

	(tbot-visited ?tbot - turtlebot ?wp - waypoint)

        (free ?dr - drone)

        (carrying  ?dr - drone ?cr - crate)
        (loaded ?tbot - turtlebot ?cr - crate)
        (contains ?cr - crate ?co - content)
        (delivered  ?per - person ?co - content)
)

(:functions
	(distance ?wp1 ?wp2 - waypoint) - number
        (pick-dur) - number 
        (deliver-dur) - number
        (load-dur) - number

        (total-cost) - number
)

;(:durative-action fly-to
;    :parameters (?dr  - drone
;                 ?from - waypoint
;                 ?to   - waypoint)
;                 
;    :duration (= ?duration 10); (distance ?from ?to))
;    
;    :condition (and
;                (at start (drone-at ?dr ?from))
;               )
;               
;    :effect (and
;             (at start (not (drone-at ?dr ?from)))
;             (at end (drone-at ?dr ?to))
;             (at end (increase (total-cost) 10)); (distance ?from ?to)))
;            )
;)

(:durative-action pick-crate
    :parameters (?dr - drone
                 ?crate - crate
                 ?wp - waypoint)
                 
    :duration (= ?duration 10) ;(pick-dur))
    
    :condition (and
                (at start (crate-at ?crate ?wp))
                (at start (free ?dr))
                (over all (drone-at ?dr ?wp))
               )
               
    :effect (and
             (at start (not (crate-at ?crate ?wp)))
             (at start (not (free ?dr)))
             (at end (carrying ?dr ?crate))
             (at end (increase (total-cost) 5))
            )
)

(:durative-action load-crate
    :parameters (?dr - drone
                 ?crate - crate
                 ?tbot - turtlebot
                 ?wp - waypoint)
                
    :duration (= ?duration 10) ;(load-dur)) 
    
    :condition (and
                (at start (carrying ?dr ?crate))
                (over all (drone-at ?dr ?wp))
                (over all (tbot-at ?tbot ?wp))
               )
               
    :effect (and
             (at end (not (carrying ?dr ?crate)))
             (at end (free ?dr))
             (at end (loaded ?tbot ?crate))
             (at end (increase (total-cost) 5))
    )
)

(:durative-action drive-to
    :parameters (?tbot  - turtlebot
                 ?from - waypoint
                 ?to   - waypoint)
                 
    :duration (= ?duration 10) ;(distance ?from ?to))
    
    :condition (and
                (at start (tbot-at ?tbot ?from))
               )
               
    :effect (and
             (at start (not (tbot-at ?tbot ?from)))
             (at end (tbot-at ?tbot ?to))
             (at end (tbot-visited ?tbot ?to))
             (at end (increase (total-cost) 10)) ;(distance ?from ?to)))
            )
)

(:durative-action deliver-crate
    :parameters (?tbot - turtlebot
                 ?crate - crate 
                 ?content - content 
                 ?per - person 
                 ?wp - waypoint)
                 
    :duration (= ?duration 10) ;(deliver-dur))
    
    :condition (
                and
                (at start (loaded ?tbot ?crate))
                (at start (contains ?crate ?content))
                (over all (tbot-at ?tbot ?wp))
                (over all (per-at ?per ?wp))
               )
                 
    :effect (and
             (at start (not (loaded ?tbot ?crate)))
             (at end (not (contains ?crate ?content)))
             (at end (delivered ?per ?content))
             (at end (increase (total-cost) 5))
            )
)
 
)
