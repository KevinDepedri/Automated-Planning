(define (domain emergency_service_logistic_v4)
 (:requirements :strips :typing :durative-actions)
 (:types
    location                        ; There are several locations which are all connected as in a fully connected graph. There could be multiple person with different needs for each location

    injured_person                  ; There is a specified number of injured person at some known position. They do not move and they need one or more specific supplies (food, medicine, tools)
                                    ; each person has or does not has some specific supply. Example:(Has food|Has not medicine|Has tools).

    robotic_agent                   ; There is a specified number of robotic agents (ONE) that can:
                                    ; - Fill boxes if: agent, empty box and wanted content are at the same location
                                    ; - Empty a box by leaving the content to the current location, causing a person that need this content at that location to receive it
                                    ; - Pick up a single box and load it on itself. Its carry capacity is of only one box
                                    ; - Move to another location, either loaded or not, if it is loaded also the loaded box is moved to the new location
                                    ; - Once in a location it can deliver a box to a specific person. I assume that the robot need to unfill the box, give the content to that person and bring the box back

    box                             ; There is a specified number of boxes at a specified initial location that can be filled with (ONE) specific content.
                                    ; Boxes need to be delivered to a specific person that need the content. The content need to be extracted and the box need to be taken to reuse it (assumption)

    supply - object                 ; Content of boxes need to be modeled in generic ways, so that new type of contents can be easily added
    supply_food - supply            ; One of the possible supplies to put in the box is food
    supply_medicine - supply        ; One of the possible supplies to put in the box is medicine
    supply_tools - supply           ; One of the possible supplies to put in the box is tools

    carrier                         ; Each carriers allows to transport a specific number of boxes which need to be defined in the problem file using one of the predicated defined below for CARRIER CAPACITY
  )

 (:predicates
    ;LOCATION OF OBJECTS
    (injured_person_located_at ?p - injured_person ?l - location)               ; Injured_person p is located at location l
    (robotic_agent_located_at ?r - robotic_agent ?l - location)         ; Robotic_agent r is located at location l
    (box_located_at ?b - box ?l - location)                           ; Box b is located at location l
    (supply_located_at ?s - supply ?l - location)                               ; Supply s is located at location l
    (carrier_is_located_at ?c - carrier ?l - location)                  ; Carrier c is located at location l
    
    ;BELONGING OF SUPPLIES
    (injured_person_has_supply ?p - injured_person ?s - supply)                 ; Injured_person p has a specific supply
    (injured_person_has_not_supply ?p - injured_person ?s - supply)         ; Injured_person p has not a specific supply

    ;CONTENT OF BOXES
    (box_has_not_supply ?b - box)                                        ; Box b is unfilled and has not any supply
    (box_has_supply ?b - box ?s - supply)                                ; Box b is filled with a specific supply

    ;STATES OF BOXES
    (box_is_unloaded ?b - box)                                           ; Box b is unloaded from any robotic agent / carrier
    (box_is_loaded ?b - box ?r - robotic_agent)                          ; Box b is loaded on a specific robotic agent r
    (box_is_carrier_loaded ?b - box ?c - carrier)                       ; Box b is loaded on a specific carrier c

    ;STATES OF ROBOT
    (robot_is_unloaded ?r - robotic_agent)                               ; Robotic_agent r is unloaded from any box
    (robot_is_loaded ?r - robotic_agent ?b - box)                        ; Robotic_agent r is loaded with a spceific box b

    ;CARRIER CAPACITY (Additional predicates can be defined to extend the capacity)
    (carrier_capacity_1_boxes ?c - carrier)                                     ; Carrier c has a capacity of 1 box
    (carrier_capacity_2_boxes ?c - carrier)                                     ; Carrier c has a capacity of 2 boxes
    (carrier_capacity_3_boxes ?c - carrier)                                     ; Carrier c has a capacity of 3 boxes
    (carrier_capacity_4_boxes ?c - carrier)                                     ; Carrier c has a capacity of 4 boxes

    ;CARRIER STATE (Additional predicates can be defined to handle additional capacity, with additional load, unload and move actions)
    (carrier_position_1_free ?c - carrier)                               ; Carrier c is unloaded in position 1
    (carrier_position_2_free ?c - carrier)                               ; Carrier c is unloaded in position 2
    (carrier_position_3_free ?c - carrier)                               ; Carrier c is unloaded in position 3
    (carrier_position_4_free ?c - carrier)                               ; Carrier c is unloaded in position 4

    (carrier_position_1_loaded ?c - carrier ?b - box)                    ; Carrier c is loaded in position 1
    (carrier_position_2_loaded ?c - carrier ?b - box)                    ; Carrier c is loaded in position 2
    (carrier_position_3_loaded ?c - carrier ?b - box)                    ; Carrier c is loaded in position 3
    (carrier_position_4_loaded ?c - carrier ?b - box)                    ; Carrier c is loaded in position 4

    ;ADDITIONAL PREDICATES FOR TEMPORAL PLANNING
    ; The following predicate should be integrated into every action that we do not want to parallelize. The assumption in this problem is that no one of the possible actions can be parallelized,
    ; so the predicate should be integrated into every action. This results into an important slow-down of the planner when looking for an optimal results. For this reason the predicate has been 
    ; inserted into all the actions, but it has been commented on the actions where it is not strictly necessary, since in these actions the other constratints are sufficient to avoid the parallelization.
    
    ; Generally, we can say that only filling/unfilling of boxes and loading/unloading of the carrier absolutely need this constraints, since otherwise it would be possible to work on multiple 
    ; boxes in parallel. This because the base constraint of these actions are only linked with the box, and each box is independent from the others. 
    
    ; Instead, all the action that regards moving (robot or carrier) do not really need this additional constraint. This because all the movement actions are constrained by the starting position, 
    ; so another movement cannot be performed until the action is terminated. This because we would need a new starting location to perform a parallel movement to go from second to third location. 
    ; This new starting location will not be available until the movement is terminated. Therefore, no more that one movement will be performed from a single location.
    (robot_is_not_busy ?r - robotic_agent)                            ; Used to mark the robot as buys and avoid parallel actions when these are not wanted

    ; The following predicates will be used as check that the carrier has been loaded correctly in the load_carrier action since negative preconditions cannot be used
    (carrier_position_1_not_free ?c - carrier)                           ; Carrier c is loaded in position 1 (will be used instead of using not(carrier_position_1_free))
    (carrier_position_2_not_free ?c - carrier)                           ; Carrier c is loaded in position 2 (will be used instead of using not(carrier_position_2_free))
    (carrier_position_3_not_free ?c - carrier)                           ; Carrier c is loaded in position 3 (will be used instead of using not(carrier_position_3_free))
    (carrier_position_4_not_free ?c - carrier)                               ; Carrier c is loaded in position 4 (will be used instead of using not(carrier_position_4_free))
    )



; MOVE ROBOT (LOADED/UNLOADED) FROM A LOCATION TO ANOTHER
; Moves an unloaded robot between two locations (adjacency not specified since all the locations are adjacent, with one single movement is possible to reach any of these)
    (:durative-action move_unloaded_robot
     :parameters (?r - robotic_agent ?from ?to - location)
     :duration (= ?duration 1)
     
     :condition (and (at start (robotic_agent_located_at ?r ?from))
                     (over all (robot_is_unloaded ?r))
                    ;  (at start (robot_is_not_busy ?r))
                )
     :effect (and (at start (not (robotic_agent_located_at ?r ?from)))
                  (at end (robotic_agent_located_at ?r ?to))
               ;    (at start (not(robot_is_not_busy ?r)))
               ;    (at end (robot_is_not_busy ?r))
             )
    )

; Moves a loaded robot between two locations, we are also moving the loaded package with it
    (:durative-action move_loaded_robot
     :parameters (?r - robotic_agent ?from ?to - location ?b - box)
     :duration (= ?duration 3)
     
     :condition (and (at start (robotic_agent_located_at ?r ?from))
                     (over all (robot_is_loaded ?r ?b))
                     (at start (box_located_at ?b ?from))
                    ;  (over all (box_is_loaded ?b ?r))
                    ;  (at start (robot_is_not_busy ?r))
                )
     :effect (and (at start (not (robotic_agent_located_at ?r ?from)))
                  (at end (robotic_agent_located_at ?r ?to))
                  (at start (not (box_located_at ?b ?from)))
                  (at end (box_located_at ?b ?to))
                  (at start (not(robot_is_not_busy ?r)))
               ;    (at end (robot_is_not_busy ?r))
             )
    )


; LOAD OR UNLOAD A ROBOT WITH A SPECIFIC BOX
; Load a robot with a specific box
    (:durative-action load_robot
     :parameters (?r - robotic_agent ?l - location ?b - box)
     :duration (= ?duration 1)

     :condition (and (over all (robotic_agent_located_at ?r ?l))
                     (at start (robot_is_unloaded ?r))
                     (over all (box_located_at ?b ?l))
                     (at start (box_is_unloaded ?b))
                     (at start (robot_is_not_busy ?r))
                )
     :effect (and (at start (not(robot_is_unloaded ?r)))
                  (at end (robot_is_loaded ?r ?b))
                  (at start (not(box_is_unloaded ?b)))
                  (at end (box_is_loaded ?b ?r))
                  (at start (not(robot_is_not_busy ?r)))
                  (at end (robot_is_not_busy ?r))
             )
    )

; Unload the robot from the box loaded over it
    (:durative-action unload_robot
     :parameters (?r - robotic_agent ?l - location ?b - box)
     :duration (= ?duration 1)

     :condition (and (over all (robotic_agent_located_at ?r ?l))
                     (at start (robot_is_loaded ?r ?b))
                     (over all (box_located_at ?b ?l))
                     (at start (box_is_loaded ?b ?r))
                     (at start (robot_is_not_busy ?r))
                )
     :effect (and (at start (not(robot_is_loaded ?r ?b)))
                  (at end (robot_is_unloaded ?r))
                  (at start (not(box_is_loaded ?b ?r)))
                  (at end (box_is_unloaded ?b))
                  (at start (not(robot_is_not_busy ?r)))
                  (at end (robot_is_not_busy ?r))
             )
    )


; FILL OR UNFILL A BOX WITH A SPECIFIC SUPPLY
; Fill a box with a specific supply, each supply is supposed to be unlimited at the depot and so it will not be consumed by the action of filling boxes
    (:durative-action fill_box_with_supply
     :parameters (?r - robotic_agent ?l - location ?b - box ?s - supply)
     :duration (= ?duration 2)

     :condition (and (over all (robotic_agent_located_at ?r ?l))
                     (over all (box_located_at ?b ?l))
                     (over all (box_is_unloaded ?b))
                     (at start (box_has_not_supply ?b))
                     (over all (supply_located_at ?s ?l))
                     (at start (robot_is_not_busy ?r))
                )
     :effect (and (at start (not(box_has_not_supply ?b)))
                  (at end (box_has_supply ?b ?s))
                  (at start (not(robot_is_not_busy ?r)))
                  (at end (robot_is_not_busy ?r))
             )
    )

; Unfill a box with a specific supply, the supply is automatically given to a person that does not have it at the unfilling location
    (:durative-action unfill_box_with_supply
     :parameters (?r - robotic_agent ?l - location ?b - box ?p - injured_person ?s - supply)
     :duration (= ?duration 2)

     :condition (and (over all (robotic_agent_located_at ?r ?l))
                     (over all (box_located_at ?b ?l))
                     (over all (box_is_unloaded ?b))
                     (at start (box_has_supply ?b ?s))
                     (over all (injured_person_located_at ?p ?l))
                     (at start (injured_person_has_not_supply ?p ?s))
                     (at start (robot_is_not_busy ?r))
                )
     :effect (and (at start (not(box_has_supply ?b ?s)))
                  (at end (box_has_not_supply ?b))
                  (at start (not(injured_person_has_not_supply ?p ?s)))
                  (at end (injured_person_has_supply ?p ?s))
                  (at start (not(robot_is_not_busy ?r)))
                  (at end (robot_is_not_busy ?r))
             )
    )


; MOVE CARRIER FROM A LOCATION TO ANOTHER 
; Moves a 1 loaded carrier between two locations, we are also moving the loaded package and the robot who drives it 
; The robotic agent that drives it needs to be loaded. Indeed, loading the robot is always the fastes and cheapest option, so first we want to saturate the robot capacity, then we use the carrier
    (:durative-action move_1_loaded_carrier
     :parameters (?c - carrier ?r - robotic_agent ?from ?to - location ?rb ?cb - box)
     :duration (= ?duration 3)

     :condition (and (at start (carrier_is_located_at ?c ?from))
                     (at start (robotic_agent_located_at ?r ?from))
                     (over all (robot_is_loaded ?r ?rb))
                     (at start (box_located_at ?rb ?from))
                     (over all (box_is_loaded ?rb ?r))
                     (at start (box_located_at ?cb ?from))
                     (over all (box_is_carrier_loaded ?cb ?c))
                     (over all (carrier_position_1_loaded ?c ?cb))
                     (over all (carrier_position_2_free ?c))
                     (over all (carrier_position_3_free ?c))
                     (over all (carrier_position_4_free ?c))
                    ;  (at start (robot_is_not_busy ?r))
                )
     :effect (and (at start (not(carrier_is_located_at ?c ?from)))
                  (at end (carrier_is_located_at ?c ?to))
                  (at start (not(robotic_agent_located_at ?r ?from)))
                  (at end (robotic_agent_located_at ?r ?to))
                  (at start (not(box_located_at ?rb ?from)))
                  (at end (box_located_at ?rb ?to))
                  (at start (not(box_located_at ?cb ?from)))
                  (at end (box_located_at ?cb ?to))
               ;    (at start (not(robot_is_not_busy ?r)))
               ;    (at end (robot_is_not_busy ?r))
             )
    )

; ; Moves a 2 loaded carrier between two locations, we are also moving the loaded package and the robot who drives it 
; ; The robotic agent that drives it needs to be loaded. Indeed, loading the robot is always the fastes and cheapest option, so first we want to saturate the robot capacity, then we use the carrier
; ; We assume that the carrier need always to be filled from top to bottom to be moved. I.e., if it is loaded in position 2, then it need to be loaded also in position 1 to be moved
    (:durative-action move_1_2_loaded_carrier
     :parameters (?c - carrier ?r - robotic_agent ?from ?to - location ?rb ?cb1 ?cb2 - box)
     :duration (= ?duration 3)

     :condition (and (at start (carrier_is_located_at ?c ?from))
                     (at start (robotic_agent_located_at ?r ?from))
                     (over all (robot_is_loaded ?r ?rb))
                     (at start (box_located_at ?rb ?from))
                     (over all (box_is_loaded ?rb ?r))
                     (at start (box_located_at ?cb1 ?from))
                     (over all (box_is_carrier_loaded ?cb1 ?c))
                     (over all (carrier_position_1_loaded ?c ?cb1))
                     (at start (box_located_at ?cb2 ?from))
                     (over all (box_is_carrier_loaded ?cb2 ?c))
                     (over all (carrier_position_2_loaded ?c ?cb2))
                     (over all (carrier_position_3_free ?c))
                     (over all (carrier_position_4_free ?c))
                    ;  (at start (robot_is_not_busy ?r))
                )
     :effect (and (at start (not(carrier_is_located_at ?c ?from)))
                  (at end (carrier_is_located_at ?c ?to))
                  (at start (not(robotic_agent_located_at ?r ?from)))
                  (at end (robotic_agent_located_at ?r ?to))
                  (at start (not(box_located_at ?rb ?from)))
                  (at end (box_located_at ?rb ?to))
                  (at start (not(box_located_at ?cb1 ?from)))
                  (at end (box_located_at ?cb1 ?to))
                  (at start (not(box_located_at ?cb2 ?from)))
                  (at end (box_located_at ?cb2 ?to))
               ;    (at start (not(robot_is_not_busy ?r)))
               ;    (at end (robot_is_not_busy ?r))
             )
    )

; Moves a 3 loaded carrier between two locations, we are also moving the loaded package and the robot who drives it 
; The robotic agent that drives it needs to be loaded. Indeed, loading the robot is always the fastes and cheapest option, so first we want to saturate the robot capacity, then we use the carrier
; We assume that the carrier need always to be filled from top to bottom to be moved. I.e., if it is loaded in position 3, then it need to be loaded also in position 1 and 2 to be moved
    (:durative-action move_1_2_3_loaded_carrier
     :parameters (?c - carrier ?r - robotic_agent ?from ?to - location ?rb ?cb1 ?cb2 ?cb3 - box)
     :duration (= ?duration 3)

     :condition (and (at start (carrier_is_located_at ?c ?from))
                     (at start (robotic_agent_located_at ?r ?from))
                     (over all (robot_is_loaded ?r ?rb))
                     (at start (box_located_at ?rb ?from))
                     (over all (box_is_loaded ?rb ?r))
                     (at start (box_located_at ?cb1 ?from))
                     (over all (box_is_carrier_loaded ?cb1 ?c))
                     (over all (carrier_position_1_loaded ?c ?cb1))
                     (at start (box_located_at ?cb2 ?from))
                     (over all (box_is_carrier_loaded ?cb2 ?c))
                     (over all (carrier_position_2_loaded ?c ?cb2))
                     (at start (box_located_at ?cb3 ?from))
                     (over all (box_is_carrier_loaded ?cb3 ?c))
                     (over all (carrier_position_3_loaded ?c ?cb3))
                     (over all (carrier_position_4_free ?c))
                    ;  (at start (robot_is_not_busy ?r))
                )
     :effect (and (at start (not(carrier_is_located_at ?c ?from)))
                  (at end (carrier_is_located_at ?c ?to))
                  (at start (not(robotic_agent_located_at ?r ?from)))
                  (at end (robotic_agent_located_at ?r ?to))
                  (at start (not(box_located_at ?rb ?from)))
                  (at end (box_located_at ?rb ?to))
                  (at start (not(box_located_at ?cb1 ?from)))
                  (at end (box_located_at ?cb1 ?to))
                  (at start (not(box_located_at ?cb2 ?from)))
                  (at end (box_located_at ?cb2 ?to))
                  (at start (not(box_located_at ?cb3 ?from)))
                  (at end (box_located_at ?cb3 ?to))
               ;    (at start (not(robot_is_not_busy ?r)))
               ;    (at end (robot_is_not_busy ?r))
             )
    )

; Moves a 4 loaded carrier between two locations, we are also moving the loaded package and the robot who drives it 
; The robotic agent that drives it needs to be loaded. Indeed, loading the robot is always the fastes and cheapest option, so first we want to saturate the robot capacity, then we use the carrier
; We assume that the carrier need always to be filled from top to bottom to be moved. I.e., if it is loaded in position 4, then it need to be loaded also in position 1, 2 and 3 to be moved
    (:durative-action move_1_2_3_4_loaded_carrier
     :parameters (?c - carrier ?r - robotic_agent ?from ?to - location ?rb ?cb1 ?cb2 ?cb3 ?cb4 - box)
     :duration (= ?duration 3)

     :condition (and (at start (carrier_is_located_at ?c ?from))
                     (at start (robotic_agent_located_at ?r ?from))
                     (over all (robot_is_loaded ?r ?rb))
                     (at start (box_located_at ?rb ?from))
                     (over all (box_is_loaded ?rb ?r))
                     (at start (box_located_at ?cb1 ?from))
                     (over all (box_is_carrier_loaded ?cb1 ?c))
                     (over all (carrier_position_1_loaded ?c ?cb1))
                     (at start (box_located_at ?cb2 ?from))
                     (over all (box_is_carrier_loaded ?cb2 ?c))
                     (over all (carrier_position_2_loaded ?c ?cb2))
                     (at start (box_located_at ?cb3 ?from))
                     (over all (box_is_carrier_loaded ?cb3 ?c))
                     (over all (carrier_position_3_loaded ?c ?cb3))
                     (at start (box_located_at ?cb4 ?from))
                     (over all (box_is_carrier_loaded ?cb4 ?c))
                     (over all (carrier_position_4_loaded ?c ?cb4))
                    ;  (at start (robot_is_not_busy ?r))
                )
     :effect (and (at start (not(carrier_is_located_at ?c ?from)))
                  (at end (carrier_is_located_at ?c ?to))
                  (at start (not(robotic_agent_located_at ?r ?from)))
                  (at end (robotic_agent_located_at ?r ?to))
                  (at start (not(box_located_at ?rb ?from)))
                  (at end (box_located_at ?rb ?to))
                  (at start (not(box_located_at ?cb1 ?from)))
                  (at end (box_located_at ?cb1 ?to))
                  (at start (not(box_located_at ?cb2 ?from)))
                  (at end (box_located_at ?cb2 ?to))
                  (at start (not(box_located_at ?cb3 ?from)))
                  (at end (box_located_at ?cb3 ?to))
                  (at start (not(box_located_at ?cb4 ?from)))
                  (at end (box_located_at ?cb4 ?to))
               ;    (at start (not(robot_is_not_busy ?r)))
               ;    (at end (robot_is_not_busy ?r))
             )
    )


; LOAD A CARRIER WITH A SPECIFIC NUMBER OF BOXES
; Boxes are loaded in the first available position. Each position can then be unloaded independently from which other positions are already filled. This allows us always be able to unload the
; wanted box without needing to unload other boxes In this way we also avoid all the positional attributes (like: ?b1 ?b2 ?b3) that we already have in the move_XXX_loaded_carrier action. 
; This enables us to reason about one boxes at time using the correct predicate which specifies the postion of the wanted box in the name of the predicate.

; Load a carrier with a box in the first box position
    (:durative-action load_carrier_position_1
     :parameters (?c - carrier ?r - robotic_agent ?l - location ?b - box)
     :duration (= ?duration 1)

     :condition (and (over all (carrier_is_located_at ?c ?l))
                     (over all (robotic_agent_located_at ?r ?l))
                     (over all (box_located_at ?b ?l))
                     (at start (box_is_unloaded ?b))
                     (at start (carrier_position_1_free ?c))
                     (over all (carrier_capacity_1_boxes ?c))
                     (at start (robot_is_not_busy ?r))
                )
     :effect (and (at start (not(carrier_position_1_free ?c)))
                  (at start (carrier_position_1_not_free ?c)) ; Fix the missing negative-preconditions for the next load action
                  (at end (carrier_position_1_loaded ?c ?b))
                  (at start (not(box_is_unloaded ?b)))
                  (at end (box_is_carrier_loaded ?b ?c))
                  (at start (not(robot_is_not_busy ?r)))
                  (at end (robot_is_not_busy ?r))
             )
    )

    ; Load a carrier with a box in the second box position. To load in position 2 then the position 1 need to be already used. Instead, unloading position 2 is always possible (see below)
    (:durative-action load_carrier_position_2
     :parameters (?c - carrier ?r - robotic_agent ?l - location ?b - box)
     :duration (= ?duration 1)

     :condition (and (over all (carrier_is_located_at ?c ?l))
                     (over all (robotic_agent_located_at ?r ?l))
                     (over all (box_located_at ?b ?l))
                     (at start (box_is_unloaded ?b))
                    ;  (over all (not(carrier_position_1_free ?c))) ; NEGAGTIVE PRECONDITIONS NOT SUPPORTED ON TEMPORAL PLANNER (OPTIC)
                     (over all (carrier_position_1_not_free ?c)) ; It acts as a negative-preconditions defined by the previous load action
                     (at start (carrier_position_2_free ?c))
                     (over all (carrier_capacity_2_boxes ?c))
                     (at start (robot_is_not_busy ?r))
                )
     :effect (and (at start (not(carrier_position_2_free ?c)))
                  (at start (carrier_position_2_not_free ?c)) ; Fix the missing negative-preconditions for the next load action
                  (at end (carrier_position_2_loaded ?c ?b))
                  (at start (not(box_is_unloaded ?b)))
                  (at end (box_is_carrier_loaded ?b ?c))
                  (at start (not(robot_is_not_busy ?r)))
                  (at end (robot_is_not_busy ?r))
             )
    )

    ; ; Load a carrier with a box in the third box position. To load in position 3 then the position 1 and 2 need to be already used. Instead, unloading position 3 is always possible (see below)
    (:durative-action load_carrier_position_3
     :parameters (?c - carrier ?r - robotic_agent ?l - location ?b - box)
     :duration (= ?duration 1)

     :condition (and (over all (carrier_is_located_at ?c ?l))
                     (over all (robotic_agent_located_at ?r ?l))
                     (over all (box_located_at ?b ?l))
                     (at start (box_is_unloaded ?b))
                    ;  (over all (not(carrier_position_1_free ?c))) ; NEGAGTIVE PRECONDITIONS NOT SUPPORTED ON TEMPORAL PLANNER (OPTIC)
                    ;  (over all (not(carrier_position_2_free ?c))) ; NEGAGTIVE PRECONDITIONS NOT SUPPORTED ON TEMPORAL PLANNER (OPTIC)
                     (over all (carrier_position_1_not_free ?c)) ; It acts as a negative-preconditions defined by the previous load actions
                     (over all (carrier_position_2_not_free ?c)) ; It acts as a negative-preconditions defined by the previous load actions
                     (at start (carrier_position_3_free ?c))
                     (over all (carrier_capacity_3_boxes ?c))
                     (at start (robot_is_not_busy ?r))
                )
     :effect (and (at start (not(carrier_position_3_free ?c)))
                  (at start (carrier_position_3_not_free ?c)) ; Fix the missing negative-preconditions for the next load action
                  (at end (carrier_position_3_loaded ?c ?b))
                  (at start (not(box_is_unloaded ?b)))
                  (at end (box_is_carrier_loaded ?b ?c))
                  (at start (not(robot_is_not_busy ?r)))
                  (at end (robot_is_not_busy ?r))
             )
    )

    ; ; Load a carrier with a box in the fourth box position. To load in position 4 then the position 1, 2 and 3 need to be already used. Instead, unloading position 4 is always possible (see below)
    (:durative-action load_carrier_position_4
     :parameters (?c - carrier ?r - robotic_agent ?l - location ?b - box)
     :duration (= ?duration 1)

     :condition (and (over all (carrier_is_located_at ?c ?l))
                     (over all (robotic_agent_located_at ?r ?l))
                     (over all (box_located_at ?b ?l))
                     (at start (box_is_unloaded ?b))
                    ;  (over all (not(carrier_position_1_free ?c))) ; NEGAGTIVE PRECONDITIONS NOT SUPPORTED ON TEMPORAL PLANNER (OPTIC)
                    ;  (over all (not(carrier_position_2_free ?c))) ; NEGAGTIVE PRECONDITIONS NOT SUPPORTED ON TEMPORAL PLANNER (OPTIC)
                    ;  (over all (not(carrier_position_3_free ?c))) ; NEGAGTIVE PRECONDITIONS NOT SUPPORTED ON TEMPORAL PLANNER (OPTIC)
                     (over all (carrier_position_1_not_free ?c)) ; It acts as a negative-preconditions defined by the previous load actions
                     (over all (carrier_position_2_not_free ?c)) ; It acts as a negative-preconditions defined by the previous load actions
                     (over all (carrier_position_3_not_free ?c)) ; It acts as a negative-preconditions defined by the previous load actions
                     (at start (carrier_position_4_free ?c))
                     (over all (carrier_capacity_4_boxes ?c))
                     (at start (robot_is_not_busy ?r))
                )
     :effect (and (at start (not(carrier_position_4_free ?c)))
                  (at start (carrier_position_4_not_free ?c)) ; Fix the missing negative-preconditions
                  (at end (carrier_position_4_loaded ?c ?b))
                  (at start (not(box_is_unloaded ?b)))
                  (at end (box_is_carrier_loaded ?b ?c))
                  (at start (not(robot_is_not_busy ?r)))
                  (at end (robot_is_not_busy ?r))
             )
    )


; UNLOAD A CARRIER FROM A SPECIFIC NUMBER OF BOXES
; Unload a carrier from the box in the first box position
    (:durative-action unload_carrier_position_1
     :parameters (?c - carrier ?r - robotic_agent ?l - location ?b - box)
     :duration (= ?duration 1)

     :condition (and (over all (carrier_is_located_at ?c ?l))
                     (over all (robotic_agent_located_at ?r ?l))
                     (over all (box_located_at ?b ?l))
                     (at start (box_is_carrier_loaded ?b ?c))
                     (at start (carrier_position_1_loaded ?c ?b))
                     (at start (robot_is_not_busy ?r))
                )
     :effect (and (at start (not(carrier_position_1_loaded ?c ?b)))
                  (at start (not(carrier_position_1_not_free ?c))) ; Fix the missing negative-preconditions for the load actions
                  (at end (carrier_position_1_free ?c))
                  (at start (not(box_is_carrier_loaded ?b ?c)))
                  (at end (box_is_unloaded ?b))
                  (at start (not(robot_is_not_busy ?r)))
                  (at end (robot_is_not_busy ?r))
             )
    )

; Unload a carrier from the box in the second box position
    (:durative-action unload_carrier_position_2
     :parameters (?c - carrier ?r - robotic_agent ?l - location ?b - box)
     :duration (= ?duration 1)

     :condition (and (over all (carrier_is_located_at ?c ?l))
                     (over all (robotic_agent_located_at ?r ?l))
                     (over all (box_located_at ?b ?l))
                     (at start (box_is_carrier_loaded ?b ?c))
                     (at start (carrier_position_2_loaded ?c ?b))
                     (at start (robot_is_not_busy ?r))
                )
     :effect (and (at start (not(carrier_position_2_loaded ?c ?b)))
                  (at start (not(carrier_position_2_not_free ?c))) ; Fix the missing negative-preconditions for the load actions
                  (at end (carrier_position_2_free ?c))
                  (at start (not(box_is_carrier_loaded ?b ?c)))
                  (at end (box_is_unloaded ?b))
                  (at start (not(robot_is_not_busy ?r)))
                  (at end (robot_is_not_busy ?r))
             )
    )
    
; Unload a carrier from the box in the third box position
    (:durative-action unload_carrier_position_3
     :parameters (?c - carrier ?r - robotic_agent ?l - location ?b - box)
     :duration (= ?duration 1)

     :condition (and (over all (carrier_is_located_at ?c ?l))
                     (over all (robotic_agent_located_at ?r ?l))
                     (over all (box_located_at ?b ?l))
                     (at start (box_is_carrier_loaded ?b ?c))
                     (at start (carrier_position_3_loaded ?c ?b))
                     (at start (robot_is_not_busy ?r))
                )
     :effect (and (at start (not(carrier_position_3_loaded ?c ?b)))
                  (at start (not(carrier_position_3_not_free ?c))) ; Fix the missing negative-preconditions for the load actions
                  (at end (carrier_position_3_free ?c))
                  (at start (not(box_is_carrier_loaded ?b ?c)))
                  (at end (box_is_unloaded ?b))
                  (at start (not(robot_is_not_busy ?r)))
                  (at end (robot_is_not_busy ?r))
             )
    )

; Unload a carrier from the box in the fourth box position
    (:durative-action unload_carrier_position_4
     :parameters (?c - carrier ?r - robotic_agent ?l - location ?b - box)
     :duration (= ?duration 1)

     :condition (and (over all (carrier_is_located_at ?c ?l))
                     (over all (robotic_agent_located_at ?r ?l))
                     (over all (box_located_at ?b ?l))
                     (at start (box_is_carrier_loaded ?b ?c))
                     (at start (carrier_position_4_loaded ?c ?b))
                     (at start (robot_is_not_busy ?r))
                )
     :effect (and (at start (not(carrier_position_4_loaded ?c ?b)))
                  (at start (not(carrier_position_4_not_free ?c))) ; Fix the missing negative-preconditions for the load actions
                  (at end (carrier_position_4_free ?c))
                  (at start (not(box_is_carrier_loaded ?b ?c)))
                  (at end (box_is_unloaded ?b))
                  (at start (not(robot_is_not_busy ?r)))
                  (at end (robot_is_not_busy ?r))
             )
    )
)