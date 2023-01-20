(define (domain emergency_service_logistic_v2)
 (:requirements :strips :typing :negative-preconditions)
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
    )


; MOVE ROBOT (LOADED/UNLOADED) FROM A LOCATION TO ANOTHER
; Moves an unloaded robot between two locations (adjacency not specified since all the locations are adjacent, with one single movement is possible to reach any of these)
    (:action move_unloaded_robot
     :parameters (?r - robotic_agent ?from ?to - location)
     :precondition (and (robotic_agent_located_at ?r ?from) 
                        (robot_is_unloaded ?r)
                    )
     :effect (and (not(robotic_agent_located_at ?r ?from))
                      (robotic_agent_located_at ?r ?to)
            )
    )

; Moves a loaded robot between two locations, we are also moving the loaded package with it
    (:action move_loaded_robot
     :parameters (?r - robotic_agent ?from ?to - location ?b - box)
     :precondition (and (robotic_agent_located_at ?r ?from) 
                        (robot_is_loaded ?r ?b)
                        (box_located_at ?b ?from)
                        (box_is_loaded ?b ?r)
                    )
     :effect (and (not(robotic_agent_located_at ?r ?from))
                      (robotic_agent_located_at ?r ?to)
                  (not(box_located_at ?b ?from))
                      (box_located_at ?b ?to)
            )
    )


; LOAD OR UNLOAD A ROBOT WITH A SPECIFIC BOX
; Load a robot with a specific box
    (:action load_robot
     :parameters (?r - robotic_agent ?l - location ?b - box)
     :precondition (and (robotic_agent_located_at ?r ?l) 
                        (robot_is_unloaded ?r)
                        (box_located_at ?b ?l)
                        (box_is_unloaded ?b)
                    )
     :effect (and (not(robot_is_unloaded ?r))
                      (robot_is_loaded ?r ?b)
                  (not(box_is_unloaded ?b))
                      (box_is_loaded ?b ?r)
            )
    )

; Unload the robot from the box loaded over it
    (:action unload_robot
     :parameters (?r - robotic_agent ?l - location ?b - box)
     :precondition (and (robotic_agent_located_at ?r ?l) 
                        (robot_is_loaded ?r ?b)
                        (box_located_at ?b ?l)
                        (box_is_loaded ?b ?r)
                    )
     :effect (and (not(robot_is_loaded ?r ?b))
                      (robot_is_unloaded ?r)
                  (not(box_is_loaded ?b ?r))
                      (box_is_unloaded ?b)
            )
    )


; FILL OR UNFILL A BOX WITH A SPECIFIC SUPPLY
; Fill a box with a specific supply, each supply is supposed to be unlimited at the depot and so it will not be consumed by the action of filling boxes
    (:action fill_box_with_supply
     :parameters (?r - robotic_agent ?l - location ?b - box ?s - supply)
     :precondition (and (robotic_agent_located_at ?r ?l)
                        (box_located_at ?b ?l)
                        (box_is_unloaded ?b)
                        (box_has_not_supply ?b)
                        (supply_located_at ?s ?l)
                    )
     :effect (and (not(box_has_not_supply ?b))
                      (box_has_supply ?b ?s)
            )
    )

; Unfill a box with a specific supply, the supply is automatically given to a person that does not have it at the unfilling location
    (:action unfill_box_with_supply
     :parameters (?r - robotic_agent ?l - location ?b - box ?p - injured_person ?s - supply)
     :precondition (and (robotic_agent_located_at ?r ?l)
                        (box_located_at ?b ?l)
                        (box_is_unloaded ?b)
                        (box_has_supply ?b ?s)
                        (injured_person_located_at ?p ?l)
                        (injured_person_has_not_supply ?p ?s)
                    )
     :effect (and (not(box_has_supply ?b ?s))
                      (box_has_not_supply ?b)
                  (not(injured_person_has_not_supply ?p ?s))
                      (injured_person_has_supply ?p ?s)
            )
    )


; MOVE CARRIER FROM A LOCATION TO ANOTHER 
; Moves a 1 loaded carrier between two locations, we are also moving the loaded package and the robot who drives it 
; The robotic agent that drives it needs to be loaded. Indeed, loading the robot is always the fastes and cheapest option, so first we want to saturate the robot capacity, then we use the carrier
    (:action move_1_loaded_carrier
     :parameters (?c - carrier ?r - robotic_agent ?from ?to - location ?rb ?cb - box)
     :precondition (and (carrier_is_located_at ?c ?from)
                        (robotic_agent_located_at ?r ?from) 
                        (robot_is_loaded ?r ?rb)
                        (box_located_at ?rb ?from)
                        (box_is_loaded ?rb ?r)
                        (box_located_at ?cb ?from)
                        (box_is_carrier_loaded ?cb ?c)
                        (carrier_position_1_loaded ?c ?cb)
                        (carrier_position_2_free ?c)
                        (carrier_position_3_free ?c)
                        (carrier_position_4_free ?c)
                    )
     :effect (and (not(carrier_is_located_at ?c ?from))
                      (carrier_is_located_at ?c ?to)
                  (not(robotic_agent_located_at ?r ?from))
                      (robotic_agent_located_at ?r ?to)
                  (not(box_located_at ?rb ?from))
                      (box_located_at ?rb ?to)
                  (not(box_located_at ?cb ?from))
                      (box_located_at ?cb ?to)
            )
    )

; Moves a 2 loaded carrier between two locations, we are also moving the loaded package and the robot who drives it 
; The robotic agent that drives it needs to be loaded. Indeed, loading the robot is always the fastes and cheapest option, so first we want to saturate the robot capacity, then we use the carrier
; We assume that the carrier need always to be filled from top to bottom to be moved. I.e., if it is loaded in position 2, then it need to be loaded also in position 1 to be moved
    (:action move_1_2_loaded_carrier
     :parameters (?c - carrier ?r - robotic_agent ?from ?to - location ?rb ?cb1 ?cb2 - box)
     :precondition (and (carrier_is_located_at ?c ?from)
                        (robotic_agent_located_at ?r ?from) 
                        (robot_is_loaded ?r ?rb)
                        (box_located_at ?rb ?from)
                        (box_is_loaded ?rb ?r)
                        (box_located_at ?cb1 ?from)
                        (box_is_carrier_loaded ?cb1 ?c)
                        (carrier_position_1_loaded ?c ?cb1)
                        (box_located_at ?cb2 ?from)
                        (box_is_carrier_loaded ?cb2 ?c)
                        (carrier_position_2_loaded ?c ?cb2)
                        (carrier_position_3_free ?c)
                        (carrier_position_4_free ?c)
                    )
     :effect (and (not(carrier_is_located_at ?c ?from))
                      (carrier_is_located_at ?c ?to)
                  (not(robotic_agent_located_at ?r ?from))
                      (robotic_agent_located_at ?r ?to)
                  (not(box_located_at ?rb ?from))
                      (box_located_at ?rb ?to)
                  (not(box_located_at ?cb1 ?from))
                      (box_located_at ?cb1 ?to)
                  (not(box_located_at ?cb2 ?from))
                      (box_located_at ?cb2 ?to)
            )
    )

; Moves a 3 loaded carrier between two locations, we are also moving the loaded package and the robot who drives it 
; The robotic agent that drives it needs to be loaded. Indeed, loading the robot is always the fastes and cheapest option, so first we want to saturate the robot capacity, then we use the carrier
; We assume that the carrier need always to be filled from top to bottom to be moved. I.e., if it is loaded in position 3, then it need to be loaded also in position 1 and 2 to be moved
    (:action move_1_2_3_loaded_carrier
     :parameters (?c - carrier ?r - robotic_agent ?from ?to - location ?rb ?cb1 ?cb2 ?cb3 - box)
     :precondition (and (carrier_is_located_at ?c ?from)
                        (robotic_agent_located_at ?r ?from) 
                        (robot_is_loaded ?r ?rb)
                        (box_located_at ?rb ?from)
                        (box_is_loaded ?rb ?r)
                        (box_located_at ?cb1 ?from)
                        (box_is_carrier_loaded ?cb1 ?c)
                        (carrier_position_1_loaded ?c ?cb1)
                        (box_located_at ?cb2 ?from)
                        (box_is_carrier_loaded ?cb2 ?c)
                        (carrier_position_2_loaded ?c ?cb2)
                        (box_located_at ?cb3 ?from)
                        (box_is_carrier_loaded ?cb3 ?c)
                        (carrier_position_3_loaded ?c ?cb3)
                        (carrier_position_4_free ?c)
                    )
     :effect (and (not(carrier_is_located_at ?c ?from))
                      (carrier_is_located_at ?c ?to)
                  (not(robotic_agent_located_at ?r ?from))
                      (robotic_agent_located_at ?r ?to)
                  (not(box_located_at ?rb ?from))
                      (box_located_at ?rb ?to)
                  (not(box_located_at ?cb1 ?from))
                      (box_located_at ?cb1 ?to)
                  (not(box_located_at ?cb2 ?from))
                      (box_located_at ?cb2 ?to)
                  (not(box_located_at ?cb3 ?from))
                      (box_located_at ?cb3 ?to)
            )
    )

; Moves a 4 loaded carrier between two locations, we are also moving the loaded package and the robot who drives it 
; The robotic agent that drives it needs to be loaded. Indeed, loading the robot is always the fastes and cheapest option, so first we want to saturate the robot capacity, then we use the carrier
; We assume that the carrier need always to be filled from top to bottom to be moved. I.e., if it is loaded in position 4, then it need to be loaded also in position 1, 2 and 3 to be moved
    (:action move_1_2_3_4_loaded_carrier
     :parameters (?c - carrier ?r - robotic_agent ?from ?to - location ?rb ?cb1 ?cb2 ?cb3 ?cb4 - box)
     :precondition (and (carrier_is_located_at ?c ?from)
                        (robotic_agent_located_at ?r ?from) 
                        (robot_is_loaded ?r ?rb)
                        (box_located_at ?rb ?from)
                        (box_is_loaded ?rb ?r)
                        (box_located_at ?cb1 ?from)
                        (box_is_carrier_loaded ?cb1 ?c)
                        (carrier_position_1_loaded ?c ?cb1)
                        (box_located_at ?cb2 ?from)
                        (box_is_carrier_loaded ?cb2 ?c)
                        (carrier_position_2_loaded ?c ?cb2)
                        (box_located_at ?cb3 ?from)
                        (box_is_carrier_loaded ?cb3 ?c)
                        (carrier_position_3_loaded ?c ?cb3)
                        (box_located_at ?cb4 ?from)
                        (box_is_carrier_loaded ?cb4 ?c)
                        (carrier_position_4_loaded ?c ?cb4)
                    )
     :effect (and (not(carrier_is_located_at ?c ?from))
                      (carrier_is_located_at ?c ?to)
                  (not(robotic_agent_located_at ?r ?from))
                      (robotic_agent_located_at ?r ?to)
                  (not(box_located_at ?rb ?from))
                      (box_located_at ?rb ?to)
                  (not(box_located_at ?cb1 ?from))
                      (box_located_at ?cb1 ?to)
                  (not(box_located_at ?cb2 ?from))
                      (box_located_at ?cb2 ?to)
                  (not(box_located_at ?cb3 ?from))
                      (box_located_at ?cb3 ?to)
                  (not(box_located_at ?cb4 ?from))
                      (box_located_at ?cb4 ?to)
            )
    )


; LOAD A CARRIER WITH A SPECIFIC NUMBER OF BOXES
; Boxes are loaded in the first available position. Each position can then be unloaded independently from which other positions are already filled. This allows us always be able to unload the
; wanted box without needing to unload other boxes. In this way we also avoid all the positional attributes (like: ?b1 ?b2 ?b3) that we already have in the move_XXX_loaded_carrier action. 
; This enables us to reason about one boxes at time using the correct predicate which specifies the postion of the wanted box in the name of the predicate.

; Load a carrier with a box in the first box position
    (:action load_carrier_position_1
     :parameters (?c - carrier ?r - robotic_agent ?l - location ?b - box)
     :precondition (and (carrier_is_located_at ?c ?l)
                        (robotic_agent_located_at ?r ?l) 
                        (box_located_at ?b ?l)
                        (box_is_unloaded ?b)
                        (carrier_position_1_free ?c)
                        (carrier_capacity_1_boxes ?c)
                    )
     :effect (and (not(carrier_position_1_free ?c))
                      (carrier_position_1_loaded ?c ?b)
                  (not(box_is_unloaded ?b))
                      (box_is_carrier_loaded ?b ?c)
            )
    )

    ; Load a carrier with a box in the second box position. To load in position 2 then the position 1 need to be already used. Instead, unloading position 2 is always possible (see below)
    (:action load_carrier_position_2
     :parameters (?c - carrier ?r - robotic_agent ?l - location ?b - box)
     :precondition (and (carrier_is_located_at ?c ?l)
                        (robotic_agent_located_at ?r ?l) 
                        (box_located_at ?b ?l)
                        (box_is_unloaded ?b)
                        (not(carrier_position_1_free ?c))
                        (carrier_position_2_free ?c)
                        (carrier_capacity_2_boxes ?c)
                    )
     :effect (and (not(carrier_position_2_free ?c))
                      (carrier_position_2_loaded ?c ?b)
                  (not(box_is_unloaded ?b))
                      (box_is_carrier_loaded ?b ?c)
            )
    )

    ; Load a carrier with a box in the third box position. To load in position 3 then the position 1 and 2 need to be already used. Instead, unloading position 3 is always possible (see below)
    (:action load_carrier_position_3
     :parameters (?c - carrier ?r - robotic_agent ?l - location ?b - box)
     :precondition (and (carrier_is_located_at ?c ?l)
                        (robotic_agent_located_at ?r ?l) 
                        (box_located_at ?b ?l)
                        (box_is_unloaded ?b)
                        (not(carrier_position_1_free ?c))
                        (not(carrier_position_2_free ?c))
                        (carrier_position_3_free ?c)
                        (carrier_capacity_3_boxes ?c)
                    )
     :effect (and (not(carrier_position_3_free ?c))
                      (carrier_position_3_loaded ?c ?b)
                  (not(box_is_unloaded ?b))
                      (box_is_carrier_loaded ?b ?c)
            )
    )

    ; Load a carrier with a box in the fourth box position. To load in position 4 then the position 1, 2 and 3 need to be already used. Instead, unloading position 4 is always possible (see below)
    (:action load_carrier_position_4
     :parameters (?c - carrier ?r - robotic_agent ?l - location ?b - box)
     :precondition (and (carrier_is_located_at ?c ?l)
                        (robotic_agent_located_at ?r ?l) 
                        (box_located_at ?b ?l)
                        (box_is_unloaded ?b)
                        (not(carrier_position_1_free ?c))
                        (not(carrier_position_2_free ?c))
                        (not(carrier_position_3_free ?c))
                        (carrier_position_4_free ?c)
                        (carrier_capacity_4_boxes ?c)
                    )
     :effect (and (not(carrier_position_4_free ?c))
                      (carrier_position_4_loaded ?c ?b)
                  (not(box_is_unloaded ?b))
                      (box_is_carrier_loaded ?b ?c)
            )
    )


; UNLOAD A CARRIER FROM A SPECIFIC NUMBER OF BOXES
; Unload a carrier from the box in the first box position
    (:action unload_carrier_position_1
     :parameters (?c - carrier ?r - robotic_agent ?l - location ?b - box)
     :precondition (and (carrier_is_located_at ?c ?l)
                        (robotic_agent_located_at ?r ?l)
                        (box_located_at ?b ?l)
                        (box_is_carrier_loaded ?b ?c) 
                        (carrier_position_1_loaded ?c ?b)
                    )
     :effect (and (not(carrier_position_1_loaded ?c ?b))
                      (carrier_position_1_free ?c)
                  (not(box_is_carrier_loaded ?b ?c))
                      (box_is_unloaded ?b)
            )
    )

; Unload a carrier from the box in the second box position
    (:action unload_carrier_position_2
     :parameters (?c - carrier ?r - robotic_agent ?l - location ?b - box)
     :precondition (and (carrier_is_located_at ?c ?l)
                        (robotic_agent_located_at ?r ?l)
                        (box_located_at ?b ?l)
                        (box_is_carrier_loaded ?b ?c) 
                        (carrier_position_2_loaded ?c ?b)
                    )
     :effect (and (not(carrier_position_2_loaded ?c ?b))
                      (carrier_position_2_free ?c)
                  (not(box_is_carrier_loaded ?b ?c))
                      (box_is_unloaded ?b)
            )
    )
    
; Unload a carrier from the box in the third box position
    (:action unload_carrier_position_3
     :parameters (?c - carrier ?r - robotic_agent ?l - location ?b - box)
     :precondition (and (carrier_is_located_at ?c ?l)
                        (robotic_agent_located_at ?r ?l)
                        (box_located_at ?b ?l)
                        (box_is_carrier_loaded ?b ?c) 
                        (carrier_position_3_loaded ?c ?b)
                    )
     :effect (and (not(carrier_position_3_loaded ?c ?b))
                      (carrier_position_3_free ?c)
                  (not(box_is_carrier_loaded ?b ?c))
                      (box_is_unloaded ?b)
            )
    )

; Unload a carrier from the box in the fourth box position
    (:action unload_carrier_position_4
     :parameters (?c - carrier ?r - robotic_agent ?l - location ?b - box)
     :precondition (and (carrier_is_located_at ?c ?l)
                        (robotic_agent_located_at ?r ?l)
                        (box_located_at ?b ?l)
                        (box_is_carrier_loaded ?b ?c) 
                        (carrier_position_4_loaded ?c ?b)
                    )
     :effect (and (not(carrier_position_4_loaded ?c ?b))
                      (carrier_position_4_free ?c)
                  (not(box_is_carrier_loaded ?b ?c))
                      (box_is_unloaded ?b)
            )
    )
)