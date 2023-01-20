(define (domain emergency_service_logistic_v1)
 (:requirements :strips :typing)
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
  )

 (:predicates
    ;LOCATION OF OBJECTS
    (injured_person_located_at ?p - injured_person ?l - location)            ; Injured_person p is located at location l
    (robotic_agent_located_at ?r - robotic_agent ?l - location)       ; Robotic_agent r is located at location l
    (box_located_at ?b - box ?l - location)                           ; Box b is located at location l
    (supply_located_at ?s - supply ?l - location)                            ; Supply s is located at location l
    
    ;BELONGING OF SUPPLIES
    (injured_person_has_supply ?p - injured_person ?s - supply)              ; Injured_person p has a specific supply
    (injured_person_has_not_supply ?p - injured_person ?s - supply)      ; Injured_person p has not a specific supply

    ;CONTENT OF BOXES
    (box_has_not_supply ?b - box)                                     ; Box b is unfilled and has not any supply
    (box_has_supply ?b - box ?s - supply)                             ; Box b is filled with a specific supply
 
    ;STATES OF BOXES
    (box_is_unloaded ?b - box)                                        ; Box b is unloaded from any robotic agent
    (box_is_loaded ?b - box ?r - robotic_agent)                       ; Box b is loaded on a specific robotic agent r

    ;STATES OF ROBOT
    (robot_is_unloaded ?r - robotic_agent)                            ; Robotic_agent r is unloaded from any box
    (robot_is_loaded ?r - robotic_agent ?b - box)                     ; Robotic_agent r is loaded with a spceific box b
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

; Unload the robot from the package loaded over it
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
)