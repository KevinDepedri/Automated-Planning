(define (problem emergency_service_logistic_problem1)
  (:domain emergency_service_logistic_v1)

  (:objects
    l0 l1 l2 l3 l4 - location      ; Define 5 location. The location l0 is the depot and there are no injured people at the depot
    p1 p2 - injured_person         ; Define 2 injured_person
    r1 - robotic_agent             ; Define 1 robotic_agent
    b1 - box                       ; Define 1 box since we decide that boxes need always to be returned to depot, and since in this problem we can only carry one box at time
    food - supply_food             ; Define content_food, we assume it to be unlimited since we are at the depot
    medicine - supply_medicine     ; Define content_medicine, we assume it to be unlimited since we are at the depot
    tools - supply_tools           ; Define content_tools, we assume it to be unlimited since we are at the depot
   )

  (:init
    ; INJURED PEOPLE -------------------------------------------------
    ; Two injured people are located in two different location (Different from l0 since there are not injured people at the depot)
    (injured_person_located_at p1 l2) (injured_person_located_at p2 l4) ; Assumption

    ; Injured person 1 has nothing. He needs just food and medicine, he does not need tools
    (injured_person_has_not_supply p1 food) (injured_person_has_not_supply p1 medicine) (injured_person_has_not_supply p1 tools) ; Assumption

    ; Injured person 2 has food and medicine. He needs tools
    (injured_person_has_supply p2 food) (injured_person_has_supply p2 medicine) (injured_person_has_not_supply p2 tools) ; Assumption
    ; ----------------------------------------------------------------

    ; ROBOTIC AGENT --------------------------------------------------
    ; A single robotic agent is located at the depot to deliver boxes
    (robotic_agent_located_at r1 l0)

    ; The robotic agent is initially unloaded 
    (robot_is_unloaded r1) ; Assumption
    ; ----------------------------------------------------------------

    ; BOXES ----------------------------------------------------------
    ; Initially all the boxes are located at the depot and are unloaded and unfilled
    (box_located_at b1 l0)
    (box_is_unloaded b1) ; Assumption
    (box_has_not_supply b1) ; Assumption

    ; Initially all the contents to load in the boxes (food, medicine and tools) are located at the depot
    (supply_located_at food l0)
    (supply_located_at medicine l0)
    (supply_located_at tools l0)
    ; ----------------------------------------------------------------
  )

  (:goal
    (and
      ; INJURED PEOPLE
      ; Injured person 1 just food and medicine, he does not need tools
      (injured_person_has_supply p1 food) ; Assumption
      (injured_person_has_supply p1 medicine) ; Assumption
      
      ; Injured person 2 need tools
      (injured_person_has_supply p2 tools) ; Assumption
      
      ; ROBOTIC AGENT
      ; Robotic agent need to return to the depot to be ready to help other people
      (robotic_agent_located_at r1 l0) ; Assumption

      ; BOXES
      ; All boxes need to be returned to the depot to be ready to help other people
      (box_located_at b1 l0) ; Assumption
      ; All the boxes as the depot need to be unloaded to be ready to be filled
      (box_is_unloaded b1) ; Assumption
      ; All the boxex at the depot need to be unfilled to be redy to be filled with the right content
      (box_has_not_supply b1) ; Assumption
    )
  )
)