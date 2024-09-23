(define (problem fr_5_9)
    (:domain first-response)
    (:objects
        l1 l2 l3 l4 l5 - location
        f1 f2 f3 f4 - fire_unit
        v1 v2 v3 v4 v5 v6 v7 v8 v9 - victim
        m1 m2 - medical_unit
    )
    (:init
        (hospital l1)
        (hospital l4)
        (water-at l3)
        (water-at l2)
        (water-at l5)
        (fire l4)
        (victim-at v1 l3)
        (victim-status v1 hurt)
        (fire l2)
        (victim-at v2 l5)
        (victim-status v2 hurt)
        (victim-at v3 l1)
        (victim-status v3 dying)
        (fire l5)
        (victim-at v4 l3)
        (victim-status v4 dying)
        (victim-at v5 l4)
        (victim-status v5 hurt)
        (fire l3)
        (victim-at v6 l1)
        (victim-status v6 hurt)
        (victim-at v7 l2)
        (victim-status v7 dying)
        (fire l1)
        (victim-at v8 l4)
        (victim-status v8 dying)
        (victim-at v9 l2)
        (victim-status v9 hurt)
        (adjacent l1 l1)
        (adjacent l2 l2)
        (adjacent l3 l3)
        (adjacent l4 l4)
        (adjacent l5 l5)
        (adjacent l1 l2)
        (adjacent l2 l1)
        (adjacent l1 l3)
        (adjacent l3 l1)
        (adjacent l2 l3)
        (adjacent l3 l2)
        (adjacent l4 l1)
        (adjacent l1 l4)
        (adjacent l4 l2)
        (adjacent l2 l4)
        (fire-unit-at f1 l1)
        (fire-unit-at f2 l5)
        (fire-unit-at f3 l4)
        (fire-unit-at f4 l2)
        (medical-unit-at m1 l1)
        (medical-unit-at m2 l4)
    )
    (:goal
        (and
            (nfire l4)
            (nfire l2)
            (nfire l5)
            (nfire l3)
            (nfire l1)
            (victim-status v1 healthy)
            (victim-status v2 healthy)
            (victim-status v3 healthy)
            (victim-status v4 healthy)
            (victim-status v5 healthy)
            (victim-status v6 healthy)
            (victim-status v7 healthy)
            (victim-status v8 healthy)
            (victim-status v9 healthy)
        )
    )
)
