(define (problem fr_6_10)
    (:domain first-response)
    (:objects
        l1 l2 l3 l4 l5 l6 - location
        f1 f2 f3 f4 f5 f6 - fire_unit
        v1 v2 v3 v4 v5 v6 v7 v8 v9 v10 - victim
        m1 m2 m3 m4 m5 m6 - medical_unit
    )
    (:init
        (hospital l3)
        (hospital l6)
        (hospital l2)
        (water-at l1)
        (water-at l4)
        (water-at l6)
        (water-at l3)
        (fire l2)
        (victim-at v1 l3)
        (victim-status v1 dying)
        (victim-at v2 l4)
        (victim-status v2 dying)
        (fire l1)
        (victim-at v3 l4)
        (victim-status v3 hurt)
        (fire l3)
        (victim-at v4 l5)
        (victim-status v4 hurt)
        (victim-at v5 l5)
        (victim-status v5 dying)
        (fire l4)
        (victim-at v6 l6)
        (victim-status v6 dying)
        (victim-at v7 l6)
        (victim-status v7 dying)
        (fire l5)
        (victim-at v8 l5)
        (victim-status v8 hurt)
        (victim-at v9 l1)
        (victim-status v9 hurt)
        (victim-at v10 l1)
        (victim-status v10 dying)
        (adjacent l1 l1)
        (adjacent l2 l2)
        (adjacent l3 l3)
        (adjacent l4 l4)
        (adjacent l5 l5)
        (adjacent l6 l6)
        (adjacent l1 l2)
        (adjacent l2 l1)
        (adjacent l1 l3)
        (adjacent l3 l1)
        (adjacent l1 l4)
        (adjacent l4 l1)
        (adjacent l2 l3)
        (adjacent l3 l2)
        (adjacent l2 l4)
        (adjacent l4 l2)
        (adjacent l2 l5)
        (adjacent l5 l2)
        (adjacent l5 l1)
        (adjacent l1 l5)
        (adjacent l5 l3)
        (adjacent l3 l5)
        (adjacent l5 l4)
        (adjacent l4 l5)
        (adjacent l6 l1)
        (adjacent l1 l6)
        (adjacent l6 l2)
        (adjacent l2 l6)
        (adjacent l6 l3)
        (adjacent l3 l6)
        (adjacent l6 l4)
        (adjacent l4 l6)
        (fire-unit-at f1 l1)
        (fire-unit-at f2 l4)
        (fire-unit-at f3 l1)
        (fire-unit-at f4 l3)
        (fire-unit-at f5 l6)
        (fire-unit-at f6 l2)
        (medical-unit-at m1 l5)
        (medical-unit-at m2 l2)
        (medical-unit-at m3 l4)
        (medical-unit-at m4 l5)
        (medical-unit-at m5 l1)
        (medical-unit-at m6 l4)
    )
    (:goal
        (and
            (nfire l2)
            (nfire l1)
            (nfire l3)
            (nfire l4)
            (nfire l5)
            (victim-status v1 healthy)
            (victim-status v2 healthy)
            (victim-status v3 healthy)
            (victim-status v4 healthy)
            (victim-status v5 healthy)
            (victim-status v6 healthy)
            (victim-status v7 healthy)
            (victim-status v8 healthy)
            (victim-status v9 healthy)
            (victim-status v10 healthy)
        )
    )
)
