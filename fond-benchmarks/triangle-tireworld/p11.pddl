(define (problem triangle-tire-11)(:domain triangle-tire)(:objects l-1-1 l-1-2 l-1-3 l-1-4 l-1-5 l-1-6 l-1-7 l-1-8 l-1-9 l-1-10 l-1-11 l-1-12 l-1-13 l-1-14 l-1-15 l-1-16 l-1-17 l-1-18 l-1-19 l-1-20 l-1-21 l-1-22 l-1-23 l-2-1 l-2-2 l-2-3 l-2-4 l-2-5 l-2-6 l-2-7 l-2-8 l-2-9 l-2-10 l-2-11 l-2-12 l-2-13 l-2-14 l-2-15 l-2-16 l-2-17 l-2-18 l-2-19 l-2-20 l-2-21 l-2-22 l-2-23 l-3-1 l-3-2 l-3-3 l-3-4 l-3-5 l-3-6 l-3-7 l-3-8 l-3-9 l-3-10 l-3-11 l-3-12 l-3-13 l-3-14 l-3-15 l-3-16 l-3-17 l-3-18 l-3-19 l-3-20 l-3-21 l-3-22 l-3-23 l-4-1 l-4-2 l-4-3 l-4-4 l-4-5 l-4-6 l-4-7 l-4-8 l-4-9 l-4-10 l-4-11 l-4-12 l-4-13 l-4-14 l-4-15 l-4-16 l-4-17 l-4-18 l-4-19 l-4-20 l-4-21 l-4-22 l-4-23 l-5-1 l-5-2 l-5-3 l-5-4 l-5-5 l-5-6 l-5-7 l-5-8 l-5-9 l-5-10 l-5-11 l-5-12 l-5-13 l-5-14 l-5-15 l-5-16 l-5-17 l-5-18 l-5-19 l-5-20 l-5-21 l-5-22 l-5-23 l-6-1 l-6-2 l-6-3 l-6-4 l-6-5 l-6-6 l-6-7 l-6-8 l-6-9 l-6-10 l-6-11 l-6-12 l-6-13 l-6-14 l-6-15 l-6-16 l-6-17 l-6-18 l-6-19 l-6-20 l-6-21 l-6-22 l-6-23 l-7-1 l-7-2 l-7-3 l-7-4 l-7-5 l-7-6 l-7-7 l-7-8 l-7-9 l-7-10 l-7-11 l-7-12 l-7-13 l-7-14 l-7-15 l-7-16 l-7-17 l-7-18 l-7-19 l-7-20 l-7-21 l-7-22 l-7-23 l-8-1 l-8-2 l-8-3 l-8-4 l-8-5 l-8-6 l-8-7 l-8-8 l-8-9 l-8-10 l-8-11 l-8-12 l-8-13 l-8-14 l-8-15 l-8-16 l-8-17 l-8-18 l-8-19 l-8-20 l-8-21 l-8-22 l-8-23 l-9-1 l-9-2 l-9-3 l-9-4 l-9-5 l-9-6 l-9-7 l-9-8 l-9-9 l-9-10 l-9-11 l-9-12 l-9-13 l-9-14 l-9-15 l-9-16 l-9-17 l-9-18 l-9-19 l-9-20 l-9-21 l-9-22 l-9-23 l-10-1 l-10-2 l-10-3 l-10-4 l-10-5 l-10-6 l-10-7 l-10-8 l-10-9 l-10-10 l-10-11 l-10-12 l-10-13 l-10-14 l-10-15 l-10-16 l-10-17 l-10-18 l-10-19 l-10-20 l-10-21 l-10-22 l-10-23 l-11-1 l-11-2 l-11-3 l-11-4 l-11-5 l-11-6 l-11-7 l-11-8 l-11-9 l-11-10 l-11-11 l-11-12 l-11-13 l-11-14 l-11-15 l-11-16 l-11-17 l-11-18 l-11-19 l-11-20 l-11-21 l-11-22 l-11-23 l-12-1 l-12-2 l-12-3 l-12-4 l-12-5 l-12-6 l-12-7 l-12-8 l-12-9 l-12-10 l-12-11 l-12-12 l-12-13 l-12-14 l-12-15 l-12-16 l-12-17 l-12-18 l-12-19 l-12-20 l-12-21 l-12-22 l-12-23 l-13-1 l-13-2 l-13-3 l-13-4 l-13-5 l-13-6 l-13-7 l-13-8 l-13-9 l-13-10 l-13-11 l-13-12 l-13-13 l-13-14 l-13-15 l-13-16 l-13-17 l-13-18 l-13-19 l-13-20 l-13-21 l-13-22 l-13-23 l-14-1 l-14-2 l-14-3 l-14-4 l-14-5 l-14-6 l-14-7 l-14-8 l-14-9 l-14-10 l-14-11 l-14-12 l-14-13 l-14-14 l-14-15 l-14-16 l-14-17 l-14-18 l-14-19 l-14-20 l-14-21 l-14-22 l-14-23 l-15-1 l-15-2 l-15-3 l-15-4 l-15-5 l-15-6 l-15-7 l-15-8 l-15-9 l-15-10 l-15-11 l-15-12 l-15-13 l-15-14 l-15-15 l-15-16 l-15-17 l-15-18 l-15-19 l-15-20 l-15-21 l-15-22 l-15-23 l-16-1 l-16-2 l-16-3 l-16-4 l-16-5 l-16-6 l-16-7 l-16-8 l-16-9 l-16-10 l-16-11 l-16-12 l-16-13 l-16-14 l-16-15 l-16-16 l-16-17 l-16-18 l-16-19 l-16-20 l-16-21 l-16-22 l-16-23 l-17-1 l-17-2 l-17-3 l-17-4 l-17-5 l-17-6 l-17-7 l-17-8 l-17-9 l-17-10 l-17-11 l-17-12 l-17-13 l-17-14 l-17-15 l-17-16 l-17-17 l-17-18 l-17-19 l-17-20 l-17-21 l-17-22 l-17-23 l-18-1 l-18-2 l-18-3 l-18-4 l-18-5 l-18-6 l-18-7 l-18-8 l-18-9 l-18-10 l-18-11 l-18-12 l-18-13 l-18-14 l-18-15 l-18-16 l-18-17 l-18-18 l-18-19 l-18-20 l-18-21 l-18-22 l-18-23 l-19-1 l-19-2 l-19-3 l-19-4 l-19-5 l-19-6 l-19-7 l-19-8 l-19-9 l-19-10 l-19-11 l-19-12 l-19-13 l-19-14 l-19-15 l-19-16 l-19-17 l-19-18 l-19-19 l-19-20 l-19-21 l-19-22 l-19-23 l-20-1 l-20-2 l-20-3 l-20-4 l-20-5 l-20-6 l-20-7 l-20-8 l-20-9 l-20-10 l-20-11 l-20-12 l-20-13 l-20-14 l-20-15 l-20-16 l-20-17 l-20-18 l-20-19 l-20-20 l-20-21 l-20-22 l-20-23 l-21-1 l-21-2 l-21-3 l-21-4 l-21-5 l-21-6 l-21-7 l-21-8 l-21-9 l-21-10 l-21-11 l-21-12 l-21-13 l-21-14 l-21-15 l-21-16 l-21-17 l-21-18 l-21-19 l-21-20 l-21-21 l-21-22 l-21-23 l-22-1 l-22-2 l-22-3 l-22-4 l-22-5 l-22-6 l-22-7 l-22-8 l-22-9 l-22-10 l-22-11 l-22-12 l-22-13 l-22-14 l-22-15 l-22-16 l-22-17 l-22-18 l-22-19 l-22-20 l-22-21 l-22-22 l-22-23 l-23-1 l-23-2 l-23-3 l-23-4 l-23-5 l-23-6 l-23-7 l-23-8 l-23-9 l-23-10 l-23-11 l-23-12 l-23-13 l-23-14 l-23-15 l-23-16 l-23-17 l-23-18 l-23-19 l-23-20 l-23-21 l-23-22 l-23-23 - location)(:init (vehicle-at l-1-1)(road l-1-1 l-1-2)(road l-1-2 l-1-3)(road l-1-3 l-1-4)(road l-1-4 l-1-5)(road l-1-5 l-1-6)(road l-1-6 l-1-7)(road l-1-7 l-1-8)(road l-1-8 l-1-9)(road l-1-9 l-1-10)(road l-1-10 l-1-11)(road l-1-11 l-1-12)(road l-1-12 l-1-13)(road l-1-13 l-1-14)(road l-1-14 l-1-15)(road l-1-15 l-1-16)(road l-1-16 l-1-17)(road l-1-17 l-1-18)(road l-1-18 l-1-19)(road l-1-19 l-1-20)(road l-1-20 l-1-21)(road l-1-21 l-1-22)(road l-1-22 l-1-23)(road l-1-1 l-2-1)(road l-1-2 l-2-2)(road l-1-3 l-2-3)(road l-1-4 l-2-4)(road l-1-5 l-2-5)(road l-1-6 l-2-6)(road l-1-7 l-2-7)(road l-1-8 l-2-8)(road l-1-9 l-2-9)(road l-1-10 l-2-10)(road l-1-11 l-2-11)(road l-1-12 l-2-12)(road l-1-13 l-2-13)(road l-1-14 l-2-14)(road l-1-15 l-2-15)(road l-1-16 l-2-16)(road l-1-17 l-2-17)(road l-1-18 l-2-18)(road l-1-19 l-2-19)(road l-1-20 l-2-20)(road l-1-21 l-2-21)(road l-1-22 l-2-22)(road l-2-1 l-1-2)(road l-2-2 l-1-3)(road l-2-3 l-1-4)(road l-2-4 l-1-5)(road l-2-5 l-1-6)(road l-2-6 l-1-7)(road l-2-7 l-1-8)(road l-2-8 l-1-9)(road l-2-9 l-1-10)(road l-2-10 l-1-11)(road l-2-11 l-1-12)(road l-2-12 l-1-13)(road l-2-13 l-1-14)(road l-2-14 l-1-15)(road l-2-15 l-1-16)(road l-2-16 l-1-17)(road l-2-17 l-1-18)(road l-2-18 l-1-19)(road l-2-19 l-1-20)(road l-2-20 l-1-21)(road l-2-21 l-1-22)(road l-2-22 l-1-23)(spare-in l-2-1)(spare-in l-2-2)(spare-in l-2-3)(spare-in l-2-4)(spare-in l-2-5)(spare-in l-2-6)(spare-in l-2-7)(spare-in l-2-8)(spare-in l-2-9)(spare-in l-2-10)(spare-in l-2-11)(spare-in l-2-12)(spare-in l-2-13)(spare-in l-2-14)(spare-in l-2-15)(spare-in l-2-16)(spare-in l-2-17)(spare-in l-2-18)(spare-in l-2-19)(spare-in l-2-20)(spare-in l-2-21)(spare-in l-2-22)(road l-3-1 l-3-2)(road l-3-2 l-3-3)(road l-3-3 l-3-4)(road l-3-4 l-3-5)(road l-3-5 l-3-6)(road l-3-6 l-3-7)(road l-3-7 l-3-8)(road l-3-8 l-3-9)(road l-3-9 l-3-10)(road l-3-10 l-3-11)(road l-3-11 l-3-12)(road l-3-12 l-3-13)(road l-3-13 l-3-14)(road l-3-14 l-3-15)(road l-3-15 l-3-16)(road l-3-16 l-3-17)(road l-3-17 l-3-18)(road l-3-18 l-3-19)(road l-3-19 l-3-20)(road l-3-20 l-3-21)(road l-2-1 l-3-1)(road l-2-3 l-3-3)(road l-2-5 l-3-5)(road l-2-7 l-3-7)(road l-2-9 l-3-9)(road l-2-11 l-3-11)(road l-2-13 l-3-13)(road l-2-15 l-3-15)(road l-2-17 l-3-17)(road l-2-19 l-3-19)(road l-2-21 l-3-21)(road l-3-1 l-2-2)(road l-3-3 l-2-4)(road l-3-5 l-2-6)(road l-3-7 l-2-8)(road l-3-9 l-2-10)(road l-3-11 l-2-12)(road l-3-13 l-2-14)(road l-3-15 l-2-16)(road l-3-17 l-2-18)(road l-3-19 l-2-20)(road l-3-21 l-2-22)(spare-in l-3-1)(spare-in l-3-21)(road l-3-1 l-4-1)(road l-3-2 l-4-2)(road l-3-3 l-4-3)(road l-3-4 l-4-4)(road l-3-5 l-4-5)(road l-3-6 l-4-6)(road l-3-7 l-4-7)(road l-3-8 l-4-8)(road l-3-9 l-4-9)(road l-3-10 l-4-10)(road l-3-11 l-4-11)(road l-3-12 l-4-12)(road l-3-13 l-4-13)(road l-3-14 l-4-14)(road l-3-15 l-4-15)(road l-3-16 l-4-16)(road l-3-17 l-4-17)(road l-3-18 l-4-18)(road l-3-19 l-4-19)(road l-3-20 l-4-20)(road l-4-1 l-3-2)(road l-4-2 l-3-3)(road l-4-3 l-3-4)(road l-4-4 l-3-5)(road l-4-5 l-3-6)(road l-4-6 l-3-7)(road l-4-7 l-3-8)(road l-4-8 l-3-9)(road l-4-9 l-3-10)(road l-4-10 l-3-11)(road l-4-11 l-3-12)(road l-4-12 l-3-13)(road l-4-13 l-3-14)(road l-4-14 l-3-15)(road l-4-15 l-3-16)(road l-4-16 l-3-17)(road l-4-17 l-3-18)(road l-4-18 l-3-19)(road l-4-19 l-3-20)(road l-4-20 l-3-21)(spare-in l-4-1)(spare-in l-4-2)(spare-in l-4-3)(spare-in l-4-4)(spare-in l-4-5)(spare-in l-4-6)(spare-in l-4-7)(spare-in l-4-8)(spare-in l-4-9)(spare-in l-4-10)(spare-in l-4-11)(spare-in l-4-12)(spare-in l-4-13)(spare-in l-4-14)(spare-in l-4-15)(spare-in l-4-16)(spare-in l-4-17)(spare-in l-4-18)(spare-in l-4-19)(spare-in l-4-20)(road l-5-1 l-5-2)(road l-5-2 l-5-3)(road l-5-3 l-5-4)(road l-5-4 l-5-5)(road l-5-5 l-5-6)(road l-5-6 l-5-7)(road l-5-7 l-5-8)(road l-5-8 l-5-9)(road l-5-9 l-5-10)(road l-5-10 l-5-11)(road l-5-11 l-5-12)(road l-5-12 l-5-13)(road l-5-13 l-5-14)(road l-5-14 l-5-15)(road l-5-15 l-5-16)(road l-5-16 l-5-17)(road l-5-17 l-5-18)(road l-5-18 l-5-19)(road l-4-1 l-5-1)(road l-4-3 l-5-3)(road l-4-5 l-5-5)(road l-4-7 l-5-7)(road l-4-9 l-5-9)(road l-4-11 l-5-11)(road l-4-13 l-5-13)(road l-4-15 l-5-15)(road l-4-17 l-5-17)(road l-4-19 l-5-19)(road l-5-1 l-4-2)(road l-5-3 l-4-4)(road l-5-5 l-4-6)(road l-5-7 l-4-8)(road l-5-9 l-4-10)(road l-5-11 l-4-12)(road l-5-13 l-4-14)(road l-5-15 l-4-16)(road l-5-17 l-4-18)(road l-5-19 l-4-20)(spare-in l-5-1)(spare-in l-5-19)(road l-5-1 l-6-1)(road l-5-2 l-6-2)(road l-5-3 l-6-3)(road l-5-4 l-6-4)(road l-5-5 l-6-5)(road l-5-6 l-6-6)(road l-5-7 l-6-7)(road l-5-8 l-6-8)(road l-5-9 l-6-9)(road l-5-10 l-6-10)(road l-5-11 l-6-11)(road l-5-12 l-6-12)(road l-5-13 l-6-13)(road l-5-14 l-6-14)(road l-5-15 l-6-15)(road l-5-16 l-6-16)(road l-5-17 l-6-17)(road l-5-18 l-6-18)(road l-6-1 l-5-2)(road l-6-2 l-5-3)(road l-6-3 l-5-4)(road l-6-4 l-5-5)(road l-6-5 l-5-6)(road l-6-6 l-5-7)(road l-6-7 l-5-8)(road l-6-8 l-5-9)(road l-6-9 l-5-10)(road l-6-10 l-5-11)(road l-6-11 l-5-12)(road l-6-12 l-5-13)(road l-6-13 l-5-14)(road l-6-14 l-5-15)(road l-6-15 l-5-16)(road l-6-16 l-5-17)(road l-6-17 l-5-18)(road l-6-18 l-5-19)(spare-in l-6-1)(spare-in l-6-2)(spare-in l-6-3)(spare-in l-6-4)(spare-in l-6-5)(spare-in l-6-6)(spare-in l-6-7)(spare-in l-6-8)(spare-in l-6-9)(spare-in l-6-10)(spare-in l-6-11)(spare-in l-6-12)(spare-in l-6-13)(spare-in l-6-14)(spare-in l-6-15)(spare-in l-6-16)(spare-in l-6-17)(spare-in l-6-18)(road l-7-1 l-7-2)(road l-7-2 l-7-3)(road l-7-3 l-7-4)(road l-7-4 l-7-5)(road l-7-5 l-7-6)(road l-7-6 l-7-7)(road l-7-7 l-7-8)(road l-7-8 l-7-9)(road l-7-9 l-7-10)(road l-7-10 l-7-11)(road l-7-11 l-7-12)(road l-7-12 l-7-13)(road l-7-13 l-7-14)(road l-7-14 l-7-15)(road l-7-15 l-7-16)(road l-7-16 l-7-17)(road l-6-1 l-7-1)(road l-6-3 l-7-3)(road l-6-5 l-7-5)(road l-6-7 l-7-7)(road l-6-9 l-7-9)(road l-6-11 l-7-11)(road l-6-13 l-7-13)(road l-6-15 l-7-15)(road l-6-17 l-7-17)(road l-7-1 l-6-2)(road l-7-3 l-6-4)(road l-7-5 l-6-6)(road l-7-7 l-6-8)(road l-7-9 l-6-10)(road l-7-11 l-6-12)(road l-7-13 l-6-14)(road l-7-15 l-6-16)(road l-7-17 l-6-18)(spare-in l-7-1)(spare-in l-7-17)(road l-7-1 l-8-1)(road l-7-2 l-8-2)(road l-7-3 l-8-3)(road l-7-4 l-8-4)(road l-7-5 l-8-5)(road l-7-6 l-8-6)(road l-7-7 l-8-7)(road l-7-8 l-8-8)(road l-7-9 l-8-9)(road l-7-10 l-8-10)(road l-7-11 l-8-11)(road l-7-12 l-8-12)(road l-7-13 l-8-13)(road l-7-14 l-8-14)(road l-7-15 l-8-15)(road l-7-16 l-8-16)(road l-8-1 l-7-2)(road l-8-2 l-7-3)(road l-8-3 l-7-4)(road l-8-4 l-7-5)(road l-8-5 l-7-6)(road l-8-6 l-7-7)(road l-8-7 l-7-8)(road l-8-8 l-7-9)(road l-8-9 l-7-10)(road l-8-10 l-7-11)(road l-8-11 l-7-12)(road l-8-12 l-7-13)(road l-8-13 l-7-14)(road l-8-14 l-7-15)(road l-8-15 l-7-16)(road l-8-16 l-7-17)(spare-in l-8-1)(spare-in l-8-2)(spare-in l-8-3)(spare-in l-8-4)(spare-in l-8-5)(spare-in l-8-6)(spare-in l-8-7)(spare-in l-8-8)(spare-in l-8-9)(spare-in l-8-10)(spare-in l-8-11)(spare-in l-8-12)(spare-in l-8-13)(spare-in l-8-14)(spare-in l-8-15)(spare-in l-8-16)(road l-9-1 l-9-2)(road l-9-2 l-9-3)(road l-9-3 l-9-4)(road l-9-4 l-9-5)(road l-9-5 l-9-6)(road l-9-6 l-9-7)(road l-9-7 l-9-8)(road l-9-8 l-9-9)(road l-9-9 l-9-10)(road l-9-10 l-9-11)(road l-9-11 l-9-12)(road l-9-12 l-9-13)(road l-9-13 l-9-14)(road l-9-14 l-9-15)(road l-8-1 l-9-1)(road l-8-3 l-9-3)(road l-8-5 l-9-5)(road l-8-7 l-9-7)(road l-8-9 l-9-9)(road l-8-11 l-9-11)(road l-8-13 l-9-13)(road l-8-15 l-9-15)(road l-9-1 l-8-2)(road l-9-3 l-8-4)(road l-9-5 l-8-6)(road l-9-7 l-8-8)(road l-9-9 l-8-10)(road l-9-11 l-8-12)(road l-9-13 l-8-14)(road l-9-15 l-8-16)(spare-in l-9-1)(spare-in l-9-15)(road l-9-1 l-10-1)(road l-9-2 l-10-2)(road l-9-3 l-10-3)(road l-9-4 l-10-4)(road l-9-5 l-10-5)(road l-9-6 l-10-6)(road l-9-7 l-10-7)(road l-9-8 l-10-8)(road l-9-9 l-10-9)(road l-9-10 l-10-10)(road l-9-11 l-10-11)(road l-9-12 l-10-12)(road l-9-13 l-10-13)(road l-9-14 l-10-14)(road l-10-1 l-9-2)(road l-10-2 l-9-3)(road l-10-3 l-9-4)(road l-10-4 l-9-5)(road l-10-5 l-9-6)(road l-10-6 l-9-7)(road l-10-7 l-9-8)(road l-10-8 l-9-9)(road l-10-9 l-9-10)(road l-10-10 l-9-11)(road l-10-11 l-9-12)(road l-10-12 l-9-13)(road l-10-13 l-9-14)(road l-10-14 l-9-15)(spare-in l-10-1)(spare-in l-10-2)(spare-in l-10-3)(spare-in l-10-4)(spare-in l-10-5)(spare-in l-10-6)(spare-in l-10-7)(spare-in l-10-8)(spare-in l-10-9)(spare-in l-10-10)(spare-in l-10-11)(spare-in l-10-12)(spare-in l-10-13)(spare-in l-10-14)(road l-11-1 l-11-2)(road l-11-2 l-11-3)(road l-11-3 l-11-4)(road l-11-4 l-11-5)(road l-11-5 l-11-6)(road l-11-6 l-11-7)(road l-11-7 l-11-8)(road l-11-8 l-11-9)(road l-11-9 l-11-10)(road l-11-10 l-11-11)(road l-11-11 l-11-12)(road l-11-12 l-11-13)(road l-10-1 l-11-1)(road l-10-3 l-11-3)(road l-10-5 l-11-5)(road l-10-7 l-11-7)(road l-10-9 l-11-9)(road l-10-11 l-11-11)(road l-10-13 l-11-13)(road l-11-1 l-10-2)(road l-11-3 l-10-4)(road l-11-5 l-10-6)(road l-11-7 l-10-8)(road l-11-9 l-10-10)(road l-11-11 l-10-12)(road l-11-13 l-10-14)(spare-in l-11-1)(spare-in l-11-13)(road l-11-1 l-12-1)(road l-11-2 l-12-2)(road l-11-3 l-12-3)(road l-11-4 l-12-4)(road l-11-5 l-12-5)(road l-11-6 l-12-6)(road l-11-7 l-12-7)(road l-11-8 l-12-8)(road l-11-9 l-12-9)(road l-11-10 l-12-10)(road l-11-11 l-12-11)(road l-11-12 l-12-12)(road l-12-1 l-11-2)(road l-12-2 l-11-3)(road l-12-3 l-11-4)(road l-12-4 l-11-5)(road l-12-5 l-11-6)(road l-12-6 l-11-7)(road l-12-7 l-11-8)(road l-12-8 l-11-9)(road l-12-9 l-11-10)(road l-12-10 l-11-11)(road l-12-11 l-11-12)(road l-12-12 l-11-13)(spare-in l-12-1)(spare-in l-12-2)(spare-in l-12-3)(spare-in l-12-4)(spare-in l-12-5)(spare-in l-12-6)(spare-in l-12-7)(spare-in l-12-8)(spare-in l-12-9)(spare-in l-12-10)(spare-in l-12-11)(spare-in l-12-12)(road l-13-1 l-13-2)(road l-13-2 l-13-3)(road l-13-3 l-13-4)(road l-13-4 l-13-5)(road l-13-5 l-13-6)(road l-13-6 l-13-7)(road l-13-7 l-13-8)(road l-13-8 l-13-9)(road l-13-9 l-13-10)(road l-13-10 l-13-11)(road l-12-1 l-13-1)(road l-12-3 l-13-3)(road l-12-5 l-13-5)(road l-12-7 l-13-7)(road l-12-9 l-13-9)(road l-12-11 l-13-11)(road l-13-1 l-12-2)(road l-13-3 l-12-4)(road l-13-5 l-12-6)(road l-13-7 l-12-8)(road l-13-9 l-12-10)(road l-13-11 l-12-12)(spare-in l-13-1)(spare-in l-13-11)(road l-13-1 l-14-1)(road l-13-2 l-14-2)(road l-13-3 l-14-3)(road l-13-4 l-14-4)(road l-13-5 l-14-5)(road l-13-6 l-14-6)(road l-13-7 l-14-7)(road l-13-8 l-14-8)(road l-13-9 l-14-9)(road l-13-10 l-14-10)(road l-14-1 l-13-2)(road l-14-2 l-13-3)(road l-14-3 l-13-4)(road l-14-4 l-13-5)(road l-14-5 l-13-6)(road l-14-6 l-13-7)(road l-14-7 l-13-8)(road l-14-8 l-13-9)(road l-14-9 l-13-10)(road l-14-10 l-13-11)(spare-in l-14-1)(spare-in l-14-2)(spare-in l-14-3)(spare-in l-14-4)(spare-in l-14-5)(spare-in l-14-6)(spare-in l-14-7)(spare-in l-14-8)(spare-in l-14-9)(spare-in l-14-10)(road l-15-1 l-15-2)(road l-15-2 l-15-3)(road l-15-3 l-15-4)(road l-15-4 l-15-5)(road l-15-5 l-15-6)(road l-15-6 l-15-7)(road l-15-7 l-15-8)(road l-15-8 l-15-9)(road l-14-1 l-15-1)(road l-14-3 l-15-3)(road l-14-5 l-15-5)(road l-14-7 l-15-7)(road l-14-9 l-15-9)(road l-15-1 l-14-2)(road l-15-3 l-14-4)(road l-15-5 l-14-6)(road l-15-7 l-14-8)(road l-15-9 l-14-10)(spare-in l-15-1)(spare-in l-15-9)(road l-15-1 l-16-1)(road l-15-2 l-16-2)(road l-15-3 l-16-3)(road l-15-4 l-16-4)(road l-15-5 l-16-5)(road l-15-6 l-16-6)(road l-15-7 l-16-7)(road l-15-8 l-16-8)(road l-16-1 l-15-2)(road l-16-2 l-15-3)(road l-16-3 l-15-4)(road l-16-4 l-15-5)(road l-16-5 l-15-6)(road l-16-6 l-15-7)(road l-16-7 l-15-8)(road l-16-8 l-15-9)(spare-in l-16-1)(spare-in l-16-2)(spare-in l-16-3)(spare-in l-16-4)(spare-in l-16-5)(spare-in l-16-6)(spare-in l-16-7)(spare-in l-16-8)(road l-17-1 l-17-2)(road l-17-2 l-17-3)(road l-17-3 l-17-4)(road l-17-4 l-17-5)(road l-17-5 l-17-6)(road l-17-6 l-17-7)(road l-16-1 l-17-1)(road l-16-3 l-17-3)(road l-16-5 l-17-5)(road l-16-7 l-17-7)(road l-17-1 l-16-2)(road l-17-3 l-16-4)(road l-17-5 l-16-6)(road l-17-7 l-16-8)(spare-in l-17-1)(spare-in l-17-7)(road l-17-1 l-18-1)(road l-17-2 l-18-2)(road l-17-3 l-18-3)(road l-17-4 l-18-4)(road l-17-5 l-18-5)(road l-17-6 l-18-6)(road l-18-1 l-17-2)(road l-18-2 l-17-3)(road l-18-3 l-17-4)(road l-18-4 l-17-5)(road l-18-5 l-17-6)(road l-18-6 l-17-7)(spare-in l-18-1)(spare-in l-18-2)(spare-in l-18-3)(spare-in l-18-4)(spare-in l-18-5)(spare-in l-18-6)(road l-19-1 l-19-2)(road l-19-2 l-19-3)(road l-19-3 l-19-4)(road l-19-4 l-19-5)(road l-18-1 l-19-1)(road l-18-3 l-19-3)(road l-18-5 l-19-5)(road l-19-1 l-18-2)(road l-19-3 l-18-4)(road l-19-5 l-18-6)(spare-in l-19-1)(spare-in l-19-5)(road l-19-1 l-20-1)(road l-19-2 l-20-2)(road l-19-3 l-20-3)(road l-19-4 l-20-4)(road l-20-1 l-19-2)(road l-20-2 l-19-3)(road l-20-3 l-19-4)(road l-20-4 l-19-5)(spare-in l-20-1)(spare-in l-20-2)(spare-in l-20-3)(spare-in l-20-4)(road l-21-1 l-21-2)(road l-21-2 l-21-3)(road l-20-1 l-21-1)(road l-20-3 l-21-3)(road l-21-1 l-20-2)(road l-21-3 l-20-4)(spare-in l-21-1)(spare-in l-21-3)(road l-21-1 l-22-1)(road l-21-2 l-22-2)(road l-22-1 l-21-2)(road l-22-2 l-21-3)(spare-in l-22-1)(spare-in l-22-2)(road l-22-1 l-23-1)(road l-23-1 l-22-2)(spare-in l-23-1)(not-flattire))(:goal (vehicle-at l-1-23)))
