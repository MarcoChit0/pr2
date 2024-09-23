; MAPF/DU corridor problem (FOND version) with 2 agents and 8 nodes
; Author: Thorsten Engesser
(define (problem mapfdu_2_8)
  (:domain mapfdu_2_agents)
  (:objects c00 c01 c02 c03 c04 c05 c06 x03 - pos)
  (:init (adj c00 c01)
         (adj c01 c00) (adj c01 c02)
         (adj c02 c01) (adj c02 c03)
         (adj c03 c02) (adj c03 c04) (adj c03 x03)
         (adj c04 c03) (adj c04 c05)
         (adj c05 c04) (adj c05 c06)
         (adj c06 c05)
         (adj x03 c03)
         (at a0 c06) (at a1 c03)
         (ind a0 w0 w2) (ind a0 w1 w3) (ind a0 w2 w0) (ind a0 w3 w1)
         (ind a1 w0 w1) (ind a1 w1 w0) (ind a1 w2 w3) (ind a1 w3 w2)
         (goal w0 a0 c00) (goal w0 a1 c03)
         (goal w1 a0 x03) (goal w1 a1 c03)
         (goal w2 a0 c00) (goal w2 a1 c06)
         (goal w3 a0 x03) (goal w3 a1 c06)
         (des w0) (next-choose))
  (:goal (and (stopped a0) (stopped a1))))