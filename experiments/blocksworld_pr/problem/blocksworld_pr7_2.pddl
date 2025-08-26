(define (problem blocksworld_pr7_2)
  (:domain blocksworld-original)
  (:objects
    blue yellow red white magenta grey green
  )
  (:init
    (arm-empty)
    (on-table blue)
    (on yellow blue)
    (on red yellow)
    (on white red)
    (clear white)
    (on-table magenta)
    (on grey magenta)
    (on green grey)
    (clear green)
  )
  (:goal
    (and
      (on-table green)
      (on white green)
      (on grey white)
      (on red grey)
      (on-table yellow)
      (on magenta yellow)
      (on blue magenta)
    )
  )
)