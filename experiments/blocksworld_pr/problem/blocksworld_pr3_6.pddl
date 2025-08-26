(define (problem blocksworld_pr3_5)
  (:domain blocksworld-original)
  (:objects
    brown blue green
  )
  (:init
    (arm-empty)
    (on-table brown)
    (on blue brown)
    (clear blue)
    (on-table green)
    (clear green)
  )
  (:goal
    (and
      (on-table blue)
      (on green blue)
      (on-table brown)
    )
  )
)