(define (problem blocksworld_pr7_1)
  (:domain blocksworld-original)
  (:objects
    yellow blue cyan white green red brown
  )
  (:init
    (arm-empty)
    (on-table yellow)
    (on blue yellow)
    (on cyan blue)
    (on white cyan)
    (clear white)
    (on-table green)
    (on red green)
    (on brown red)
    (clear brown)
  )
  (:goal
    (and
      (on-table cyan)
      (on yellow cyan)
      (on green yellow)
      (on blue green)
      (on red blue)
      (on white red)
      (on-table brown)
    )
  )
)