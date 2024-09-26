---
tags:
  - "#note"
created: 2024-09-26
authors:
  - Jacobus Burger
---
```table-of-contents
```

# Charging Protocol
The charging method for the system is a bit weird and wonky, so bear that in mind.
Protocol is:
- disconnect 36v battery from everything else (isolate it)
- connect hoverboard charger to hoverboard main-board in back right of robot
- connect 36v battery to main-board
- the hoverboard charger light should turn red if charging (if it's off, keep trying to reconnect and disconnect the battery and main-board until it turns red or green)
- charging is done when it turns green
