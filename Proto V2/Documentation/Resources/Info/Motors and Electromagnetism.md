---
tags:
  - "#note"
created: 2024-10-02
authors:
  - Jacobus Burger
---
The motors used are **350W 36V BLDC** motors taken from a hoverboard.

BLDC is short for BrushLess Direct Current. It is a contactless motor type which uses the flow of DC current in electromagnetic coils to generate a magnetic force, which then pushes and pulls permanent magnets inside the rotating part of the motor (the stator) to generate movement.

<iframe title="Brushless DC Motor, How it works ?" src="https://www.youtube.com/embed/bCEiOnuODac?feature=oembed" height="113" width="200" style="aspect-ratio: 1.76991 / 1; width: 100%; height: 100%;" allowfullscreen="" allow="fullscreen"></iframe>

The phenomenon central to the operation of electromagnetic motors like this is [inductance](https://en.wikipedia.org/wiki/Inductance). The phenomenon is a two-way street, so interestingly, just like _supplying_ current to the motor induces movement, moving the motors _generates_ a supply of current. Effectively, **every (electromagnetic) motor is a generator, and vice-versa**.