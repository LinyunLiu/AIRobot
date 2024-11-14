---
tags:
  - "#note"
  - archive
created: 2024-09-20
authors:
  - Jacobus Burger
---
[Proto V1](Archive/Proto%20V1/Proto%20V1.md) utilized a 3-level voltage system (because voltage needs weren't well-defined beforehand, in the future it could all be 1 voltage level for battery supply, then convert voltage up or down for different devices).
The first voltage level was **36V DC**. This was for the hoverboard motors which require 250W at 36V of power. This was stepped up and down by a 36V<->12V step up/down buck-boost converter.
The second voltage level was **12V DC**. This ended up functioning as a common voltage for in-between use. This would then be converted with a 12V->5V5A step down converter that supplies power to the RPI4.
Between the Two batteries (36V 4Ah LiPO and 12V 8Ah LiFePO), the system has a capacity of 12Ah.


![Proto V1 Power Supply Diagram](Archive/Proto%20V1/2024-06-26%20power%20plan%20(Jacobus).jpg)
_Power Supply Diagram: Notice that 2 batteries and 3 voltage levels are used and converted between._

