# 09 · Rev B mechanical redesign
*2026-11-25*

**The servo sandwich.** Every servo is clamped between two 5 mm printed faces sitting on its real case faces, with pockets for the label boss and pad and 2 × M2 into the case on each side. As a result:

- the plate gap is set by the servo (30.4 mm);
- the body origin is the coxa-joint height;
- the plates sit at +13.8…+18.8 and −21.6…−16.6 mm.

**Two-sided joints.** The STS servos have a driven horn on one side and an idler disc on the other, so every bracket bolts to both through pads. The joint is supported on both sides by the servo's own bearings, and the rev A 623 bearings were removed.

**Links.**
- **Coxa:** CoxaMain (two horn arms, web, + clamp plate) and CoxaCap (− clamp plate). It's split because a one-piece U with both clamp plates can't print without supports.
- **Femur:** two arms with bosses that clamp the tibia servo, and a bridge.
- **Tibia:** its arms moved *outboard* of the femur arms. With both pairs in the same planes, the tibia could only fold 30°; outboard arms plus a chamfered bridge give 45°.

**Body.**
- The plate edge sits 11.5 mm inside each coxa axis.
- Integral pillars (no standoffs).
- The battery bay was moved rearward to make room for a front power strip.
- Everything now fits a 180 mm bed.
