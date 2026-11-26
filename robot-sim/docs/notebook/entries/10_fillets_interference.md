# 10 · Fillets, chamfers and interference
*2026-11-25*

Per the user's request, the generator now applies four finishes automatically, before any hole or pocket is cut:

- 3 mm fillets on the main load-path inside corners;
- 2 mm on every other inside corner (pad and boss roots, pillar roots);
- 0.8 mm edge breaks on the outside edges of each part's top face;
- 0.5 mm elephant-foot chamfers on bed-side edges.

Sliding fits stay sharp: the dovetail slot and the jigsaw keys.

**Interference those fillets caused, and the fixes:**
- 1.5 mm rail-root fillets against the receiver lip: rail fillet reduced to 0.5 mm, inside the lip's 0.8 mm round.
- Fillets inside the dovetail slot: that zone is excluded from the fillet pass.

{{fillets}}
