# Hexapod Mk1: Engineering Design Notebook

*Project with Oli. The real hexapod behind the Robot Sim digital twin.*

This notebook records the end-to-end design process in order: requirements, concepts, analysis, decisions, verification and open questions. It is compiled from the dated entries in `docs/notebook/entries/`. The tables marked *(generated)* are rebuilt from the current CAD outputs every time the notebook is compiled, so they always match the files in the repo.

Compile it:

```
python robot-sim/docs/notebook/build_notebook.py
```

This writes `robot-sim/docs/notebook/Hexapod_Mk1_Design_Notebook.html`, a single file with every image embedded. Print it to PDF from a browser if you need a PDF.

To refresh the CAD snapshots first, run `save_snapshots(<folder>)` from the generator inside Fusion (see entry 11).

**Revisions.**
- **Rev A:** entries 01–04.
- **Rev B** (bus servos, rechargeable power, real vendor CAD, harness): entries 05–12.
- **Rev B2** (leg study, onboard control and sensors, radio, brain module, full wiring, MuJoCo): entries 13–17.
- **Open items:** entry 18.
