"""Slim the official Raspberry Pi 5 STEP (77 MB, stalls Fusion's importer) into a
4 MB model that still looks and fits like a Pi 5.

    python slim_pi5.py rpi5.step rpi5_slim.step 2.5 12      (needs: pip install cadquery-ocp)

- drops every solid smaller than 2.5 mm (about 2,500 SMD parts)
- the PCB becomes a plain 85 x 56 board with the four 2.7 mm mounting holes
  (58 x 49 mm pattern, 3.5 mm in from the corner, per the official mechanical drawing)
- any part with more than 12 faces (USB / Ethernet stacks, headers, chips...) becomes
  its exact bounding box; simpler parts are kept as they are
Same frame as the original, so the generator places it identically."""
import sys
from OCP.STEPControl import STEPControl_Reader, STEPControl_Writer, STEPControl_AsIs
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Compound, TopoDS
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
MAX_FACES = int(sys.argv[4]) if len(sys.argv) > 4 else 40   # more detailed solids become their bounding box

src, dst = sys.argv[1], sys.argv[2]
MIN_SIZE = float(sys.argv[3]) if len(sys.argv) > 3 else 2.5   # mm, largest bbox side
r = STEPControl_Reader()
assert r.ReadFile(src) == IFSelect_RetDone
r.TransferRoots()
shape = r.OneShape()

def nfaces(s):
    e = TopExp_Explorer(s, TopAbs_FACE); n = 0
    while e.More(): n += 1; e.Next()
    return n

b = BRep_Builder(); comp = TopoDS_Compound(); b.MakeCompound(comp)
e = TopExp_Explorer(shape, TopAbs_SOLID)
kept = dropped = fk = ft = 0
boxed_n = [0]
while e.More():
    s = e.Current()
    bb = Bnd_Box(); BRepBndLib.Add_s(s, bb)
    mn, mx = bb.CornerMin(), bb.CornerMax(); x0, y0, z0, x1, y1, z1 = mn.X(), mn.Y(), mn.Z(), mx.X(), mx.Y(), mx.Z()
    size = max(x1 - x0, y1 - y0, z1 - z0)
    f = nfaces(s); ft += f
    if size >= MIN_SIZE:
        if x1 - x0 > 80 and y1 - y0 > 50 and z1 - z0 < 3:               # the PCB: plain board + 4 x 2.7 mm holes
            board = BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), gp_Pnt(x1, y1, z1)).Shape()
            for hx, hy in ((3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5)):
                cyl = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x0 + hx, y0 + hy, z0 - 1), gp_Dir(0, 0, 1)), 1.35, z1 - z0 + 2).Shape()
                board = BRepAlgoAPI_Cut(board, cyl).Shape()
            s = board; f = 10; print('PCB', round(x1 - x0, 2), round(y1 - y0, 2), round(z0, 2), round(z1, 2))
        elif f > MAX_FACES:
            s = BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), gp_Pnt(x1, y1, z1)).Shape(); f = 6; boxed_n[0] += 1
        b.Add(comp, s); kept += 1; fk += f
    else:
        dropped += 1
    e.Next()
w = STEPControl_Writer()
w.Transfer(comp, STEPControl_AsIs)
assert w.Write(dst) == IFSelect_RetDone
bb = Bnd_Box(); BRepBndLib.Add_s(comp, bb)
print('boxed %d;' % boxed_n[0], end=' ')
print('solids kept %d dropped %d; faces kept %d of %d; bbox %s' % (kept, dropped, fk, ft, [round(v, 2) for v in (bb.CornerMin().X(), bb.CornerMin().Y(), bb.CornerMin().Z(), bb.CornerMax().X(), bb.CornerMax().Y(), bb.CornerMax().Z())]))
