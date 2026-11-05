// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// The commands of SWDRAW_ShapeTool.cxx (unit 6L): <c>anaedges</c>, <c>expwire</c>, <c>ssolid</c> and
/// <c>samerange</c>; and SWDRAW_ShapeExtend.cxx's <c>sortcompound</c>.
/// </summary>
/// <remarks>
/// <c>ssolid</c> sets the Free flag of a shell that isn't free before adding it to the solid; a frozen shell can't be
/// changed, and BRep_Builder adds a frozen shape whatever its flag, so Draw-lite prints OCCT's message
/// and adds it as it is. <c>samerange</c> on a shape works in place as OCCT's does, on an unfrozen copy rebound
/// afterwards (<see cref="SWDRAWShapeFixCommands.InPlace"/>).
/// </remarks>
internal static class SWDRAWShapeToolCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "anaedges", XSHAPE_edge);
        DrawCxx.Add(theInterpreter, "expwire", XSHAPE_explorewire);
        DrawCxx.Add(theInterpreter, "ssolid", XSHAPE_ssolid);
        DrawCxx.Add(theInterpreter, "samerange", SameRange);
        DrawCxx.Add(theInterpreter, "sortcompound", SortCompound);
    }

    private static int XSHAPE_edge(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 2)
        {
            di.Append("donner un nom de shape\n");
            return 1 /* Error */;
        }
        string arg1 = argv[1];
        Shape Shape = theDI.GetShape(arg1);
        if (Shape.IsNull)
        {
            di.Append(arg1).Append(" inconnu\n");
            return 1 /* Error */;
        }
        int nbe = 0, nbf = 0;

        for (var exp = new Explorer(Shape, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge Edge = ShapeCasts.Edge(exp.Current());
            nbe++;
            if (Tool.Degenerated(Edge))
            {
                continue;
            }
            Curve3D? curve3d = Tool.Curve(Edge, out _, out _);
            if (curve3d is null)
            {
                nbf++;
                di.Append("Edge sans Curve3d, n0 ").Append(nbe).Append("\n");
                theDI.SetShape($"faultedge_{nbf}", Edge);
            }
        }
        return 0;
    }

    private static int XSHAPE_explorewire(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 2)
        {
            di.Append("donner un nom de wire\n");
            return 1 /* Error */;
        }
        string arg1 = argv[1];
        Shape Shape = theDI.GetShape(arg1);
        if (Shape.IsNull)
        {
            di.Append(arg1).Append(" inconnu\n");
            return 1 /* Error */;
        }
        if (Shape.Type != ShapeType.Wire)
        {
            di.Append("Pas un WIRE\n");
            return 1 /* Error */;
        }
        Wire W = ShapeCasts.Wire(Shape);
        Face F = new();
        if (argc > 2)
        {
            Shape aLocalShape = theDI.GetShape(argv[2]);
            F = ShapeCasts.Face(aLocalShape);
        }

        int i, num = 0, nbw, nbe = 0;
        var map = new IndexedSet<Shape>(ShapeHasher.Instance);
        for (var ext = new Iterator(W); ext.More(); ext.Next())
        {
            if (ext.Value().Type != ShapeType.Edge)
            {
                continue;
            }
            Edge E = ShapeCasts.Edge(ext.Value());
            nbe++;
            num = map.Add(E) + 1;
        }
        int[] nbs = new int[nbe + 1];

        di.Append("TopoDS_Iterator(EDGE)  donne ").Append(nbe).Append(" Edges dont ").Append(num).Append(" distinctes\n");
        nbe = num;
        nbw = 0;
        for (var exe = new Explorer(W.Oriented(Orientation.Forward), ShapeType.Edge); exe.More(); exe.Next())
        {
            nbw++;
        }
        di.Append("TopExp_Explorer(EDGE)  donne ").Append(nbw).Append(" Edges\n");
        nbw = 0;
        var bwe = new WireExplorer();
        if (F.IsNull)
        {
            bwe.Init(W);
        }
        else
        {
            bwe.Init(W, F);
        }
        for (; bwe.More(); bwe.Next())
        {
            Edge E = ShapeCasts.Edge(bwe.Current());
            nbw++;
            num = map.IndexOf(E) + 1;
            nbs[num]++;
        }
        di.Append("BRepTools_WireExplorer donne ").Append(nbw).Append(" Edges\n");
        di.Append("Par rapport a la map, edges sautees par WE en NOWE_num, passees > 1 fois en MULTWE_num\n");
        if (nbs[0] > 0)
        {
            di.Append("NB : Edge n0 0 comptee ").Append(nbs[0]).Append(" fois\n");
        }
        for (i = 1; i <= nbe; i++)
        {
            if (nbs[i] < 1)
            {
                di.Append("Edge n0 ").Append(i).Append(" pas vue par WE\n");
                theDI.SetShape($"NOWE_{i}", map[i - 1]);
            }
            else if (nbs[i] > 1)
            {
                di.Append("Edge n0 ").Append(i).Append(" vue par WE : ").Append(nbs[i]).Append(" fois\n");
                theDI.SetShape($"MULT_{i}", map[i - 1]);
            }
        }
        return 0;
    }

    private static int XSHAPE_ssolid(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 3)
        {
            di.Append("Give new solid name + shell name\n");
            return 1 /* Error */;
        }
        string arg1 = argv[1];
        Shape Shape = theDI.GetShape(arg1);
        if (Shape.IsNull)
        {
            di.Append("Shape unknown : ").Append(arg1).Append("\n");
            return 1 /* Error */;
        }
        ShapeType shen = Shape.Type;
        if (shen == ShapeType.Solid)
        {
            di.Append(" Already a Solide ! nothing done\n");
            return 0;
        }
        if (shen != ShapeType.Shell)
        {
            di.Append(" Not a Shell\n");
            return 1 /* Error */;
        }
        if (!Shape.Free())
        {
            di.Append("Shape non Free -> Freeing\n");
            // Kerf: a frozen shell keeps its flag, and BRep_Builder adds it all the same (see the class)
            if (!Shape.TShape()!.IsFrozen)
            {
                Shape.Free(true);
            }
        }
        Shell sh = ShapeCasts.Shell(Shape);
        var B = new BrepBuilder();
        var solid = new Solid();
        B.MakeSolid(ref solid);
        B.Add(solid, sh);
        //   Pas encore fini : il faut une bonne orientation
        var bsc3d = new SolidClassifier(solid);
        bsc3d.PerformInfinitePoint(BRepBuilderApi.Precision());
        if (bsc3d.State() == Containment.In)
        {
            //         Ensuite, inverser C-A-D REPRENDRE LES SHELLS
            //         (l inversion du solide n est pas bien prise en compte)
            di.Append("NB : Shell to be reversed\n");
            var soli2 = new Solid();
            B.MakeSolid(ref soli2); // on recommence
            sh = ShapeCasts.Shell(sh.Reversed());
            B.Add(soli2, sh);
            solid = soli2;
        }
        theDI.SetShape(argv[2], solid);
        return 0; // Done
    }

    private static int SameRange(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc == 2)
        {
            Shape Shape = theDI.GetShape(argv[1]);
            if (Shape.IsNull)
            {
                // OCCT prints argv[2], past the words (a null pointer); Draw-lite prints nothing there
                di.Append("Shape unknown: ").Append("\n");
                return 1;
            }

            // OCCT works on the shape itself: Draw-lite on an unfrozen copy, rebound afterwards
            SWDRAWShapeFixCommands.InPlace(theDI, Shape, aWork =>
            {
                for (var exp = new Explorer(aWork, ShapeType.Edge); exp.More(); exp.Next())
                {
                    Edge edge = ShapeCasts.Edge(exp.Current());
                    BrepLibrary.SameRange(edge, global::Kerf.Engine.Foundation.Precision.PConfusion());
                }
                return aWork;
            });
        }
        else if (argc == 7)
        {
            Curve2D? C = DrawTrSurf.GetCurve2d(theDI, argv[2]);
            if (C is null)
            {
                di.Append("Curve unknown: ").Append(argv[2]).Append("\n");
                return 1;
            }

            double oldFirst = theDI.Atof(argv[3]);
            double oldLast = theDI.Atof(argv[4]);
            double current_first = theDI.Atof(argv[5]);
            double current_last = theDI.Atof(argv[6]);
            double Tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
            Curve2D? NewC2d = null;
            global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(Tol, C, oldFirst, oldLast, current_first, current_last, ref NewC2d);
            DrawTrSurf.Set(theDI, argv[1], NewC2d);
        }
        else
        {
            di.Append("Apply BRepLib::SameRange() to shape or GeomLib::SameRange() to pcurve:\n");
            di.Append("> samerange shape\n");
            di.Append("or\n");
            di.Append("> samerange newcurve curve2d first last newfirst newlast\n");
        }

        return 0;
    }

    private static int SortCompound(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 2)
        {
            di.Append("Give shape name. As is, just displays shape type\n");
            di.Append(" or add result_name type [mode]\n");
            di.Append(" Type : v (vertex)  e (edge)  w (wire)  f (face)  s (shell)  so (solid)\n");
            di.Append(" Mode : n (normal, by default)  e (explore)  c (compound)  x (explore+comp)\n");
            return 0;
        }
        string arg1 = argv[1];
        Shape Shape = theDI.GetShape(arg1);
        if (Shape.IsNull)
        {
            di.Append("Shape unknown : ").Append(arg1).Append("\n");
            return 1 /* Error */;
        }

        if (argc < 4)
        {
            var sbx = new ExtendExplorer();
            for (int i = 0; i < 2; i++)
            {
                bool viaCompound = i != 0;
                ShapeType ts = sbx.ShapeType(Shape, viaCompound);
                if (!viaCompound)
                {
                    di.Append("   Type (True) : ");
                }
                else
                {
                    di.Append("   Type  (via Compound) : ");
                }
                switch (ts)
                {
                    case ShapeType.Shape:
                        di.Append("SHAPE (empty)");
                        break;
                    case ShapeType.Vertex:
                        di.Append("VERTEX");
                        break;
                    case ShapeType.Edge:
                        di.Append("EDGE");
                        break;
                    case ShapeType.Wire:
                        di.Append("WIRE");
                        break;
                    case ShapeType.Face:
                        di.Append("FACE");
                        break;
                    case ShapeType.Shell:
                        di.Append("SHELL");
                        break;
                    case ShapeType.Solid:
                        di.Append("SOLID");
                        break;
                    case ShapeType.CompSolid:
                        di.Append("COMPSOLID");
                        break;
                    case ShapeType.Compound:
                        di.Append("COMPOUND");
                        if (i > 0)
                        {
                            di.Append(" (heterogeneous)");
                        }
                        break;
                }
            }
            di.Append("\n");
            return 0;
        }

        ShapeType tse = ShapeType.Compound;
        string aType = argv[3];
        char tsc = aType.Length > 0 ? aType[0] : '\0';
        switch (tsc)
        {
            case 'v':
                tse = ShapeType.Vertex;
                break;
            case 'e':
                tse = ShapeType.Edge;
                break;
            case 'w':
                tse = ShapeType.Wire;
                break;
            case 'f':
                tse = ShapeType.Face;
                break;
            case 's':
                tse = aType.Length > 1 && aType[1] == 'o' ? ShapeType.Solid : ShapeType.Shell;
                break;
        }
        bool modexp = false;
        bool modcom = false;
        if (argc > 4)
        {
            switch (argv[4].Length > 0 ? argv[4][0] : '\0')
            {
                case 'e':
                    modexp = true;
                    break;
                case 'c':
                    modcom = true;
                    break;
                case 'x':
                    modexp = modcom = true;
                    break;
            }
            var sbx = new ExtendExplorer();
            Shape = sbx.SortedCompound(Shape, tse, modexp, modcom);
            if (Shape.IsNull)
            {
                di.Append("Empty result\n");
                return 0;
            }
            theDI.SetShape(argv[2], Shape);
        }
        return 0; // Done
    }
}
