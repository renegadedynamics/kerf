// Port of: src/Draw/TKTopTest/BRepTest/BRepTest_OtherCommands.cxx (shape, subshape, BRepIntCS, xbounds, xclassify)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace OcctNet.DrawLite;

/// <summary>
/// BRepTest_OtherCommands.cxx's commands over ported packages (unit 7W): <c>shape</c> (an empty shape),
/// <c>subshape</c>, <c>BRepIntCS</c> (BRepIntCurveSurface_Inter), <c>xbounds</c> (BRepTools::UVBounds) and
/// <c>xclassify</c> (BRepClass3d_SolidClassifier's infinite point).
/// </summary>
/// <remarks>
/// <c>makeboss</c> and <c>mksh</c> need LocOpe and BRepFeat (phase 7) and aren't registered. An empty shape is bound as
/// it is; <c>add</c> then binds a new shape with the sub-shape added (decision 272).
/// </remarks>
internal static class BRepTestOtherCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "shape", EmptyShape);
        DrawCxx.Add(theInterpreter, "subshape", SubShape);
        DrawCxx.Add(theInterpreter, "BRepIntCS", BRepIntCS);
        DrawCxx.Add(theInterpreter, "xbounds", XBounds);
        DrawCxx.Add(theInterpreter, "xclassify", XClassify);
    }

    // shape name V/E/W/F/SH/SO/CS/C
    private static int EmptyShape(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n <= 1)
        {
            return 1;
        }

        var B = new BrepBuilder();
        Shape S = new();

        if (n == 3)
        {
            string a2 = a[2].ToLowerInvariant();

            if (a2 == "c")
            {
                var SS = new Compound();
                B.MakeCompound(ref SS);
                S = SS;
            }
            else if (a2 == "cs")
            {
                var SS = new CompSolid();
                B.MakeCompSolid(ref SS);
                S = SS;
            }
            else if (a2 == "so")
            {
                var SS = new Solid();
                B.MakeSolid(ref SS);
                S = SS;
            }
            else if (a2 == "sh")
            {
                var SS = new Shell();
                B.MakeShell(ref SS);
                S = SS;
            }
            else if (a2 == "f")
            {
                var SS = new Face();
                B.MakeFace(ref SS);
                S = SS;
            }
            else if (a2 == "w")
            {
                var SS = new Wire();
                B.MakeWire(ref SS);
                S = SS;
            }
            else if (a2 == "e")
            {
                var SS = new Edge();
                B.MakeEdge(ref SS);
                S = SS;
            }
            else if (a2 == "v")
            {
                var SS = new Vertex();
                B.MakeVertex(ref SS);
                S = SS;
            }
            else
            {
                return 1;
            }
        }

        theDI.SetShape(a[1], S);
        return 0;
    }

    // subshape name V/E/W/F/Sh/So/CS/C index
    private static int SubShape(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n <= 2)
        {
            return 1;
        }

        Shape S = theDI.GetShape(a[1]);
        if (S.IsNull)
        {
            return 0;
        }
        string aPrefix = a[1] + "_";
        int i = 0;
        if (n == 3)
        {
            int isub = theDI.Atoi(a[2]);
            var itr = new Iterator(S);
            while (itr.More())
            {
                i++;
                if (i == isub)
                {
                    string newname = aPrefix + i;
                    theDI.SetShape(newname, itr.Value());
                    di.AppendElement(newname);
                    break;
                }
                itr.Next();
            }
        }
        else
        {
            // explode a type
            ShapeType typ;
            char a20 = a[2].Length > 0 ? a[2][0] : '\0';
            char a21 = a[2].Length > 1 ? a[2][1] : '\0';
            switch (a20)
            {
                case 'C':
                case 'c':
                    typ = ShapeType.CompSolid;
                    break;

                case 'S':
                case 's':
                    if ((a21 == 'O') || (a21 == 'o'))
                    {
                        typ = ShapeType.Solid;
                    }
                    else if ((a21 == 'H') || (a21 == 'h'))
                    {
                        typ = ShapeType.Shell;
                    }
                    else
                    {
                        return 1;
                    }
                    break;

                case 'F':
                case 'f':
                    typ = ShapeType.Face;
                    break;

                case 'W':
                case 'w':
                    typ = ShapeType.Wire;
                    break;

                case 'E':
                case 'e':
                    typ = ShapeType.Edge;
                    break;

                case 'V':
                case 'v':
                    typ = ShapeType.Vertex;
                    break;

                default:
                    return 1;
            }

            int isub = theDI.Atoi(n > 3 ? a[3] : "");
            var M = new NCollection_Map<Shape>(ShapeHasher.Instance);
            M.Add(S);
            var ex = new Explorer(S, typ);
            while (ex.More())
            {
                if (M.Add(ex.Current()))
                {
                    i++;
                    if (i == isub)
                    {
                        string newname = aPrefix + i;
                        theDI.SetShape(newname, ex.Current());
                        di.AppendElement(newname);
                        break;
                    }
                }
                ex.Next();
            }
        }
        return 0;
    }

    // BRepIntCS curve1 [curve2 ...] shape [res] [tol]
    private static int BRepIntCS(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n <= 2)
        {
            // Message::SendFail (decision 405)
            theDI.Output.WriteLine("Invalid input arguments. Should be: curve1 [curve2 ...] shape [result] [tol]");
            return 1;
        }
        int indshape = 2;
        Shape S = new();
        for (; indshape <= n - 1; indshape++)
        {
            S = theDI.GetShape(a[indshape]);
            if (!S.IsNull)
            {
                break;
            }
        }
        if (S.IsNull)
        {
            theDI.Output.WriteLine("Invalid input shape");
            return 1;
        }

        var theAlg = new BRepIntCurveSurface_Inter();
        double tol = 1e-6;
        if (indshape < n - 1)
        {
            // atof: a word that isn't a number reads as 0
            double preci = global::Kerf.Engine.Foundation.StreamUtilities.Strtod(a[n - 1], out _);
            if (preci >= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                tol = preci;
            }
        }
        int nbpi = 0;
        var aB = new BrepBuilder();
        var aComp = new Compound();
        aB.MakeCompound(ref aComp);

        void AddPoint(in Point3D curp)
        {
            var aV = new Vertex();
            aB.MakeVertex(ref aV, curp, 0);
            aB.Add(aComp, aV);
            nbpi++;
            di.Append("Point ").Append(nbpi).Append(" : ").Append(curp.X()).Append(" ").Append(curp.Y()).Append(" ").Append(curp.Z()).Append("\n");
            DrawTrSurf.Set(theDI, $"brics_{nbpi}", curp);
        }

        if (indshape == 2)
        {
            Curve3D? C = DrawTrSurf.GetCurve(theDI, a[1]);
            if (C is null)
            {
                return 2;
            }
            var acur = new CurveAdaptor(C);
            theAlg.Init(S, acur, tol);

            for (; theAlg.More(); theAlg.Next())
            {
                AddPoint(theAlg.Pnt());
            }
        }
        else
        {
            theAlg.Load(S, tol);
            for (int il = 1; il < indshape; il++)
            {
                Curve3D? hl = DrawTrSurf.GetCurve(theDI, a[il]);
                if (hl is not null)
                {
                    theAlg.Init(new CurveAdaptor(hl));
                    for (; theAlg.More(); theAlg.Next())
                    {
                        AddPoint(theAlg.Pnt());
                    }
                }
            }
        }
        if (nbpi == 0)
        {
            di.Append("Points of intersections are not found\n");
        }
        if (indshape < n - 1)
        {
            theDI.SetShape(a[n - 1], aComp);
        }
        return 0;
    }

    // xbounds face
    private static int XBounds(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            di.Append("Usage : ").Append(a[0]).Append(" face\n");
            return 0;
        }
        //
        Shape aS = theDI.GetShape(a[1]);
        if (aS.IsNull)
        {
            di.Append(" null shapes is not allowed here\n");
            return 0;
        }
        if (aS.ShapeType() != ShapeType.Face)
        {
            di.Append(" shape").Append(a[1]).Append(" must be a face\n"); // OCCT: no space before the name
            return 0;
        }
        //
        Face aF = ShapeCasts.Face(aS);
        //
        BrepTools.UVBounds(aF, out double aUMin, out double aUMax, out double aVMin, out double aVMax);
        //
        // TCollection_AsciiString(double): %g
        di.Append(global::Kerf.Engine.Foundation.StreamUtilities.FormatReal(aUMin, 6)).Append("\n");
        di.Append(global::Kerf.Engine.Foundation.StreamUtilities.FormatReal(aUMax, 6)).Append("\n");
        di.Append(global::Kerf.Engine.Foundation.StreamUtilities.FormatReal(aVMin, 6)).Append("\n");
        di.Append(global::Kerf.Engine.Foundation.StreamUtilities.FormatReal(aVMax, 6)).Append("\n");
        //
        return 0;
    }

    // xclassify Solid [Tolerance=1.e-7]
    private static int XClassify(DrawInterpreter theDI, DrawResult aDI, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            aDI.Append(" use xclassify Solid [Tolerance=1.e-7]\n");
            return 1;
        }

        Shape aS = theDI.GetShape(a[1]);
        if (aS.IsNull)
        {
            aDI.Append(" Null Shape is not allowed here\n");
            return 0;
        }

        if (aS.ShapeType() != ShapeType.Solid)
        {
            aDI.Append(" Shape type must be SOLID\n");
            return 0;
        }
        //
        double aTol = 1.0e-7;
        if (n == 3)
        {
            aTol = theDI.Atof(a[2]);
        }
        //
        var aSC = new BRepClass3d_SolidClassifier(aS);
        aSC.PerformInfinitePoint(aTol);

        Containment aState = aSC.State();
        aDI.Append("state is: ");
        aDI.Append(aState switch
        {
            Containment.In => "IN\n",
            Containment.Out => "OUT\n",
            Containment.On => "ON\n",
            _ => "UNKNOWN\n",
        });
        //
        return 0;
    }
}
