// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// BOPTest_TolerCommands.cxx: <c>btolx</c>, <c>breducetolerance</c> and <c>bopaddpcs</c>, which reduce tolerances and
/// add pcurves in place (unit 6L).
/// </summary>
/// <remarks>
/// OCCT writes the new tolerances and pcurves into the shape itself; Draw-lite runs them on an unfrozen copy and rebinds
/// every variable sharing a changed sub-shape (<see cref="SWDRAWShapeFixCommands.InPlace"/>).
/// </remarks>
internal static class BOPTestTolerCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "breducetolerance", BReduceTolerance);
        DrawCxx.Add(theInterpreter, "btolx", BTolX);
        DrawCxx.Add(theInterpreter, "bopaddpcs", BopAddPCs);
    }

    private static int BTolX(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            di.Append(" use btolx Shape [minTol=1.e-7]\n");
            return 1;
        }

        Shape aS = theDI.GetShape(a[1]);

        if (aS.IsNull)
        {
            di.Append(" Null shape is not allowed\n");
            return 1;
        }
        //
        double aTolEMin = 1.0e-7;
        if (n == 3)
        {
            aTolEMin = theDI.Atof(a[2]);
        }
        // OCCT works on the shape itself: Draw-lite on an unfrozen copy, rebound afterwards (see the class)
        Shape aR = SWDRAWShapeFixCommands.InPlace(theDI, aS, aWork =>
        {
            // Edge Tolerances
            ReduceEdgeTolerance(aWork, aTolEMin);
            //
            // Face Tolerances
            ReduceFaceTolerance(aWork);
            //
            // Vertex Tolerances
            ReduceVertexTolerance(aWork);
            //
            BRepLib.SameParameter(aWork, 1.0e-7, true);
            return aWork;
        });
        //
        theDI.SetShape(a[1], aR);
        return 0;
    }

    private static void ReduceEdgeTolerance(in Shape aS, double aTolTreshold)
    {
        NCollection_IndexedMap<Shape> aEMap = new(ShapeHasher.Instance);
        //
        Exploration.MapShapes(aS, ShapeType.Edge, aEMap);
        //
        int aNbE = aEMap.Extent();
        for (int i = 1; i <= aNbE; i++)
        {
            Edge aE = ShapeCasts.Edge(aEMap[i]);

            ProcessEdge(aE, aTolTreshold);
        }
    }

    private static void ReduceFaceTolerance(in Shape aS)
    {
        double aTolEMin = 1.0e-7;
        NCollection_IndexedMap<Shape> aFMap = new(ShapeHasher.Instance);
        NCollection_IndexedMap<Shape> aEMap = new(ShapeHasher.Instance);
        Exploration.MapShapes(aS, ShapeType.Face, aFMap);
        int aNbF = aFMap.Extent();
        for (int i = 1; i <= aNbF; i++)
        {
            double aTolx = 1.0e6;
            Face aF = ShapeCasts.Face(aFMap[i]);
            BrepTFace aTF = (BrepTFace)aF.TShape()!;
            // OCCT: the edge map isn't cleared between faces, so a face's tolerance is the smallest of the edges of
            // this face and of every face before it
            Exploration.MapShapes(aF, ShapeType.Edge, aEMap);
            int aNbE = aEMap.Extent();
            for (int j = 1; j <= aNbE; ++j)
            {
                Edge aE = ShapeCasts.Edge(aEMap[j]);
                double aTolE = Tool.Tolerance(aE);
                if (aTolE < aTolx)
                {
                    aTolx = aTolE;
                }
            }
            double aTolF = (aTolx > aTolEMin) ? aTolx : aTolEMin;
            aTF.Tolerance(aTolF);
        }
    }

    private static void ReduceVertexTolerance(in Shape aS)
    {
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aVEMap = new(ShapeHasher.Instance);
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aVFMap = new(ShapeHasher.Instance);

        Exploration.MapShapesAndUniqueAncestors(aS, ShapeType.Vertex, ShapeType.Edge, aVEMap);
        Exploration.MapShapesAndUniqueAncestors(aS, ShapeType.Vertex, ShapeType.Face, aVFMap);

        int aNbV = aVEMap.Extent();
        for (int i = 1; i <= aNbV; i++)
        {
            Vertex aV = ShapeCasts.Vertex(aVEMap.FindKey(i));
            NCollection_List<Shape> aLE = aVEMap[i];
            NCollection_List<Shape> aLF = aVFMap.FindFromKey(aV);

            ProcessVertex(aV, aLE, aLF);
        }
    }

    private static void ProcessEdge(in Edge aE, double aTolTreshold)
    {
        int aNb = 23;
        Curve3D? aC3D = Tool.Curve(aE, out double aT1, out double aT2);
        if (aC3D is null)
        {
            return;
        }
        double dT = (aT2 - aT1) / aNb;
        BrepTEdge TE = (BrepTEdge)aE.TShape()!;
        Placement Eloc = aE.Location();
        double aTolMax2 = -1.0e6;
        foreach (CurveRepresentation cr in TE.Curves())
        {
            Placement loc = cr.Location();
            Placement L = Eloc * loc;
            // 3D-Curve
            if (cr.IsCurve3D)
            {
                continue;
            }
            // 2D-Curve
            else if (cr.IsCurveOnSurface())
            {
                Curve2D? aC2D = cr.PCurve();
                if (aC2D is null)
                {
                    continue;
                }
                // Surface
                Surface aS = cr.Surface()!;
                // 2D-point treatment
                for (int i = 0; i <= aNb; ++i)
                {
                    double aT = aT1 + i * dT;
                    if (i == aNb)
                    {
                        aT = aT2;
                    }
                    Point3D aPC3D = aC3D.Value(aT);
                    Point2D aPC2D = aC2D.Value(aT);
                    Point3D aP3D = aS.Value(aPC2D.X(), aPC2D.Y());
                    aP3D.Transform(L.Transformation());
                    double aD2 = aPC3D.SquareDistance(aP3D);
                    if (aD2 > aTolMax2)
                    {
                        aTolMax2 = aD2;
                    }
                }
            }
        }

        if (aTolMax2 < 0.0)
        {
            return;
        }
        aTolMax2 = Math.Sqrt(aTolMax2);

        double aTolSet = (aTolMax2 > aTolTreshold) ? aTolMax2 : aTolTreshold;

        TE.Tolerance(aTolSet);
    }

    private static void ProcessVertex(in Vertex aV, NCollection_List<Shape> aLE, NCollection_List<Shape> aLF)
    {
        double aTolMax2 = -1.0e6;
        BrepTVertex TV = (BrepTVertex)aV.TShape()!;
        Point3D aPV3D = TV.Pnt();
        double aTol = Tool.Tolerance(aV);
        foreach (Shape aLEi in aLE)
        {
            Edge aE = ShapeCasts.Edge(aLEi);
            BrepTEdge TE = (BrepTEdge)aE.TShape()!;
            Placement Eloc = aE.Location();
            Explorer aVExp = new(aE, ShapeType.Vertex);
            for (; aVExp.More(); aVExp.Next())
            {
                Vertex aVx = ShapeCasts.Vertex(aVExp.Current());
                if (!aVx.IsSame(aV))
                {
                    continue;
                }
                Orientation anOrV = aVx.Orientation();
                if (anOrV != Orientation.Forward && anOrV != Orientation.Reversed)
                {
                    continue;
                }
                foreach (CurveRepresentation cr in TE.Curves())
                {
                    Placement loc = cr.Location();
                    Placement L = (Eloc * loc).Predivided(aV.Location());
                    // 3D-Curve
                    if (cr.IsCurve3D)
                    {
                        Curve3D? aC3D = cr.Curve3D();
                        if (aC3D is null)
                        {
                            continue;
                        }
                        // 3D-point treatment
                        double aParam = Tool.Parameter(aVx, aE);
                        Point3D aPC3D = aC3D.Value(aParam);
                        aPC3D.Transform(L.Transformation());
                        double aD2 = aPV3D.SquareDistance(aPC3D);
                        if (aD2 > aTolMax2)
                        {
                            aTolMax2 = aD2;
                        }
                    }
                    // 2D-Curve
                    else if (cr.IsCurveOnSurface())
                    {
                        Curve2D? aC2D = cr.PCurve();
                        if (aC2D is null)
                        {
                            continue;
                        }
                        // Surface
                        Surface aS = cr.Surface()!;
                        // 2D-point treatment
                        double aParam = Tool.Parameter(aVx, aE, aS, L);
                        Point2D aPC2D = aC2D.Value(aParam);
                        Point3D aPC3D = aS.Value(aPC2D.X(), aPC2D.Y());
                        aPC3D.Transform(L.Transformation());
                        double aD2 = aPV3D.SquareDistance(aPC3D);
                        if (aD2 > aTolMax2)
                        {
                            aTolMax2 = aD2;
                        }
                    }
                }
            }
        }
        //  Reducing
        if (aTolMax2 < 0.0)
        {
            return;
        }
        aTolMax2 = Math.Sqrt(aTolMax2);
        if (aTolMax2 > aTol)
        {
            return;
        }
        foreach (Shape aLEi in aLE)
        {
            Edge aE = ShapeCasts.Edge(aLEi);

            double aTolE = Tool.Tolerance(aE);
            if (aTolMax2 < aTolE)
            {
                aTolMax2 = aTolE;
            }
        }
        foreach (Shape aLFi in aLF)
        {
            Face aF = ShapeCasts.Face(aLFi);

            double aTolE = Tool.Tolerance(aF);
            if (aTolMax2 < aTolE)
            {
                aTolMax2 = aTolE;
            }
        }
        if (aTolMax2 > aTol)
        {
            return;
        }
        //
        // Update Tolerance
        TV.Tolerance(aTolMax2);
    }

    private static int BReduceTolerance(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            di.Append(" use bupdatetolerance Shape\n");
            return 1;
        }

        Shape aS = theDI.GetShape(a[1]);

        if (aS.IsNull)
        {
            di.Append(" Null shape is not allowed \n");
            return 1;
        }
        Shape aR = SWDRAWShapeFixCommands.InPlace(theDI, aS, aWork =>
        {
            ReduceVertexTolerance(aWork);
            return aWork;
        });
        theDI.SetShape(a[1], aR);

        return 0;
    }

    /// <summary>
    /// <c>bopaddpcs Shape</c>: adds to the edges the pcurves on the faces holding them that they don't store (on planes,
    /// BRep_Tool computes them).
    /// </summary>
    private static int BopAddPCs(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            di.Append(" Use >bopaddpcs Shape\n");
            return 1;
        }

        Shape aS = theDI.GetShape(a[1]);

        if (aS.IsNull)
        {
            di.Append(" Null shape is not allowed \n");
            return 1;
        }
        //
        Shape aR = SWDRAWShapeFixCommands.InPlace(theDI, aS, aWork =>
        {
            PreparePCurves(aWork, di);
            return aWork;
        });
        //
        theDI.SetShape(a[1], aR);
        return 0;
    }

    private static void PreparePCurves(in Shape aShape, DrawResult di)
    {
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aEFMap = new(ShapeHasher.Instance);
        BrepBuilder aBB = new();
        //
        Exploration.MapShapesAndAncestors(aShape, ShapeType.Edge, ShapeType.Face, aEFMap);
        //
        int aNbE = aEFMap.Extent();
        for (int i = 1; i <= aNbE; ++i)
        {
            Edge aE = ShapeCasts.Edge(aEFMap.FindKey(i));
            //
            if (Tool.Degenerated(aE))
            {
                continue;
            }
            //
            Curve3D? aC3D = Tool.Curve(aE, out double aT1, out double aT2);
            if (aC3D is null)
            {
                continue;
            }
            double aTolE = Tool.Tolerance(aE);
            //
            NCollection_List<Shape> aLF = aEFMap[i];
            foreach (Shape aLFi in aLF)
            {
                Face aF = ShapeCasts.Face(aLFi);
                //
                // Map of surfaces on which the edge lays .
                List<Surface?> aSCRMap = [];
                BrepTEdge TE = (BrepTEdge)aE.TShape()!;
                foreach (CurveRepresentation cr in TE.Curves())
                {
                    //
                    if (cr.IsCurveOnSurface())
                    {
                        Surface? aSCR = cr.Surface();
                        if (!aSCRMap.Contains(aSCR))
                        {
                            aSCRMap.Add(aSCR);
                        }
                    }
                    //
                }
                //
                Surface? aS = Tool.Surface(aF, out Placement aLoc);
                if (!aSCRMap.Contains(aS))
                {
                    // try to obtain 2D curve
                    Curve2D? aC2D = Tool.CurveOnSurface(aE, aS, aLoc, out aT1, out aT2);
                    if (aC2D is null)
                    {
                        di.Append(" Warning: Can not obtain P-Curve\n");
                        continue;
                    }
                    else
                    {
                        aBB.UpdateEdge(aE, aC2D, aF, aTolE);
                    }
                }
            }
        }
    }
}
