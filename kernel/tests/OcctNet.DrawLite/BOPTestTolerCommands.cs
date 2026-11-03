// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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

        TopoDS_Shape aS = theDI.GetShape(a[1]);

        if (aS.IsNull())
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
        TopoDS_Shape aR = SWDRAWShapeFixCommands.InPlace(theDI, aS, aWork =>
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

    private static void ReduceEdgeTolerance(in TopoDS_Shape aS, double aTolTreshold)
    {
        NCollection_IndexedMap<TopoDS_Shape> aEMap = new(TopTools_ShapeMapHasher.Instance);
        //
        TopExp.MapShapes(aS, TopAbs_ShapeEnum.TopAbs_EDGE, aEMap);
        //
        int aNbE = aEMap.Extent();
        for (int i = 1; i <= aNbE; i++)
        {
            TopoDS_Edge aE = TopoDS.Edge(aEMap[i]);

            ProcessEdge(aE, aTolTreshold);
        }
    }

    private static void ReduceFaceTolerance(in TopoDS_Shape aS)
    {
        double aTolEMin = 1.0e-7;
        NCollection_IndexedMap<TopoDS_Shape> aFMap = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedMap<TopoDS_Shape> aEMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aS, TopAbs_ShapeEnum.TopAbs_FACE, aFMap);
        int aNbF = aFMap.Extent();
        for (int i = 1; i <= aNbF; i++)
        {
            double aTolx = 1.0e6;
            TopoDS_Face aF = TopoDS.Face(aFMap[i]);
            BRep_TFace aTF = (BRep_TFace)aF.TShape()!;
            // OCCT: the edge map isn't cleared between faces, so a face's tolerance is the smallest of the edges of
            // this face and of every face before it
            TopExp.MapShapes(aF, TopAbs_ShapeEnum.TopAbs_EDGE, aEMap);
            int aNbE = aEMap.Extent();
            for (int j = 1; j <= aNbE; ++j)
            {
                TopoDS_Edge aE = TopoDS.Edge(aEMap[j]);
                double aTolE = BRep_Tool.Tolerance(aE);
                if (aTolE < aTolx)
                {
                    aTolx = aTolE;
                }
            }
            double aTolF = (aTolx > aTolEMin) ? aTolx : aTolEMin;
            aTF.Tolerance(aTolF);
        }
    }

    private static void ReduceVertexTolerance(in TopoDS_Shape aS)
    {
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aVEMap = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aVFMap = new(TopTools_ShapeMapHasher.Instance);

        TopExp.MapShapesAndUniqueAncestors(aS, TopAbs_ShapeEnum.TopAbs_VERTEX, TopAbs_ShapeEnum.TopAbs_EDGE, aVEMap);
        TopExp.MapShapesAndUniqueAncestors(aS, TopAbs_ShapeEnum.TopAbs_VERTEX, TopAbs_ShapeEnum.TopAbs_FACE, aVFMap);

        int aNbV = aVEMap.Extent();
        for (int i = 1; i <= aNbV; i++)
        {
            TopoDS_Vertex aV = TopoDS.Vertex(aVEMap.FindKey(i));
            NCollection_List<TopoDS_Shape> aLE = aVEMap[i];
            NCollection_List<TopoDS_Shape> aLF = aVFMap.FindFromKey(aV);

            ProcessVertex(aV, aLE, aLF);
        }
    }

    private static void ProcessEdge(in TopoDS_Edge aE, double aTolTreshold)
    {
        int aNb = 23;
        Geom_Curve? aC3D = BRep_Tool.Curve(aE, out double aT1, out double aT2);
        if (aC3D is null)
        {
            return;
        }
        double dT = (aT2 - aT1) / aNb;
        BRep_TEdge TE = (BRep_TEdge)aE.TShape()!;
        Placement Eloc = aE.Location();
        double aTolMax2 = -1.0e6;
        foreach (BRep_CurveRepresentation cr in TE.Curves())
        {
            Placement loc = cr.Location();
            Placement L = Eloc * loc;
            // 3D-Curve
            if (cr.IsCurve3D())
            {
                continue;
            }
            // 2D-Curve
            else if (cr.IsCurveOnSurface())
            {
                Geom2d_Curve? aC2D = cr.PCurve();
                if (aC2D is null)
                {
                    continue;
                }
                // Surface
                Geom_Surface aS = cr.Surface()!;
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

    private static void ProcessVertex(in TopoDS_Vertex aV, NCollection_List<TopoDS_Shape> aLE, NCollection_List<TopoDS_Shape> aLF)
    {
        double aTolMax2 = -1.0e6;
        BRep_TVertex TV = (BRep_TVertex)aV.TShape()!;
        Point3D aPV3D = TV.Pnt();
        double aTol = BRep_Tool.Tolerance(aV);
        foreach (TopoDS_Shape aLEi in aLE)
        {
            TopoDS_Edge aE = TopoDS.Edge(aLEi);
            BRep_TEdge TE = (BRep_TEdge)aE.TShape()!;
            Placement Eloc = aE.Location();
            TopExp_Explorer aVExp = new(aE, TopAbs_ShapeEnum.TopAbs_VERTEX);
            for (; aVExp.More(); aVExp.Next())
            {
                TopoDS_Vertex aVx = TopoDS.Vertex(aVExp.Current());
                if (!aVx.IsSame(aV))
                {
                    continue;
                }
                TopAbs_Orientation anOrV = aVx.Orientation();
                if (anOrV != TopAbs_Orientation.TopAbs_FORWARD && anOrV != TopAbs_Orientation.TopAbs_REVERSED)
                {
                    continue;
                }
                foreach (BRep_CurveRepresentation cr in TE.Curves())
                {
                    Placement loc = cr.Location();
                    Placement L = (Eloc * loc).Predivided(aV.Location());
                    // 3D-Curve
                    if (cr.IsCurve3D())
                    {
                        Geom_Curve? aC3D = cr.Curve3D();
                        if (aC3D is null)
                        {
                            continue;
                        }
                        // 3D-point treatment
                        double aParam = BRep_Tool.Parameter(aVx, aE);
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
                        Geom2d_Curve? aC2D = cr.PCurve();
                        if (aC2D is null)
                        {
                            continue;
                        }
                        // Surface
                        Geom_Surface aS = cr.Surface()!;
                        // 2D-point treatment
                        double aParam = BRep_Tool.Parameter(aVx, aE, aS, L);
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
        foreach (TopoDS_Shape aLEi in aLE)
        {
            TopoDS_Edge aE = TopoDS.Edge(aLEi);

            double aTolE = BRep_Tool.Tolerance(aE);
            if (aTolMax2 < aTolE)
            {
                aTolMax2 = aTolE;
            }
        }
        foreach (TopoDS_Shape aLFi in aLF)
        {
            TopoDS_Face aF = TopoDS.Face(aLFi);

            double aTolE = BRep_Tool.Tolerance(aF);
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

        TopoDS_Shape aS = theDI.GetShape(a[1]);

        if (aS.IsNull())
        {
            di.Append(" Null shape is not allowed \n");
            return 1;
        }
        TopoDS_Shape aR = SWDRAWShapeFixCommands.InPlace(theDI, aS, aWork =>
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

        TopoDS_Shape aS = theDI.GetShape(a[1]);

        if (aS.IsNull())
        {
            di.Append(" Null shape is not allowed \n");
            return 1;
        }
        //
        TopoDS_Shape aR = SWDRAWShapeFixCommands.InPlace(theDI, aS, aWork =>
        {
            PreparePCurves(aWork, di);
            return aWork;
        });
        //
        theDI.SetShape(a[1], aR);
        return 0;
    }

    private static void PreparePCurves(in TopoDS_Shape aShape, DrawResult di)
    {
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aEFMap = new(TopTools_ShapeMapHasher.Instance);
        BRep_Builder aBB = new();
        //
        TopExp.MapShapesAndAncestors(aShape, TopAbs_ShapeEnum.TopAbs_EDGE, TopAbs_ShapeEnum.TopAbs_FACE, aEFMap);
        //
        int aNbE = aEFMap.Extent();
        for (int i = 1; i <= aNbE; ++i)
        {
            TopoDS_Edge aE = TopoDS.Edge(aEFMap.FindKey(i));
            //
            if (BRep_Tool.Degenerated(aE))
            {
                continue;
            }
            //
            Geom_Curve? aC3D = BRep_Tool.Curve(aE, out double aT1, out double aT2);
            if (aC3D is null)
            {
                continue;
            }
            double aTolE = BRep_Tool.Tolerance(aE);
            //
            NCollection_List<TopoDS_Shape> aLF = aEFMap[i];
            foreach (TopoDS_Shape aLFi in aLF)
            {
                TopoDS_Face aF = TopoDS.Face(aLFi);
                //
                // Map of surfaces on which the edge lays .
                List<Geom_Surface?> aSCRMap = [];
                BRep_TEdge TE = (BRep_TEdge)aE.TShape()!;
                foreach (BRep_CurveRepresentation cr in TE.Curves())
                {
                    //
                    if (cr.IsCurveOnSurface())
                    {
                        Geom_Surface? aSCR = cr.Surface();
                        if (!aSCRMap.Contains(aSCR))
                        {
                            aSCRMap.Add(aSCR);
                        }
                    }
                    //
                }
                //
                Geom_Surface? aS = BRep_Tool.Surface(aF, out Placement aLoc);
                if (!aSCRMap.Contains(aS))
                {
                    // try to obtain 2D curve
                    Geom2d_Curve? aC2D = BRep_Tool.CurveOnSurface(aE, aS, aLoc, out aT1, out aT2);
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
