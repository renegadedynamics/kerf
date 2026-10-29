// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_NurbsConvertModification.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_NurbsConvertModification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The modification that converts the geometry of a shape to NURBS: surfaces to B-spline surfaces (trimmed to the
/// face's UV bounds), curves to B-spline curves reparametrised to the edge's range, and pcurves to B-splines on the
/// new surfaces (projected again where the parametrisation changes). Meshes are kept, their parameters mapped onto
/// the new geometry.
/// </summary>
/// <remarks>
/// The pcurves are recomputed with ProjLib_ComputeApprox (on planes) and ProjLib_ComputeApproxOnPolarSurface. The maps
/// of converted geometry compare handles by reference, as OCCT's do.
/// </remarks>
public class BRepTools_NurbsConvertModification : BRepTools_CopyModification
{
    private readonly NCollection_List<TopoDS_Shape> myled = new();
    private readonly NCollection_List<Standard_Transient?> mylcu = new();
    private readonly NCollection_IndexedDataMap<object, object> myMap = new(ReferenceEqualityComparer.Instance);
    private readonly NCollection_List<TopoDS_Shape> myUpdatedEdges = new();

    /// <summary>A NURBS conversion.</summary>
    public BRepTools_NurbsConvertModification()
    {
    }

    private static void GeomLib_ChangeUBounds(Geom_BSplineSurface aSurface, double newU1, double newU2)
    {
        NCollection_Array1<double> knots = new(aSurface.InternalUKnots());
        BSplCLib.Reparametrize(newU1, newU2, knots);
        aSurface.SetUKnots(knots);
    }

    private static void GeomLib_ChangeVBounds(Geom_BSplineSurface aSurface, double newV1, double newV2)
    {
        NCollection_Array1<double> knots = new(aSurface.InternalVKnots());
        BSplCLib.Reparametrize(newV1, newV2, knots);
        aSurface.SetVKnots(knots);
    }

    // find 3D curve from theEdge in theMap, and return the transformed curve or NULL
    private static Geom_Curve? newCurve(NCollection_IndexedDataMap<object, object> theMap,
                                        in TopoDS_Edge theEdge,
                                        out double theFirst,
                                        out double theLast)
    {
        Geom_Curve? aNewCurve = null;

        Geom_Curve? aCurve = BRep_Tool.Curve(theEdge, out TopLoc_Location aLoc, out theFirst, out theLast);
        if (aCurve is not null && theMap.Contains(aCurve))
        {
            aNewCurve = (Geom_Curve)theMap.FindFromKey(aCurve);
            aNewCurve = (Geom_Curve)aNewCurve.Transformed(aLoc.Transformation());
        }
        return aNewCurve;
    }

    // find 2D curve from theEdge on theFace in theMap, and return the transformed curve or NULL
    private static Geom2d_Curve? newCurve(NCollection_IndexedDataMap<object, object> theMap,
                                          in TopoDS_Edge theEdge,
                                          in TopoDS_Face theFace,
                                          out double theFirst,
                                          out double theLast)
    {
        Geom2d_Curve? aC2d = BRep_Tool.CurveOnSurface(theEdge, theFace, out theFirst, out theLast);
        return (aC2d is not null && theMap.Contains(aC2d))
                 ? (Geom2d_Curve)theMap.FindFromKey(aC2d)
                 : null;
    }

    // find surface from theFace in theMap, and return the transformed surface or NULL
    private static Geom_Surface? newSurface(NCollection_IndexedDataMap<object, object> theMap, in TopoDS_Face theFace)
    {
        Geom_Surface? aNewSurf = null;

        Geom_Surface? aSurf = BRep_Tool.Surface(theFace, out TopLoc_Location aLoc);
        if (aSurf is not null && theMap.Contains(aSurf))
        {
            aNewSurf = (Geom_Surface)theMap.FindFromKey(aSurf);
            aNewSurf = (Geom_Surface)aNewSurf.Transformed(aLoc.Transformation());
        }
        return aNewSurf;
    }

    private static bool newParameter(in gp_Pnt thePoint,
                                     Geom_Curve theCurve,
                                     double theFirst,
                                     double theLast,
                                     double theTol,
                                     ref double theParam)
    {
        GeomAdaptor_Curve anAdaptor = new GeomAdaptor_Curve(theCurve);
        Extrema_LocateExtPC proj = new Extrema_LocateExtPC(thePoint,
                                                           anAdaptor,
                                                           theParam,
                                                           theFirst,
                                                           theLast,
                                                           Precision.PConfusion());
        if (proj.IsDone())
        {
            double aDist2Min = proj.SquareDistance();
            if (aDist2Min < theTol * theTol)
            {
                theParam = proj.Point().Parameter();
                return true;
            }
        }
        return false;
    }

    private static bool newParameter(in gp_Pnt2d theUV,
                                     Geom2d_Curve theCurve2d,
                                     double theFirst,
                                     double theLast,
                                     double theTol,
                                     ref double theParam)
    {
        Geom2dAdaptor_Curve anAdaptor = new Geom2dAdaptor_Curve(theCurve2d);
        Extrema_LocateExtPC2d aProj = new Extrema_LocateExtPC2d(theUV, anAdaptor, theParam, Precision.PConfusion());
        if (aProj.IsDone())
        {
            double aDist2Min = aProj.SquareDistance();
            if (aDist2Min < theTol * theTol)
            {
                theParam = aProj.Point().Parameter();
                return true;
            }
        }
        else
        {
            // Try to use general extrema to find the parameter, because Extrema_LocateExtPC2d
            // sometimes could not find a solution if the parameter's first approach is several
            // spans away from the expected solution (test bugs/modalg_7/bug28722).
            Extrema_ExtPC2d anExt = new Extrema_ExtPC2d(theUV, anAdaptor, theFirst, theLast);
            if (anExt.IsDone())
            {
                int aMinInd = 0;
                double aMinSqDist = Precision.Infinite();
                for (int anIndex = 1; anIndex <= anExt.NbExt(); ++anIndex)
                {
                    if (anExt.SquareDistance(anIndex) < aMinSqDist)
                    {
                        aMinSqDist = anExt.SquareDistance(anIndex);
                        aMinInd = anIndex;
                    }
                }
                if (aMinSqDist < theTol * theTol)
                {
                    theParam = anExt.Point(aMinInd).Parameter();
                    return true;
                }
            }
        }
        return false;
    }

    private static bool newUV(in gp_Pnt thePoint, Geom_Surface theSurf, double theTol, ref gp_Pnt2d theUV)
    {
        GeomAdaptor_Surface anAdaptor = new GeomAdaptor_Surface(theSurf);
        Extrema_GenLocateExtPS aProj = new Extrema_GenLocateExtPS(anAdaptor);
        aProj.Perform(thePoint, theUV.X(), theUV.Y());
        if (aProj.IsDone())
        {
            double aDist2Min = aProj.SquareDistance();
            if (aDist2Min < theTol * theTol)
            {
                aProj.Point().Parameter(out double aU, out double aV);
                theUV.SetCoord(aU, aV);
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// The face's surface converted to a B-spline surface (trimmed to the face's UV bounds where they are inside the
    /// surface's); false if the surface is already a B-spline or Bezier surface, or the face has none.
    /// </summary>
    public override bool NewSurface(in TopoDS_Face F,
                                    out Geom_Surface? S,
                                    out TopLoc_Location L,
                                    out double Tol,
                                    out bool RevWires,
                                    out bool RevFace)
    {
        double U1, U2, curvU1, curvU2, surfU1, surfU2, UTol;
        double V1, V2, curvV1, curvV2, surfV1, surfV2, VTol;
        S = null;
        Tol = 0.0;
        RevWires = false;
        RevFace = false;
        Geom_Surface? SS = BRep_Tool.Surface(F, out L);
        if (SS is null)
        {
            // processing the case when there is no geometry
            return false;
        }

        Type TheTypeSS = SS.GetType();
        if ((TheTypeSS == typeof(Geom_BSplineSurface))
            || (TheTypeSS == typeof(Geom_BezierSurface)))
        {
            return false;
        }
        S = SS;
        BRepTools.UVBounds(F, out curvU1, out curvU2, out curvV1, out curvV2);
        Tol = BRep_Tool.Tolerance(F);
        double TolPar = 0.1 * Tol;
        bool IsUp = S.IsUPeriodic(), IsVp = S.IsVPeriodic();
        // OCC466(apo)->
        U1 = curvU1;
        U2 = curvU2;
        V1 = curvV1;
        V2 = curvV2;
        S.Bounds(out surfU1, out surfU2, out surfV1, out surfV2);

        if (Math.Abs(U1 - surfU1) <= TolPar)
        {
            U1 = surfU1;
        }
        if (Math.Abs(U2 - surfU2) <= TolPar)
        {
            U2 = surfU2;
        }
        if (Math.Abs(V1 - surfV1) <= TolPar)
        {
            V1 = surfV1;
        }
        if (Math.Abs(V2 - surfV2) <= TolPar)
        {
            V2 = surfV2;
        }

        if (!IsUp)
        {
            U1 = Max(surfU1, curvU1);
            U2 = Min(surfU2, curvU2);
        }
        if (!IsVp)
        {
            V1 = Max(surfV1, curvV1);
            V2 = Min(surfV2, curvV2);
        }
        //<-OCC466(apo)

        if (IsUp)
        {
            double Up = S.UPeriod();
            if (U2 - U1 > Up)
            {
                U2 = U1 + Up;
            }
        }
        if (IsVp)
        {
            double Vp = S.VPeriod();
            if (V2 - V1 > Vp)
            {
                V2 = V1 + Vp;
            }
        }

        if (Math.Abs(surfU1 - U1) > Tol || Math.Abs(surfU2 - U2) > Tol || Math.Abs(surfV1 - V1) > Tol
            || Math.Abs(surfV2 - V2) > Tol)
        {
            S = new Geom_RectangularTrimmedSurface(S, U1, U2, V1, V2);
        }
        S.Bounds(out surfU1, out surfU2, out surfV1, out surfV2);

        S = GeomConvert.SurfaceToBSplineSurface(S);
        Geom_BSplineSurface BS = (Geom_BSplineSurface)S;
        BS.Resolution(Tol, out UTol, out VTol);

        //
        // Readjust the bounds of S, otherwise the old PCurves become invalid
        //

        if (Math.Abs(curvU1 - surfU1) > UTol && !BS.IsUPeriodic())
        {
            GeomLib_ChangeUBounds(BS, U1, U2);
        }
        if (Math.Abs(curvV1 - surfV1) > VTol && !BS.IsVPeriodic())
        {
            GeomLib_ChangeVBounds(BS, V1, V2);
        }

        if (!myMap.Contains(SS))
        {
            myMap.Add(SS, S);
        }
        return true;
    }

    private static bool IsConvert(in TopoDS_Edge E)
    {
        bool isConvert = false;
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        // iterate on pcurves
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());
        for (; itcr.More() && !isConvert; itcr.Next())
        {
            BRep_GCurve? GC = itcr.Value() as BRep_GCurve;
            if (GC is null || !GC.IsCurveOnSurface())
            {
                continue;
            }
            Geom_Surface aSurface = GC.Surface()!;
            Geom2d_Curve aCurve2d = GC.PCurve()!;
            isConvert = ((aSurface is not Geom_BSplineSurface
                          && aSurface is not Geom_BezierSurface)
                         || (aCurve2d is not Geom2d_BSplineCurve
                             && aCurve2d is not Geom2d_BezierCurve));
        }
        return isConvert;
    }

    /// <summary>The face's triangulation, its UV nodes projected on the new surface (see the copy modification for when).</summary>
    public override bool NewTriangulation(in TopoDS_Face theFace, out Poly_Triangulation? theTri)
    {
        if (!base.NewTriangulation(theFace, out theTri))
        {
            return false;
        }

        // convert UV nodes of the mesh
        if (theTri!.HasUVNodes())
        {
            Geom_Surface? aSurf = BRep_Tool.Surface(theFace, out _);
            Geom_Surface? aNewSurf = newSurface(myMap, theFace);
            if (aSurf is not null && aNewSurf is not null)
            {
                double aTol = BRep_Tool.Tolerance(theFace);
                for (int anInd = 1; anInd <= theTri.NbNodes(); ++anInd)
                {
                    gp_Pnt2d aUV = theTri.UVNode(anInd);
                    gp_Pnt aPoint = aSurf.Value(aUV.X(), aUV.Y());
                    if (newUV(aPoint, aNewSurf, aTol, ref aUV))
                    {
                        theTri.SetUVNode(anInd, aUV);
                    }
                }
            }
        }

        return true;
    }

    /// <summary>
    /// The edge's 3D curve converted to a B-spline reparametrised to the edge's range (a copy of a B-spline or Bezier
    /// curve if one of the edge's pcurves or surfaces needs converting); none for a degenerated edge.
    /// </summary>
    public override bool NewCurve(in TopoDS_Edge E, out Geom_Curve? C, out TopLoc_Location L, out double Tol)
    {
        C = null;
        Tol = BRep_Tool.Tolerance(E);
        if (BRep_Tool.Degenerated(E))
        {
            C = null;
            L = new TopLoc_Location();
            return true;
        }

        Geom_Curve? Caux = BRep_Tool.Curve(E, out L, out double f, out double l);

        if (Caux is null)
        {
            L.Identity();
            return false;
        }
        Type TheType = Caux.GetType();
        if ((TheType == typeof(Geom_BSplineCurve)) || (TheType == typeof(Geom_BezierCurve)))
        {
            if (IsConvert(E))
            {
                C = (Geom_Curve)Caux.Copy();
                return true;
            }
            return false;
        }

        C = Caux;

        double TolPar = Tol * .1;

        if (C.IsPeriodic())
        {
            double p = C.Period();
            double d = Math.Abs(l - f);
            if (Math.Abs(d - p) <= TolPar && l <= p)
            {
            }
            else
            {
                C = new Geom_TrimmedCurve(C, f, l);
            }
        }
        else
        {
            C = new Geom_TrimmedCurve(C, f, l);
        }

        // modification for WOK++ HP porting (fbi 14/03/97)
        //   gp_Trsf trsf(L);
        //   gp_Trsf trsf = L.Transformation();

        //  C = GeomConvert::CurveToBSplineCurve(C,Convert_QuasiAngular);

        C = GeomConvert.CurveToBSplineCurve(C);

        double fnew = C.FirstParameter(), lnew = C.LastParameter(), UTol;

        Geom_BSplineCurve BC = (Geom_BSplineCurve)C;

        if (!BC.IsPeriodic())
        {
            BC.Resolution(Tol, out UTol);
            if (Math.Abs(f - fnew) > UTol || Math.Abs(l - lnew) > UTol)
            {
                NCollection_Array1<double> knots = new(BC.InternalKnots());
                BSplCLib.Reparametrize(f, l, knots);
                BC.SetKnots(knots);
            }
        }

        if (!myMap.Contains(Caux))
        {
            myMap.Add(Caux, C);
        }
        return true;
    }

    /// <summary>The edge's 3D polygon, its parameters projected on the new curve (see the copy modification for when).</summary>
    public override bool NewPolygon(in TopoDS_Edge theEdge, out Poly_Polygon3D? thePoly)
    {
        if (!base.NewPolygon(theEdge, out thePoly))
        {
            return false;
        }

        if (!thePoly!.HasParameters())
        {
            return false;
        }
        // update parameters of polygon
        double aTol = BRep_Tool.Tolerance(theEdge);
        Geom_Curve? aCurve = BRep_Tool.Curve(theEdge, out double aFirst, out double aLast);
        Geom_Curve? aNewCurve = newCurve(myMap, theEdge, out aFirst, out aLast);
        if (aCurve is null || aNewCurve is null) // skip processing degenerated edges
        {
            return false;
        }
        NCollection_Array1<double> aParams = thePoly.ChangeParameters();
        for (int anInd = aParams.Lower(); anInd <= aParams.Upper(); ++anInd)
        {
            ref double aParam = ref aParams[anInd];
            gp_Pnt aPoint = aCurve.Value(aParam);
            newParameter(aPoint, aNewCurve, aFirst, aLast, aTol, ref aParam);
        }
        return true;
    }

    /// <summary>Vertices keep their points: always false.</summary>
    public override bool NewPoint(in TopoDS_Vertex V, out gp_Pnt P, out double Tol)
    {
        P = new gp_Pnt();
        Tol = 0.0;
        return false;
    }

    /// <summary>
    /// The edge's pcurve on the face as a B-spline on the new surface, the edge's tolerance raised to the new
    /// deviation (recorded in <see cref="GetUpdatedEdges"/>).
    /// </summary>
    public override bool NewCurve2d(in TopoDS_Edge E,
                                    in TopoDS_Face F,
                                    in TopoDS_Edge newE,
                                    in TopoDS_Face newF,
                                    out Geom2d_Curve? Curve2d,
                                    out double Tol)
    {
        Curve2d = null;
        Tol = BRep_Tool.Tolerance(E);
        Geom2d_Curve? aBaseC2d = BRep_Tool.CurveOnSurface(E, F, out double f2d, out double l2d);
        Geom_Curve? C3d = BRep_Tool.Curve(E, out TopLoc_Location Loc, out double f3d, out double l3d);
        bool isConvert2d = ((C3d is not null && C3d is not Geom_BSplineCurve
                             && C3d is not Geom_BezierCurve)
                            || IsConvert(E));

        Geom2d_Curve C2d = aBaseC2d!;
        if (BRep_Tool.Degenerated(E))
        {
            // Curve2d = C2d;
            if (C2d is not Geom2d_TrimmedCurve)
            {
                Geom2d_TrimmedCurve aTrimC = new Geom2d_TrimmedCurve(C2d, f2d, l2d);
                C2d = aTrimC;
            }
            Curve2d = Geom2dConvert.CurveToBSplineCurve(C2d);
            myMap.Add(aBaseC2d!, Curve2d);
            return true;
        }
        if (!BRepTools.IsReallyClosed(E, F))
        {
            Type TheTypeC2d = C2d.GetType();

            if (TheTypeC2d == typeof(Geom2d_TrimmedCurve))
            {
                Geom2d_TrimmedCurve TC = (Geom2d_TrimmedCurve)C2d;
                C2d = TC.BasisCurve();
            }

            double fc = C2d.FirstParameter(), lc = C2d.LastParameter();

            if (!C2d.IsPeriodic())
            {
                if (fc - f2d > Precision.PConfusion())
                {
                    f2d = fc;
                }
                if (l2d - lc > Precision.PConfusion())
                {
                    l2d = lc;
                }
            }

            C2d = new Geom2d_TrimmedCurve(C2d, f2d, l2d);

            Geom2dAdaptor_Curve G2dAC = new Geom2dAdaptor_Curve(C2d, f2d, l2d);
            Geom2dAdaptor_Curve G2dAHC = (Geom2dAdaptor_Curve)G2dAC.ShallowCopy();

            if (!newE.IsNull())
            {
                C3d = BRep_Tool.Curve(newE, out f3d, out l3d);
            }
            if (C3d is null)
            {
                C3d = BRep_Tool.Curve(E, out f3d, out l3d);
            }
            if (C3d is null)
            {
                return false;
            }
            GeomAdaptor_Curve G3dAC = new GeomAdaptor_Curve(C3d, f3d, l3d);
            GeomAdaptor_Curve G3dAHC = (GeomAdaptor_Curve)G3dAC.ShallowCopy();

            double Uinf, Usup, Vinf, Vsup, u = 0, v = 0;
            Geom_Surface S = BRep_Tool.Surface(F)!;
            Type myT = S.GetType();
            if (myT != typeof(Geom_Plane))
            {
                if (newF.IsNull())
                {
                    Type st = C2d.GetType();
                    if ((st == typeof(Geom2d_BSplineCurve)) || (st == typeof(Geom2d_BezierCurve)))
                    {
                        if (isConvert2d)
                        {
                            Curve2d = (Geom2d_Curve)C2d.Copy();
                            double newTol = BRepTools.EvalAndUpdateTol(newE, C3d, Curve2d, S, f3d, l3d);
                            if (newTol > Tol)
                            {
                                Tol = newTol;
                                myUpdatedEdges.Append(newE);
                            }
                            myMap.Add(aBaseC2d!, Curve2d);
                            return true;
                        }
                        return false;
                    }
                }
                else
                {
                    Geom_Surface? aNewS = BRep_Tool.Surface(newF);
                    if (aNewS is not null)
                    {
                        S = aNewS;
                    }
                }
                S.Bounds(out Uinf, out Usup, out Vinf, out Vsup);
                // Uinf -= 1e-9; Usup += 1e-9; Vinf -= 1e-9; Vsup += 1e-9;
                u = (Usup - Uinf) * 0.1;
                v = (Vsup - Vinf) * 0.1;
                if (S.IsUPeriodic())
                {
                    double uperiod = S.UPeriod();
                    if (uperiod < (Usup + 2 * u - Uinf))
                    {
                        if (uperiod <= (Usup - Uinf))
                        {
                            u = 0;
                        }
                        else
                        {
                            u = (uperiod - (Usup - Uinf)) * 0.5;
                        }
                    }
                }
                if (S.IsVPeriodic())
                {
                    double vperiod = S.VPeriod();
                    if (vperiod < (Vsup + 2 * v - Vinf))
                    {
                        if (vperiod <= (Vsup - Vinf))
                        {
                            v = 0;
                        }
                        else
                        {
                            v = (vperiod - (Vsup - Vinf)) * 0.5;
                        }
                    }
                }
            }
            else
            {
                S = BRep_Tool.Surface(F)!; // If S is a plane, no change of parametrization
                GeomAdaptor_Surface GAHS = new GeomAdaptor_Surface(S);
                ProjLib_ComputeApprox ProjOnCurve = new ProjLib_ComputeApprox(G3dAHC, GAHS, Tol);
                if (ProjOnCurve.InternalBSpline() is null)
                {
                    Curve2d = Geom2dConvert.CurveToBSplineCurve(ProjOnCurve.InternalBezier()!);
                    double newTol = BRepTools.EvalAndUpdateTol(newE, C3d, Curve2d, S, f3d, l3d);
                    if (newTol > Tol)
                    {
                        Tol = newTol;
                        myUpdatedEdges.Append(newE);
                    }
                    myMap.Add(aBaseC2d!, Curve2d);
                    return true;
                }
                Curve2d = ProjOnCurve.InternalBSpline();
                {
                    double newTol = BRepTools.EvalAndUpdateTol(newE, C3d, Curve2d!, S, f3d, l3d);
                    if (newTol > Tol)
                    {
                        Tol = newTol;
                        myUpdatedEdges.Append(newE);
                    }
                }
                myMap.Add(aBaseC2d!, Curve2d!);
                return true;
            }

            //
            {
                GeomAdaptor_Surface GAHS = new GeomAdaptor_Surface(S, Uinf - u, Usup + u, Vinf - v, Vsup + v);

                ProjLib_ComputeApproxOnPolarSurface ProjOnCurve =
                    new ProjLib_ComputeApproxOnPolarSurface(G2dAHC, G3dAHC, GAHS, Tol);

                if (ProjOnCurve.IsDone())
                {
                    Curve2d = ProjOnCurve.InternalBSpline()!;
                    if (S.IsUPeriodic() || S.IsVPeriodic())
                    {
                        // Surface is periodic, checking curve2d domain
                        // Old domain
                        double aMinDist = Precision.Infinite();
                        if (S.IsUPeriodic())
                        {
                            aMinDist = Min(0.5 * S.UPeriod(), aMinDist);
                        }
                        if (S.IsVPeriodic())
                        {
                            aMinDist = Min(0.5 * S.VPeriod(), aMinDist);
                        }
                        aMinDist *= aMinDist;
                        // Old domain
                        gp_Pnt2d aPf = C2d.Value(f2d);
                        // New domain
                        gp_Pnt2d aNewPf = Curve2d.Value(f2d);
                        gp_Vec2d aT = new gp_Vec2d(aNewPf, aPf);
                        if (aT.SquareMagnitude() > aMinDist)
                        {
                            Curve2d = (Geom2d_Curve)Curve2d.Translated(aT);
                        }
                    }
                    double newTol = BRepTools.EvalAndUpdateTol(newE, C3d, Curve2d, S, f3d, l3d);
                    if (newTol > Tol)
                    {
                        Tol = newTol;
                        myUpdatedEdges.Append(newE);
                    }
                    myMap.Add(aBaseC2d!, Curve2d);
                    return true;
                }
                else
                {
                    Curve2d = Geom2dConvert.CurveToBSplineCurve(C2d);
                    double newTol = BRepTools.EvalAndUpdateTol(newE, C3d, Curve2d, S, f3d, l3d);
                    if (newTol > Tol)
                    {
                        Tol = newTol;
                        myUpdatedEdges.Append(newE);
                    }
                    myMap.Add(aBaseC2d!, Curve2d);
                    return true;
                }
            }
        }
        else
        {
            var itled = new NCollection_List<TopoDS_Shape>.Iterator();
            var itlcu = new NCollection_List<Standard_Transient?>.Iterator();

            for (itled.Initialize(myled), itlcu.Initialize(mylcu); itled.More(); // itlcu.More()
                 itled.Next(), itlcu.Next())
            {
                if (itled.Value().IsSame(E))
                {
                    // already processed
                    break;
                }
            }
            if (!itled.More())
            { // store the edge and the 2D curve
                Geom2d_Curve? C2dBis;
                C2d = new Geom2d_TrimmedCurve(C2d, f2d, l2d);
                Geom2dAdaptor_Curve G2dAHC = new Geom2dAdaptor_Curve(C2d, f2d, l2d);
                TopoDS_Edge ERevers = E;
                ERevers.Reverse();
                C2dBis = BRep_Tool.CurveOnSurface(ERevers, F, out double f2dBis, out double l2dBis);
                C2dBis = new Geom2d_TrimmedCurve(C2dBis!, f2dBis, l2dBis);
                Geom2dAdaptor_Curve G2dAHCBis = new Geom2dAdaptor_Curve(C2dBis, f2dBis, l2dBis);

                if (C3d is null)
                {
                    if (isConvert2d)
                    {
                        Curve2d = (Geom2d_Curve)C2d.Copy();

                        // OCCT.NET: OCCT's S, renamed: C# can't shadow the S declared below
                        Geom_Surface? aS;
                        if (newF.IsNull())
                        {
                            aS = BRep_Tool.Surface(F);
                        }
                        else
                        {
                            aS = BRep_Tool.Surface(newF);
                        }
                        //
                        // OCCT: EvalAndUpdateTol dereferences the null 3D curve here (C3d->IsPeriodic())
                        double newTol = BRepTools.EvalAndUpdateTol(newE, C3d!, Curve2d, aS!, f3d, l3d);
                        if (newTol > Tol)
                        {
                            Tol = newTol;
                            myUpdatedEdges.Append(newE);
                        }
                        myMap.Add(aBaseC2d!, Curve2d);
                        return true;
                    }
                    return false;
                }
                if (!newE.IsNull())
                {
                    C3d = BRep_Tool.Curve(newE, out f3d, out l3d);
                }
                GeomAdaptor_Curve G3dAHC = new GeomAdaptor_Curve(C3d!, f3d, l3d);

                Geom_Surface S = BRep_Tool.Surface(F)!;
                if (newF.IsNull())
                {
                    mylcu.Append(C2dBis);
                    Type st = C2d.GetType();
                    if ((st == typeof(Geom2d_BSplineCurve)) || (st == typeof(Geom2d_BezierCurve)))
                    {
                        if (isConvert2d)
                        {
                            Curve2d = (Geom2d_Curve)C2d.Copy();
                            double newTol = BRepTools.EvalAndUpdateTol(newE, C3d!, Curve2d, S, f3d, l3d);
                            if (newTol > Tol)
                            {
                                Tol = newTol;
                                myUpdatedEdges.Append(newE);
                            }
                            myMap.Add(aBaseC2d!, Curve2d);
                            return true;
                        }
                        return false;
                    }
                }
                else
                {
                    S = BRep_Tool.Surface(newF)!; // S is a BSplineSurface: no need for the trimmed one
                }
                double Uinf, Usup, Vinf, Vsup, u = 0, v = 0;
                S.Bounds(out Uinf, out Usup, out Vinf, out Vsup);
                // Uinf -= 1e-9; Usup += 1e-9; Vinf -= 1e-9; Vsup += 1e-9;
                u = (Usup - Uinf) * 0.1;
                v = (Vsup - Vinf) * 0.1;
                if (S.IsUPeriodic())
                {
                    double uperiod = S.UPeriod();
                    if (uperiod < (Usup + 2 * u - Uinf))
                    {
                        if (uperiod <= (Usup - Uinf))
                        {
                            u = 0;
                        }
                        else
                        {
                            u = (uperiod - (Usup - Uinf)) * 0.5;
                        }
                    }
                }
                if (S.IsVPeriodic())
                {
                    double vperiod = S.VPeriod();
                    if (vperiod < (Vsup + 2 * v - Vinf))
                    {
                        if (vperiod <= (Vsup - Vinf))
                        {
                            v = 0;
                        }
                        else
                        {
                            v = (vperiod - (Vsup - Vinf)) * 0.5;
                        }
                    }
                }
                GeomAdaptor_Surface GAHS = new GeomAdaptor_Surface(S, Uinf - u, Usup + u, Vinf - v, Vsup + v);
                myled.Append(E);

                ProjLib_ComputeApproxOnPolarSurface ProjOnCurve =
                    new ProjLib_ComputeApproxOnPolarSurface(G2dAHC, G2dAHCBis, G3dAHC, GAHS, Tol);

                if (ProjOnCurve.IsDone())
                {
                    Curve2d = ProjOnCurve.InternalBSpline()!;
                    mylcu.Append(ProjOnCurve.InternalCurve2d());
                    double newTol = BRepTools.EvalAndUpdateTol(newE, C3d!, Curve2d, S, f3d, l3d);
                    if (newTol > Tol)
                    {
                        Tol = newTol;
                        myUpdatedEdges.Append(newE);
                    }
                    myMap.Add(aBaseC2d!, Curve2d);
                    return true;
                }
                else
                {
                    Curve2d = Geom2dConvert.CurveToBSplineCurve(C2d);
                    double newTol = BRepTools.EvalAndUpdateTol(newE, C3d!, Curve2d, S, f3d, l3d);
                    if (newTol > Tol)
                    {
                        Tol = newTol;
                        myUpdatedEdges.Append(newE);
                    }
                    mylcu.Append(C2dBis);
                    myMap.Add(aBaseC2d!, Curve2d);
                    return true;
                }
            }
            else
            { // we are on the second pass
                C2d = (Geom2d_Curve)itlcu.Value()!;
                Type st = C2d.GetType();
                if (!(st == typeof(Geom2d_BSplineCurve)) && !(st == typeof(Geom2d_BezierCurve)))
                {
                    return false;
                }
                Curve2d = Geom2dConvert.CurveToBSplineCurve(C2d);
                myMap.Add(aBaseC2d!, Curve2d);
                return true;
            }
        }
    }

    /// <summary>The edge's polygon on the face's triangulation, its parameters projected on the new pcurve.</summary>
    public override bool NewPolygonOnTriangulation(in TopoDS_Edge theEdge, in TopoDS_Face theFace, out Poly_PolygonOnTriangulation? thePoly)
    {
        if (!base.NewPolygonOnTriangulation(theEdge, theFace, out thePoly))
        {
            return false;
        }

        // update parameters of 2D polygon
        if (thePoly!.HasParameters())
        {
            double aTol = Max(BRep_Tool.Tolerance(theEdge), thePoly.Deflection());
            Geom_Surface? aSurf = BRep_Tool.Surface(theFace, out _);
            Geom_Surface? aNewSurf = newSurface(myMap, theFace);
            Geom2d_Curve? aC2d = BRep_Tool.CurveOnSurface(theEdge, theFace, out double aFirst, out double aLast);
            Geom2d_Curve? aNewC2d = newCurve(myMap, theEdge, theFace, out aFirst, out aLast);
            if (aSurf is not null && aC2d is not null && aNewSurf is not null && aNewC2d is not null)
            {
                // compute 2D tolerance
                GeomAdaptor_Surface aSurfAdapt = new GeomAdaptor_Surface(aSurf);
                double aTol2D = Max(aSurfAdapt.UResolution(aTol), aSurfAdapt.VResolution(aTol));

                for (int anInd = 1; anInd <= thePoly.NbNodes(); ++anInd)
                {
                    double aParam = thePoly.Parameter(anInd);
                    gp_Pnt2d aUV = aC2d.Value(aParam);
                    gp_Pnt aPoint = aSurf.Value(aUV.X(), aUV.Y());
                    if (newUV(aPoint, aNewSurf, aTol, ref aUV)
                        && newParameter(aUV, aNewC2d, aFirst, aLast, aTol2D, ref aParam))
                    {
                        thePoly.SetParameter(anInd, aParam);
                    }
                }
            }
        }
        return true;
    }

    /// <summary>The vertex's parameter projected on the edge's new curve; false on a degenerated edge or without one.</summary>
    public override bool NewParameter(in TopoDS_Vertex V, in TopoDS_Edge E, ref double P, ref double Tol)
    {
        Tol = BRep_Tool.Tolerance(V);
        if (BRep_Tool.Degenerated(E))
        {
            return false;
        }

        gp_Pnt pnt = BRep_Tool.Pnt(V);
        P = BRep_Tool.Parameter(V, E);
        Geom_Curve? aNewCurve = newCurve(myMap, E, out double aFirst, out double aLast);
        return aNewCurve is not null && newParameter(pnt, aNewCurve, aFirst, aLast, Tol, ref P);
    }

    /// <summary>The edge's continuity between the original faces.</summary>
    public override GeomAbs_Shape Continuity(in TopoDS_Edge E,
                                             in TopoDS_Face F1,
                                             in TopoDS_Face F2,
                                             in TopoDS_Edge NewE,
                                             in TopoDS_Face NewF1,
                                             in TopoDS_Face NewF2)
    {
        return BRep_Tool.Continuity(E, F1, F2);
    }

    /// <summary>
    /// The new edges whose tolerance the conversion raised. Internal: they are the modifier's unfrozen edges, which
    /// BRepBuilderAPI_NurbsConvert goes on updating.
    /// </summary>
    internal NCollection_List<TopoDS_Shape> GetUpdatedEdges() => myUpdatedEdges;
}
