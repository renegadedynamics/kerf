// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

public static partial class BRepLib
{
    // Structure for calculation of properties, necessary for decision about continuity
    private sealed class SurfaceProperties
    {
        private readonly GeomLProp_SLProps mySurfaceProps; // properties calculator
        private readonly gp_Trsf mySurfaceTrsf;
        private readonly Geom2d_Curve myCurve2d;
        private readonly bool myIsReversed; // the face based on the surface is reversed

        // tangent vector to Pcurve in UV
        private gp_Vec2d myCurveTangent;

        public SurfaceProperties(Geom_Surface theSurface, in gp_Trsf theSurfaceTrsf, Geom2d_Curve theCurve2D, bool theReversed)
        {
            mySurfaceProps = new GeomLProp_SLProps(theSurface, 2, Occt.Precision.Confusion());
            mySurfaceTrsf = theSurfaceTrsf;
            myCurve2d = theCurve2D;
            myIsReversed = theReversed;
            myCurveTangent = new gp_Vec2d();
        }

        // Calculate derivatives on surface related to the point on curve
        public void Calculate(double theParamOnCurve)
        {
            myCurve2d.D1(theParamOnCurve, out gp_Pnt2d aUV, out myCurveTangent);
            mySurfaceProps.SetParameters(aUV.X(), aUV.Y());
        }

        // Returns point just calculated
        public gp_Pnt Value() => mySurfaceProps.Value().Transformed(mySurfaceTrsf);

        // Calculate a derivative orthogonal to curve's tangent vector
        public gp_Vec Derivative()
        {
            var aDeriv = new gp_Vec();
            // direction orthogonal to tangent vector of the curve
            var anOrtho = new gp_Vec2d(-myCurveTangent.Y(), myCurveTangent.X());
            double aLen = anOrtho.Magnitude();
            if (aLen < Occt.Precision.Confusion())
            {
                return aDeriv;
            }
            anOrtho /= aLen;
            if (myIsReversed)
            {
                anOrtho.Reverse();
            }

            aDeriv.SetLinearForm(anOrtho.X(), mySurfaceProps.D1U(), anOrtho.Y(), mySurfaceProps.D1V());
            return aDeriv.Transformed(mySurfaceTrsf);
        }

        public gp_Dir Normal()
        {
            gp_Dir aNormal = mySurfaceProps.Normal();
            return aNormal.Transformed(mySurfaceTrsf);
        }

        // Calculate principal curvatures, which consist of minimal and maximal normal curvatures and
        // the directions on the tangent plane (principal direction) where the extremums are reached
        public void Curvature(out gp_Dir thePrincipalDir1, out double theCurvature1, out gp_Dir thePrincipalDir2, out double theCurvature2)
        {
            mySurfaceProps.CurvatureDirections(out thePrincipalDir1, out thePrincipalDir2);
            theCurvature1 = mySurfaceProps.MaxCurvature();
            theCurvature2 = mySurfaceProps.MinCurvature();
            if (myIsReversed)
            {
                theCurvature1 = -theCurvature1;
                theCurvature2 = -theCurvature2;
            }
            if (mySurfaceTrsf.IsNegative())
            {
                theCurvature1 = -theCurvature1;
                theCurvature2 = -theCurvature2;
            }

            thePrincipalDir1.Transform(mySurfaceTrsf);
            thePrincipalDir2.Transform(mySurfaceTrsf);
        }
    }

    /// <summary>
    /// The order of continuity between two faces connected by an edge (the angular tolerance in radians). A seam
    /// edge (both faces the same) on an elementary surface is CN.
    /// </summary>
    public static GeomAbs_Shape ContinuityOfFaces(in TopoDS_Edge theEdge,
                                                  in TopoDS_Face theFace1,
                                                  in TopoDS_Face theFace2,
                                                  double theAngleTol)
    {
        bool isSeam = theFace1.IsEqual(theFace2);

        TopoDS_Edge anEdgeInFace1 = new(), anEdgeInFace2 = new();
        Geom2d_Curve? aCurve1, aCurve2;

        double aFirst, aLast;

        if (!theFace1.IsSame(theFace2) && BRep_Tool.IsClosed(theEdge, theFace1)
            && BRep_Tool.IsClosed(theEdge, theFace2))
        {
            // Find the edge in the face 1: this edge will have correct orientation
            TopoDS_Face aFace1 = theFace1;
            aFace1.Orientation(TopAbs_FORWARD);
            var anExplo = new TopExp_Explorer(aFace1, TopAbs_EDGE);
            for (; anExplo.More(); anExplo.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(anExplo.Current());
                if (anEdge.IsSame(theEdge))
                {
                    anEdgeInFace1 = anEdge;
                    break;
                }
            }
            if (anEdgeInFace1.IsNull())
            {
                return GeomAbs_C0;
            }

            aCurve1 = BRep_Tool.CurveOnSurface(anEdgeInFace1, aFace1, out aFirst, out aLast);
            TopoDS_Face aFace2 = theFace2;
            aFace2.Orientation(TopAbs_FORWARD);
            anEdgeInFace2 = anEdgeInFace1;
            anEdgeInFace2.Reverse();
            aCurve2 = BRep_Tool.CurveOnSurface(anEdgeInFace2, aFace2, out aFirst, out aLast);
        }
        else
        {
            // Obtaining of pcurves of edge on two faces.
            anEdgeInFace1 = anEdgeInFace2 = theEdge;
            aCurve1 = BRep_Tool.CurveOnSurface(anEdgeInFace1, theFace1, out aFirst, out aLast);
            // For the case of seam edge
            if (theFace1.IsSame(theFace2))
            {
                anEdgeInFace2.Reverse();
            }
            aCurve2 = BRep_Tool.CurveOnSurface(anEdgeInFace2, theFace2, out aFirst, out aLast);
        }

        if (aCurve1 is null || aCurve2 is null)
        {
            return GeomAbs_C0;
        }

        Geom_Surface aSurface1 = BRep_Tool.Surface(theFace1, out TopLoc_Location aLoc1)!;
        gp_Trsf aSurf1Trsf = aLoc1.Transformation();
        Geom_Surface aSurface2 = BRep_Tool.Surface(theFace2, out TopLoc_Location aLoc2)!;
        gp_Trsf aSurf2Trsf = aLoc2.Transformation();

        if (aSurface1 is Geom_RectangularTrimmedSurface aTS1)
        {
            aSurface1 = aTS1.BasisSurface();
        }
        if (aSurface2 is Geom_RectangularTrimmedSurface aTS2)
        {
            aSurface2 = aTS2.BasisSurface();
        }

        // seam edge on elementary surface is always CN
        bool isElementary = (aSurface1 is Geom_ElementarySurface && aSurface2 is Geom_ElementarySurface);
        if (isSeam && isElementary)
        {
            return GeomAbs_CN;
        }

        var aSP1 = new SurfaceProperties(aSurface1, aSurf1Trsf, aCurve1, theFace1.Orientation() == TopAbs_REVERSED);
        var aSP2 = new SurfaceProperties(aSurface2, aSurf2Trsf, aCurve2, theFace2.Orientation() == TopAbs_REVERSED);

        double f, l, eps;
        BRep_Tool.Range(theEdge, out f, out l);
        var ext = new Extrema_LocateExtPC();
        BRepAdaptor_Curve? aHC2 = null;

        eps = (l - f) / 100.0;
        f += eps; // to avoid calculations on
        l -= eps; // points of pointed squares.

        double anAngleTol2 = theAngleTol * theAngleTol;

        gp_Vec aDer1, aDer2;
        double aSqLen1, aSqLen2;
        var aCrvDir1 = new gp_Dir[2];
        var aCrvDir2 = new gp_Dir[2];
        Span<double> aCrvLen1 = stackalloc double[2];
        Span<double> aCrvLen2 = stackalloc double[2];

        GeomAbs_Shape aCont = (isElementary ? GeomAbs_CN : GeomAbs_C2);
        GeomAbs_Shape aCurCont;
        double u;
        for (int i = 0; i <= 20 && aCont > GeomAbs_C0; i++)
        {
            // First suppose that this is sameParameter
            u = f + (l - f) * i / 20;

            // Check conditions for G1 and C1 continuity:
            // * calculate a derivative in tangent plane of each surface
            //   orthogonal to curve's tangent vector
            // * continuity is C1 if the vectors are equal
            // * continuity is G1 if the vectors are just parallel
            aCurCont = GeomAbs_C0;

            aSP1.Calculate(u);
            aSP2.Calculate(u);

            aDer1 = aSP1.Derivative();
            aSqLen1 = aDer1.SquareMagnitude();
            aDer2 = aSP2.Derivative();
            aSqLen2 = aDer2.SquareMagnitude();
            bool isSmoothSuspect = (aDer1.CrossSquareMagnitude(aDer2) <= anAngleTol2 * aSqLen1 * aSqLen2);
            if (isSmoothSuspect)
            {
                gp_Dir aNormal1 = aSP1.Normal();
                if (theFace1.Orientation() == TopAbs_REVERSED)
                {
                    aNormal1.Reverse();
                }
                gp_Dir aNormal2 = aSP2.Normal();
                if (theFace2.Orientation() == TopAbs_REVERSED)
                {
                    aNormal2.Reverse();
                }

                if (aNormal1 * aNormal2 < 0.0)
                {
                    return GeomAbs_C0;
                }
            }

            if (!isSmoothSuspect)
            {
                // Refine by projection
                if (aHC2 is null)
                {
                    // adaptor for pcurve on the second surface
                    aHC2 = new BRepAdaptor_Curve(anEdgeInFace2, theFace2);
                    ext.Initialize(aHC2, f, l, Occt.Precision.PConfusion());
                }
                ext.Perform(aSP1.Value(), u);
                if (ext.IsDone() && ext.IsMin())
                {
                    Extrema_POnCurv poc = ext.Point();
                    aSP2.Calculate(poc.Parameter());
                    aDer2 = aSP2.Derivative();
                    aSqLen2 = aDer2.SquareMagnitude();
                }
                isSmoothSuspect = (aDer1.CrossSquareMagnitude(aDer2) <= anAngleTol2 * aSqLen1 * aSqLen2);
            }
            if (isSmoothSuspect)
            {
                aCurCont = GeomAbs_G1;
                if (Math.Abs(Math.Sqrt(aSqLen1) - Math.Sqrt(aSqLen2)) < Occt.Precision.Confusion()
                    && aDer1.Dot(aDer2) > Occt.Precision.SquareConfusion())
                { // <= check vectors are codirectional
                    aCurCont = GeomAbs_C1;
                }
            }
            else
            {
                return GeomAbs_C0;
            }

            if (aCont < GeomAbs_G2)
            {
                continue; // no need further processing, because maximal continuity is less than G2
            }

            // Check conditions for G2 and C2 continuity:
            // * calculate principal curvatures on each surface
            // * continuity is C2 if directions of principal curvatures are equal on different surfaces
            // * continuity is G2 if directions of principal curvatures are just parallel
            //   and values of curvatures are the same
            aSP1.Curvature(out aCrvDir1[0], out aCrvLen1[0], out aCrvDir1[1], out aCrvLen1[1]);
            aSP2.Curvature(out aCrvDir2[0], out aCrvLen2[0], out aCrvDir2[1], out aCrvLen2[1]);
            for (int aStep = 0; aStep <= 1; ++aStep)
            {
                if (aCrvDir1[0].XYZ().CrossSquareMagnitude(aCrvDir2[aStep].XYZ()) <= Occt.Precision.SquareConfusion()
                    && Math.Abs(aCrvLen1[0] - aCrvLen2[aStep]) < Occt.Precision.Confusion()
                    && aCrvDir1[1].XYZ().CrossSquareMagnitude(aCrvDir2[1 - aStep].XYZ()) <= Occt.Precision.SquareConfusion()
                    && Math.Abs(aCrvLen1[1] - aCrvLen2[1 - aStep]) < Occt.Precision.Confusion())
                {
                    if (aCurCont == GeomAbs_C1 && aCrvDir1[0].Dot(aCrvDir2[aStep]) > Occt.Precision.Confusion()
                        && aCrvDir1[1].Dot(aCrvDir2[1 - aStep]) > Occt.Precision.Confusion())
                    {
                        aCurCont = GeomAbs_C2;
                    }
                    else
                    {
                        aCurCont = GeomAbs_G2;
                    }
                    break;
                }
            }

            if (aCurCont < aCont)
            {
                aCont = aCurCont;
            }
        }

        // according to the list of supported elementary surfaces,
        // if the continuity is C2, than it is totally CN
        if (isElementary && aCont == GeomAbs_C2)
        {
            aCont = GeomAbs_CN;
        }
        return aCont;
    }

    //=======================================================================
    // function : EncodeRegularity
    // purpose  : Code the regularities on all edges of the shape, boundary of
    //            two faces that do not have it.
    //            Takes into account that compound may consists of same solid
    //            placed with different transformations
    //=======================================================================
    private static void encodeRegularity(in TopoDS_Shape theShape,
                                         double theTolAng,
                                         NCollection_Map<TopoDS_Shape> theMap,
                                         NCollection_Map<TopoDS_Shape>? theEdgesToEncode = null)
    {
        TopoDS_Shape aShape = theShape;
        var aNullLoc = new TopLoc_Location();
        aShape.Location(aNullLoc); // nullify location
        if (!theMap.Add(aShape))
        {
            return; // do not need to process shape twice
        }

        if (aShape.ShapeType() == TopAbs_COMPOUND || aShape.ShapeType() == TopAbs_COMPSOLID)
        {
            for (var it = new TopoDS_Iterator(aShape); it.More(); it.Next())
            {
                encodeRegularity(it.Value(), theTolAng, theMap, theEdgesToEncode);
            }
            return;
        }

        try
        {
            var M = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapesAndAncestors(aShape, TopAbs_EDGE, TopAbs_FACE, M);
            var It = new NCollection_List<TopoDS_Shape>.Iterator();
            var Ex = new TopExp_Explorer();
            TopoDS_Face F1 = new(), F2 = new();
            for (int i = 1; i <= M.Extent(); i++)
            {
                TopoDS_Edge E = TopoDS.Edge(M.FindKey(i));
                if (theEdgesToEncode is not null && !theEdgesToEncode.IsEmpty())
                {
                    // process only the edges from the list to update their regularity
                    TopoDS_Shape aPureEdge = E.Located(aNullLoc);
                    aPureEdge.Orientation(TopAbs_FORWARD);
                    if (!theEdgesToEncode.Contains(aPureEdge))
                    {
                        continue;
                    }
                }

                bool found = false;
                F1.Nullify();
                for (It.Initialize(M.FindFromIndex(i)); It.More() && !found; It.Next())
                {
                    if (F1.IsNull())
                    {
                        F1 = TopoDS.Face(It.Value());
                    }
                    else
                    {
                        TopoDS_Face aTmpF2 = TopoDS.Face(It.Value());
                        if (!F1.IsSame(aTmpF2))
                        {
                            found = true;
                            F2 = aTmpF2;
                        }
                    }
                }
                if (!found && !F1.IsNull())
                { // is it a sewing edge?
                    TopAbs_Orientation orE = E.Orientation();
                    TopoDS_Edge curE;
                    for (Ex.Init(F1, TopAbs_EDGE); Ex.More() && !found; Ex.Next())
                    {
                        curE = TopoDS.Edge(Ex.Current());
                        if (E.IsSame(curE) && orE != curE.Orientation())
                        {
                            found = true;
                            F2 = F1;
                        }
                    }
                }
                if (found)
                {
                    EncodeRegularity(E, F1, F2, theTolAng);
                }
            }
        }
        catch (Standard_Failure)
        {
        }
    }

    /// <summary>
    /// Codes the regularity of every edge of the shape bounding two faces that hasn't one yet (in place: internal; the
    /// angular tolerance in radians). A compound holding the same solid at several locations is coded once.
    /// </summary>
    internal static void EncodeRegularity(in TopoDS_Shape S, double TolAng = 1.0e-10)
    {
        var aMap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        encodeRegularity(S, TolAng, aMap);
    }

    /// <summary>
    /// Codes the regularity of the edges of <paramref name="LE"/> bounding two faces of the shape that haven't one yet
    /// (in place: internal).
    /// </summary>
    internal static void EncodeRegularity(in TopoDS_Shape S, NCollection_List<TopoDS_Shape> LE, double TolAng = 1.0e-10)
    {
        // Collect edges without location and orientation
        var aPureEdges = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var aNullLoc = new TopLoc_Location();
        var anEdgeIt = new NCollection_List<TopoDS_Shape>.Iterator(LE);
        for (; anEdgeIt.More(); anEdgeIt.Next())
        {
            TopoDS_Shape anEdge = anEdgeIt.Value();
            anEdge.Location(aNullLoc);
            anEdge.Orientation(TopAbs_FORWARD);
            aPureEdges.Add(anEdge);
        }

        var aMap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        encodeRegularity(S, TolAng, aMap, aPureEdges);
    }

    /// <summary>
    /// Codes the regularity between <paramref name="F1"/> and <paramref name="F2"/> on <paramref name="E"/>, unless
    /// already coded (in place: internal).
    /// </summary>
    internal static void EncodeRegularity(in TopoDS_Edge E, in TopoDS_Face F1, in TopoDS_Face F2, double TolAng = 1.0e-10)
    {
        var B = new BRep_Builder();
        if (BRep_Tool.Continuity(E, F1, F2) <= GeomAbs_C0)
        {
            try
            {
                GeomAbs_Shape aCont = ContinuityOfFaces(E, F1, F2, TolAng);
                B.Continuity(E, F1, F2, aCont);
            }
            catch (Standard_Failure)
            {
            }
        }
    }

    /// <summary>
    /// The writable copies of the frozen triangulations an in-place static changes, attached once it is done.
    /// </summary>
    /// <remarks>
    /// OCCT.NET: frozen. OCCT writes normals and deflections into the triangulations attached to the faces, which the
    /// port freezes when they are attached (design 4.1). The port writes into a copy of each (one copy per
    /// triangulation, so faces sharing one share its copy, as they would see OCCT's change) and, at the end, puts the
    /// copy in place of the original on each face of the shape that has it active (keeping the face's list of
    /// triangulations, with the copy active) and on the edges' polygons on it.
    /// </remarks>
    private sealed class WritableTriangulations
    {
        private readonly Dictionary<Poly_Triangulation, Poly_Triangulation> myCopies = new(ReferenceEqualityComparer.Instance);

        // The triangulation to read: the copy if one was made.
        public Poly_Triangulation Read(Poly_Triangulation theT)
            => myCopies.TryGetValue(theT, out Poly_Triangulation? aCopy) ? aCopy : theT;

        // The triangulation to write into: a copy of a frozen one.
        public Poly_Triangulation Write(Poly_Triangulation theT)
        {
            if (myCopies.TryGetValue(theT, out Poly_Triangulation? aCopy))
            {
                return aCopy;
            }
            if (!theT.IsFrozen())
            {
                return theT;
            }
            aCopy = theT.Copy();
            myCopies.Add(theT, aCopy);
            return aCopy;
        }

        public void Attach(in TopoDS_Shape theShape)
        {
            if (myCopies.Count == 0)
            {
                return;
            }
            var aB = new BRep_Builder();
            var aDone = new HashSet<TopoDS_TShape>(ReferenceEqualityComparer.Instance);
            for (var anExpFace = new TopExp_Explorer(theShape, TopAbs_FACE); anExpFace.More(); anExpFace.Next())
            {
                TopoDS_Face aFace = TopoDS.Face(anExpFace.Current());
                if (!aDone.Add(aFace.TShape()!))
                {
                    continue;
                }
                var aTF = (BRep_TFace)aFace.TShape()!;
                Poly_Triangulation? anOld = aTF.ActiveTriangulation();
                if (anOld is null || !myCopies.TryGetValue(anOld, out Poly_Triangulation? aNew))
                {
                    continue;
                }
                aB.UpdateFace(aFace, aNew, false);
                for (var anExpEdge = new TopExp_Explorer(aFace, TopAbs_EDGE); anExpEdge.More(); anExpEdge.Next())
                {
                    var aTE = (BRep_TEdge)anExpEdge.Current().TShape()!;
                    if (!aDone.Add(aTE))
                    {
                        continue;
                    }
                    for (var anIt = new NCollection_List<BRep_CurveRepresentation>.Iterator(aTE.ChangeCurves()); anIt.More(); anIt.Next())
                    {
                        BRep_CurveRepresentation aCR = anIt.Value();
                        if (!aCR.IsPolygonOnTriangulation() || !myCopies.TryGetValue(aCR.Triangulation()!, out Poly_Triangulation? aNewT))
                        {
                            continue;
                        }
                        anIt.ChangeValue() = aCR.IsPolygonOnClosedTriangulation()
                            ? new BRep_PolygonOnClosedTriangulation(aCR.PolygonOnTriangulation(), aCR.PolygonOnTriangulation2(), aNewT, aCR.Location())
                            : new BRep_PolygonOnTriangulation(aCR.PolygonOnTriangulation(), aNewT, aCR.Location());
                    }
                    anExpEdge.Current().TShape()!.Modified(true);
                }
            }
        }
    }

    /// <summary>
    /// Corrects the normals of the faces' triangulations so that nodes along smooth edges have the same normal on
    /// both sides (computing missing normals first). Returns true if anything was corrected. In place: internal; the
    /// triangulations, frozen once attached, are replaced by corrected copies (see the remarks of the port).
    /// </summary>
    internal static bool EnsureNormalConsistency(in TopoDS_Shape theShape, double theAngTol = 0.001, bool theForceComputeNormals = false)
    {
        double aThresDot = Math.Cos(theAngTol);

        bool aRetVal = false, isNormalsFound = false;
        var aWritable = new WritableTriangulations(); // OCCT.NET: frozen

        // compute normals if they are absent
        var anExpFace = new TopExp_Explorer(theShape, TopAbs_FACE);
        for (; anExpFace.More(); anExpFace.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(anExpFace.Current());
            Geom_Surface? aSurf = BRep_Tool.Surface(aFace);
            if (aSurf is null)
            {
                continue;
            }
            Poly_Triangulation? anAttached = BRep_Tool.Triangulation(aFace, out TopLoc_Location aLoc);
            if (anAttached is null)
            {
                continue;
            }
            Poly_Triangulation aPT = aWritable.Read(anAttached);
            if (!theForceComputeNormals && aPT.HasNormals())
            {
                isNormalsFound = true;
                continue;
            }

            aPT = aWritable.Write(anAttached);
            aPT.AddNormals();
            var aSLP = new GeomLProp_SLProps(aSurf, 2, Occt.Precision.Confusion());
            for (int i = 1; i <= aPT.NbNodes(); i++)
            {
                gp_Pnt2d aP2d = aPT.UVNode(i);
                aSLP.SetParameters(aP2d.X(), aP2d.Y());

                if (!aSLP.IsNormalDefined())
                {
                    aPT.SetNormal(i, new NCollection_Vec3<float>(0.0f));
                }
                else
                {
                    gp_Dir aNorm = aSLP.Normal();
                    if (aFace.Orientation() == TopAbs_REVERSED)
                    {
                        aNorm.Reverse();
                    }
                    aPT.SetNormal(i, aNorm);
                }
            }

            aRetVal = true;
            isNormalsFound = true;
        }

        if (!isNormalsFound)
        {
            aWritable.Attach(theShape);
            return aRetVal;
        }

        // loop by edges
        var aMapEF = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(theShape, TopAbs_EDGE, TopAbs_FACE, aMapEF);
        for (int anInd = 1; anInd <= aMapEF.Extent(); anInd++)
        {
            TopoDS_Edge anEdg = TopoDS.Edge(aMapEF.FindKey(anInd));
            NCollection_List<TopoDS_Shape> anEdgList = aMapEF.FindFromIndex(anInd);
            if (anEdgList.Extent() != 2)
            {
                continue;
            }
            var anItF = new NCollection_List<TopoDS_Shape>.Iterator(anEdgList);
            TopoDS_Face aFace1 = TopoDS.Face(anItF.Value());
            anItF.Next();
            TopoDS_Face aFace2 = TopoDS.Face(anItF.Value());
            Poly_Triangulation? anAttached1 = BRep_Tool.Triangulation(aFace1, out TopLoc_Location aLoc1);
            Poly_Triangulation? anAttached2 = BRep_Tool.Triangulation(aFace2, out TopLoc_Location aLoc2);

            if (anAttached1 is null || anAttached2 is null)
            {
                continue;
            }
            Poly_Triangulation aPT1 = aWritable.Read(anAttached1);
            Poly_Triangulation aPT2 = aWritable.Read(anAttached2);

            if (!aPT1.HasNormals() || !aPT2.HasNormals())
            {
                continue;
            }

            // the edges still refer to the attached triangulations
            Poly_PolygonOnTriangulation aPTEF1 = BRep_Tool.PolygonOnTriangulation(anEdg, anAttached1, aLoc1)!;
            Poly_PolygonOnTriangulation aPTEF2 = BRep_Tool.PolygonOnTriangulation(anEdg, anAttached2, aLoc2)!;

            if (aPTEF1.Nodes().Lower() != aPTEF2.Nodes().Lower()
                || aPTEF1.Nodes().Upper() != aPTEF2.Nodes().Upper())
            {
                continue;
            }

            for (int anEdgNode = aPTEF1.Nodes().Lower(); anEdgNode <= aPTEF1.Nodes().Upper(); anEdgNode++)
            {
                // Number of node
                int aFNodF1 = aPTEF1.Nodes().Value(anEdgNode);
                int aFNodF2 = aPTEF2.Nodes().Value(anEdgNode);

                aPT1 = aWritable.Read(anAttached1);
                aPT2 = aWritable.Read(anAttached2);
                aPT1.Normal(aFNodF1, out NCollection_Vec3<float> aNorm1f);
                aPT2.Normal(aFNodF2, out NCollection_Vec3<float> aNorm2f);
                var aNorm1 = new gp_XYZ(aNorm1f.x(), aNorm1f.y(), aNorm1f.z());
                var aNorm2 = new gp_XYZ(aNorm2f.x(), aNorm2f.y(), aNorm2f.z());
                double aDot = aNorm1 * aNorm2;
                if (aDot > aThresDot)
                {
                    gp_XYZ aNewNorm = (aNorm1 + aNorm2).Normalized();
                    aWritable.Write(anAttached1).SetNormal(aFNodF1, new gp_Dir(aNewNorm));
                    aWritable.Write(anAttached2).SetNormal(aFNodF2, new gp_Dir(aNewNorm));
                    aRetVal = true;
                }
            }
        }

        aWritable.Attach(theShape);
        return aRetVal;
    }

    // Tool to estimate deflection of the given UV point with regard to its representation in 3D space.
    private sealed class EvalDeflection
    {
        private readonly BRepAdaptor_Surface Surface;

        // Initializes tool with the given face.
        public EvalDeflection(in TopoDS_Face theFace)
        {
            Surface = new BRepAdaptor_Surface(theFace);
        }

        // Evaluates deflection of the given 2d point from its 3d representation.
        public double Eval(in gp_Pnt2d thePoint2d, in gp_Pnt thePoint3d)
        {
            Surface.D0(thePoint2d.X(), thePoint2d.Y(), out gp_Pnt aPnt);
            return (thePoint3d.XYZ() - aPnt.XYZ()).SquareModulus();
        }
    }

    // Represents link of triangulation: equal whatever the order of its nodes.
    private readonly struct Link : IEquatable<Link>
    {
        private readonly int myNode0;
        private readonly int myNode1;

        public Link(int theNode1, int theNode2)
        {
            myNode0 = theNode1;
            myNode1 = theNode2;
        }

        public bool Equals(Link theOther)
            => (myNode0 == theOther.myNode0 && myNode1 == theOther.myNode1)
               || (myNode0 == theOther.myNode1 && myNode1 == theOther.myNode0);

        public override bool Equals(object? obj) => obj is Link aLink && Equals(aLink);

        public override int GetHashCode() => HashCode.Combine(Math.Min(myNode0, myNode1), Math.Max(myNode0, myNode1));
    }

    /// <summary>
    /// Sets the deflection of each face's triangulation to the maximum deviation measured on it (triangle and inner
    /// link midpoints against the surface). In place: internal; the triangulations, frozen once attached, are replaced
    /// by copies with the new deflection.
    /// </summary>
    internal static void UpdateDeflection(in TopoDS_Shape theShape)
    {
        var aWritable = new WritableTriangulations(); // OCCT.NET: frozen
        Span<int> aNode = stackalloc int[3];
        var anExpFace = new TopExp_Explorer(theShape, TopAbs_FACE);
        for (; anExpFace.More(); anExpFace.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(anExpFace.Current());
            Geom_Surface? aSurf = BRep_Tool.Surface(aFace);
            if (aSurf is null)
            {
                continue;
            }

            Poly_Triangulation? anAttached = BRep_Tool.Triangulation(aFace, out TopLoc_Location aLoc);
            if (anAttached is null)
            {
                continue;
            }
            Poly_Triangulation aPT = aWritable.Read(anAttached);
            if (!aPT.HasUVNodes())
            {
                continue;
            }

            // Collect all nodes of degenerative edges and skip elements
            // build upon them due to huge distortions introduced by passage
            // from UV space to 3D.
            var aDegNodes = new NCollection_Map<int>();
            var anExpEdge = new TopExp_Explorer(aFace, TopAbs_EDGE);
            for (; anExpEdge.More(); anExpEdge.Next())
            {
                TopoDS_Edge aEdge = TopoDS.Edge(anExpEdge.Current());
                if (BRep_Tool.Degenerated(aEdge))
                {
                    Poly_PolygonOnTriangulation? aPolygon = BRep_Tool.PolygonOnTriangulation(aEdge, anAttached, aLoc);
                    if (aPolygon is null)
                    {
                        continue;
                    }

                    for (int aNodeIt = aPolygon.Nodes().Lower(); aNodeIt <= aPolygon.Nodes().Upper(); ++aNodeIt)
                    {
                        aDegNodes.Add(aPolygon.Node(aNodeIt));
                    }
                }
            }

            var aTool = new EvalDeflection(aFace);
            var aLinks = new NCollection_Map<Link>();
            double aSqDeflection = 0.0;
            gp_Trsf aTrsf = aLoc.Transformation();
            var aP3d = new gp_Pnt[3];
            var aP2d = new gp_Pnt2d[3];
            for (int aTriIt = 1; aTriIt <= aPT.NbTriangles(); ++aTriIt)
            {
                Poly_Triangle aTriangle = aPT.Triangle(aTriIt);

                aTriangle.Get(out aNode[0], out aNode[1], out aNode[2]);
                if (aDegNodes.Contains(aNode[0]) || aDegNodes.Contains(aNode[1]) || aDegNodes.Contains(aNode[2]))
                {
                    continue;
                }

                aP3d[0] = aPT.Node(aNode[0]).Transformed(aTrsf);
                aP3d[1] = aPT.Node(aNode[1]).Transformed(aTrsf);
                aP3d[2] = aPT.Node(aNode[2]).Transformed(aTrsf);

                aP2d[0] = aPT.UVNode(aNode[0]);
                aP2d[1] = aPT.UVNode(aNode[1]);
                aP2d[2] = aPT.UVNode(aNode[2]);

                // Check midpoint of triangle.
                var aMid3d_t = new gp_Pnt((aP3d[0].XYZ() + aP3d[1].XYZ() + aP3d[2].XYZ()) / 3.0);
                var aMid2d_t = new gp_Pnt2d((aP2d[0].XY() + aP2d[1].XY() + aP2d[2].XY()) / 3.0);

                aSqDeflection = Max(aSqDeflection, aTool.Eval(aMid2d_t, aMid3d_t));

                for (int i = 0; i < 3; ++i)
                {
                    int j = (i + 1) % 3;
                    var aLink = new Link(aNode[i], aNode[j]);
                    if (!aLinks.Add(aLink))
                    {
                        // Do not estimate boundary links due to high distortions at the edge.
                        gp_Pnt aP3d1 = aP3d[i];
                        gp_Pnt aP3d2 = aP3d[j];

                        gp_Pnt2d aP2d1 = aP2d[i];
                        gp_Pnt2d aP2d2 = aP2d[j];

                        var aMid3d_l = new gp_Pnt((aP3d1.XYZ() + aP3d2.XYZ()) / 2.0);
                        var aMid2d_l = new gp_Pnt2d((aP2d1.XY() + aP2d2.XY()) / 2.0);

                        aSqDeflection = Max(aSqDeflection, aTool.Eval(aMid2d_l, aMid3d_l));
                    }
                }
            }

            aWritable.Write(anAttached).Deflection(Math.Sqrt(aSqDeflection));
        }
        aWritable.Attach(theShape);
    }

    /// <summary>
    /// Fills <paramref name="LF"/> with the faces of <paramref name="Sh"/> sorted by the complexity of their surfaces:
    /// planes, cylinders, cones, spheres, tori, others, then faces without a surface.
    /// </summary>
    public static void SortFaces(in TopoDS_Shape Sh, NCollection_List<TopoDS_Shape> LF)
    {
        LF.Clear();
        NCollection_List<TopoDS_Shape> LTri = new(), LPlan = new(), LCyl = new(), LCon = new(), LSphere = new(), LTor = new(), LOther = new();
        var exp = new TopExp_Explorer(Sh, TopAbs_FACE);
        Geom_Surface? S;

        for (; exp.More(); exp.Next())
        {
            TopoDS_Face F = TopoDS.Face(exp.Current());
            S = BRep_Tool.Surface(F, out _);
            if (S is not null)
            {
                if (S.GetType() == typeof(Geom_RectangularTrimmedSurface))
                {
                    S = ((Geom_RectangularTrimmedSurface)S).BasisSurface();
                }
                var AS = new GeomAdaptor_Surface(S);
                switch (AS.GetType())
                {
                    case GeomAbs_Plane:
                        LPlan.Append(F);
                        break;
                    case GeomAbs_Cylinder:
                        LCyl.Append(F);
                        break;
                    case GeomAbs_Cone:
                        LCon.Append(F);
                        break;
                    case GeomAbs_Sphere:
                        LSphere.Append(F);
                        break;
                    case GeomAbs_Torus:
                        LTor.Append(F);
                        break;
                    default:
                        LOther.Append(F);
                        break;
                }
            }
            else
            {
                LTri.Append(F);
            }
        }
        LF.Append(LPlan);
        LF.Append(LCyl);
        LF.Append(LCon);
        LF.Append(LSphere);
        LF.Append(LTor);
        LF.Append(LOther);
        LF.Append(LTri);
    }

    /// <summary>The faces of <paramref name="Sh"/> sorted as <see cref="SortFaces(in TopoDS_Shape, NCollection_List{TopoDS_Shape})"/> does.</summary>
    public static IBentoList<TopoDS_Shape> SortFaces(in TopoDS_Shape Sh)
    {
        var LF = new NCollection_List<TopoDS_Shape>();
        SortFaces(Sh, LF);
        return toList(LF);
    }

    /// <summary>
    /// Fills <paramref name="LF"/> with the faces of <paramref name="Sh"/> in the reverse order of complexity: faces
    /// without a surface, others, tori, spheres, cones, cylinders, planes.
    /// </summary>
    /// <remarks>Unlike SortFaces, OCCT doesn't look through a trimmed surface here (a trimmed plane counts as other).</remarks>
    public static void ReverseSortFaces(in TopoDS_Shape Sh, NCollection_List<TopoDS_Shape> LF)
    {
        LF.Clear();
        NCollection_List<TopoDS_Shape> LTri = new(), LPlan = new(), LCyl = new(), LCon = new(), LSphere = new(), LTor = new(), LOther = new();
        var exp = new TopExp_Explorer(Sh, TopAbs_FACE);

        for (; exp.More(); exp.Next())
        {
            TopoDS_Face F = TopoDS.Face(exp.Current());
            Geom_Surface? S = BRep_Tool.Surface(F, out _);
            if (S is not null)
            {
                var AS = new GeomAdaptor_Surface(S);
                switch (AS.GetType())
                {
                    case GeomAbs_Plane:
                        LPlan.Append(F);
                        break;
                    case GeomAbs_Cylinder:
                        LCyl.Append(F);
                        break;
                    case GeomAbs_Cone:
                        LCon.Append(F);
                        break;
                    case GeomAbs_Sphere:
                        LSphere.Append(F);
                        break;
                    case GeomAbs_Torus:
                        LTor.Append(F);
                        break;
                    default:
                        LOther.Append(F);
                        break;
                }
            }
            else
            {
                LTri.Append(F);
            }
        }
        LF.Append(LTri);
        LF.Append(LOther);
        LF.Append(LTor);
        LF.Append(LSphere);
        LF.Append(LCon);
        LF.Append(LCyl);
        LF.Append(LPlan);
    }

    /// <summary>The faces of <paramref name="Sh"/> sorted as <see cref="ReverseSortFaces(in TopoDS_Shape, NCollection_List{TopoDS_Shape})"/> does.</summary>
    public static IBentoList<TopoDS_Shape> ReverseSortFaces(in TopoDS_Shape Sh)
    {
        var LF = new NCollection_List<TopoDS_Shape>();
        ReverseSortFaces(Sh, LF);
        return toList(LF);
    }

    private static IBentoList<TopoDS_Shape> toList(NCollection_List<TopoDS_Shape> theList)
    {
        var anArray = new TopoDS_Shape[theList.Extent()];
        int i = 0;
        for (var anIt = new NCollection_List<TopoDS_Shape>.Iterator(theList); anIt.More(); anIt.Next())
        {
            anArray[i++] = anIt.Value();
        }
        return PackedList.Own(anArray);
    }

    /// <summary>
    /// The face <paramref name="theF"/> enlarged by <paramref name="theExtVal"/> in the chosen directions, frozen. An
    /// analytical surface is extended within its period and natural bounds; a bounded one by
    /// <c>GeomLib.ExtendSurfByLength</c>; another face is returned as it is.
    /// </summary>
    public static void ExtendFace(in TopoDS_Face theF,
                                  double theExtVal,
                                  bool theExtUMin,
                                  bool theExtUMax,
                                  bool theExtVMin,
                                  bool theExtVMax,
                                  out TopoDS_Face theFExtended)
    {
        InternalExtendFace(theF, theExtVal, theExtUMin, theExtUMax, theExtVMin, theExtVMax, out theFExtended);
        theFExtended.TShape()?.FreezeGraph();
    }

    /// <summary><see cref="ExtendFace"/> without freezing the new face, for ported code that goes on building on it.</summary>
    internal static void InternalExtendFace(in TopoDS_Face theF,
                                            double theExtVal,
                                            bool theExtUMin,
                                            bool theExtUMax,
                                            bool theExtVMin,
                                            bool theExtVMax,
                                            out TopoDS_Face theFExtended)
    {
        // Get face bounds
        var aBAS = new BRepAdaptor_Surface(theF);
        double aFUMin = aBAS.FirstUParameter(), aFUMax = aBAS.LastUParameter(),
               aFVMin = aBAS.FirstVParameter(), aFVMax = aBAS.LastVParameter();
        double aTol = BRep_Tool.Tolerance(theF);

        // Surface to build the face
        Geom_Surface aS;

        GeomAbs_SurfaceType aType = aBAS.GetType();
        // treat analytical surfaces first
        if (aType == GeomAbs_Plane || aType == GeomAbs_Sphere || aType == GeomAbs_Cylinder
            || aType == GeomAbs_Torus || aType == GeomAbs_Cone)
        {
            // Get basis transformed basis surface
            Geom_Surface aSurf = aBAS.GeomSurfaceTransformed()!;

            // Get bounds of the basis surface
            aSurf.Bounds(out double aSUMin, out double aSUMax, out double aSVMin, out double aSVMax);

            bool isUPeriodic = aBAS.IsUPeriodic();
            double anUPeriod = isUPeriodic ? aBAS.UPeriod() : 0.0;
            if (isUPeriodic)
            {
                // Adjust face bounds to first period
                double aDelta = aFUMax - aFUMin;
                aFUMin = Max(aSUMin, aFUMin + anUPeriod * Math.Ceiling((aSUMin - aFUMin) / anUPeriod));
                aFUMax = aFUMin + aDelta;
            }

            bool isVPeriodic = aBAS.IsVPeriodic();
            double aVPeriod = isVPeriodic ? aBAS.VPeriod() : 0.0;
            if (isVPeriodic)
            {
                // Adjust face bounds to first period
                double aDelta = aFVMax - aFVMin;
                aFVMin = Max(aSVMin, aFVMin + aVPeriod * Math.Ceiling((aSVMin - aFVMin) / aVPeriod));
                aFVMax = aFVMin + aDelta;
            }

            // Enlarge the face
            double anURes = 0.0, aVRes = 0.0;
            if (theExtUMin || theExtUMax)
            {
                anURes = aBAS.UResolution(theExtVal);
            }
            if (theExtVMin || theExtVMax)
            {
                aVRes = aBAS.VResolution(theExtVal);
            }

            if (theExtUMin)
            {
                aFUMin = Max(aSUMin, aFUMin - anURes);
            }
            if (theExtUMax)
            {
                aFUMax = Min(isUPeriodic ? aFUMin + anUPeriod : aSUMax, aFUMax + anURes);
            }
            if (theExtVMin)
            {
                aFVMin = Max(aSVMin, aFVMin - aVRes);
            }
            if (theExtVMax)
            {
                aFVMax = Min(isVPeriodic ? aFVMin + aVPeriod : aSVMax, aFVMax + aVRes);
            }

            // Check if the periodic surface should become closed.
            // In this case, use the basis surface with basis bounds.
            double anEps = Occt.Precision.PConfusion();
            if (isUPeriodic && Math.Abs(aFUMax - aFUMin - anUPeriod) < anEps)
            {
                aFUMin = aSUMin;
                aFUMax = aSUMax;
            }
            if (isVPeriodic && Math.Abs(aFVMax - aFVMin - aVPeriod) < anEps)
            {
                aFVMin = aSVMin;
                aFVMax = aSVMax;
            }

            aS = aSurf;
        }
        else
        {
            // General case

            // OCCT takes the face's surface without its location here, so the extended face of a located face
            // loses the location (occt-issues.md).
            if (BRep_Tool.Surface(theF) is not Geom_BoundedSurface aSB)
            {
                theFExtended = theF;
                return;
            }

            // Get surfaces bounds
            aSB.Bounds(out double aSUMin, out double aSUMax, out double aSVMin, out double aSVMax);

            bool isUClosed = aSB.IsUClosed();
            bool isVClosed = aSB.IsVClosed();

            // Check if the extension in necessary directions is done
            bool isExtUMin = false, isExtUMax = false, isExtVMin = false, isExtVMax = false;

            // UMin
            if (theExtUMin && !isUClosed && !Occt.Precision.IsInfinite(aSUMin))
            {
                GeomLib.ExtendSurfByLength(ref aSB, theExtVal, 1, true, false);
                isExtUMin = true;
            }
            // UMax
            if (theExtUMax && !isUClosed && !Occt.Precision.IsInfinite(aSUMax))
            {
                GeomLib.ExtendSurfByLength(ref aSB, theExtVal, 1, true, true);
                isExtUMax = true;
            }
            // VMin
            if (theExtVMin && !isVClosed && !Occt.Precision.IsInfinite(aSVMax)) // OCCT tests aSVMax where aSVMin is meant (occt-issues.md)
            {
                GeomLib.ExtendSurfByLength(ref aSB, theExtVal, 1, false, false);
                isExtVMin = true;
            }
            // VMax
            if (theExtVMax && !isVClosed && !Occt.Precision.IsInfinite(aSVMax))
            {
                GeomLib.ExtendSurfByLength(ref aSB, theExtVal, 1, false, true);
                isExtVMax = true;
            }

            aS = aSB;

            // Get new bounds
            aS.Bounds(out aSUMin, out aSUMax, out aSVMin, out aSVMax);
            if (isExtUMin)
            {
                aFUMin = aSUMin;
            }
            if (isExtUMax)
            {
                aFUMax = aSUMax;
            }
            if (isExtVMin)
            {
                aFVMin = aSVMin;
            }
            if (isExtVMax)
            {
                aFVMax = aSVMax;
            }
        }

        var aMF = new BRepLib_MakeFace(aS, aFUMin, aFUMax, aFVMin, aFVMax, aTol);
        theFExtended = TopoDS.Face(aMF.InternalShape());
        if (theF.Orientation() == TopAbs_REVERSED)
        {
            theFExtended.Reverse();
        }
    }
}
