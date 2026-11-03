// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_SolidExplorer.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_SolidExplorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Occt.Extrema_ExtFlag;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// The exploration of a shape for the classification of points in it: the lines from the point towards its faces,
/// the shells and their faces with an intersector each, and a tree of the boxes of its edges and vertices.
/// </summary>
/// <remarks>
/// A working object (decision 21). It only reads the shape. Its arguments that OCCT may leave unwritten are
/// <c>ref</c> (decision 57): the point, parameters and derivatives of the FindAPointInTheFace and PointInTheFace
/// overloads, and the line and parameter of <see cref="Segment"/> and <see cref="OtherSegment"/>.
/// </remarks>
public class BRepClass3d_SolidExplorer
{
    private TopoDS_Shape myShape = new();
    private bool myReject;
    private int myFirstFace;
    private double myParamOnEdge;
    private readonly TopExp_Explorer myShellExplorer = new();
    private readonly TopExp_Explorer myFaceExplorer = new();
    private readonly NCollection_DataMap<TopoDS_Shape, IntCurvesFace_Intersector?> myMapOfInter =
        new(TopTools_ShapeMapHasher.Instance);
    private readonly SpatialTree<int, Box> myTree = new();
    private readonly NCollection_IndexedMap<TopoDS_Shape> myMapEV = new(TopTools_ShapeMapHasher.Instance);

    /// <summary>
    /// A point <paramref name="APoint_"/> in the face <paramref name="_face"/>: <paramref name="param_"/>, in ]0, 1[,
    /// initialises the search; different values give different points.
    /// </summary>
    public static bool FindAPointInTheFace(in TopoDS_Face _face, ref Point3D APoint_, ref double param_)
    {
        double u = 0.0, v = 0.0;
        bool r = FindAPointInTheFace(_face, ref APoint_, ref u, ref v, ref param_);
        return r;
    }

    /// <summary>A point <paramref name="APoint_"/> in the face, with its parameters (see the other overloads).</summary>
    public static bool FindAPointInTheFace(in TopoDS_Face _face,
                                           ref Point3D APoint_,
                                           ref double u_,
                                           ref double v_,
                                           ref double param_)
    {
        Vector3D aVecD1U = new(), aVecD1V = new();
        return FindAPointInTheFace(_face, ref APoint_, ref u_, ref v_, ref param_, ref aVecD1U, ref aVecD1V);
    }

    /// <summary>
    /// A point <paramref name="APoint_"/> in the face, with its parameters and the surface's first derivatives there:
    /// a point a little inside an edge, moved towards the nearest other edge, where the surface isn't singular. The
    /// outputs are unchanged when none is found.
    /// </summary>
    public static bool FindAPointInTheFace(in TopoDS_Face _face,
                                           ref Point3D APoint_,
                                           ref double u_,
                                           ref double v_,
                                           ref double param_,
                                           ref Vector3D theVecD1U,
                                           ref Vector3D theVecD1V)
    {
        TopoDS_Face face = _face;
        face.Orientation(TopAbs_FORWARD);

        Vector2D T;
        Point2D P;

        for (var faceexplorer = new TopExp_Explorer(face, TopAbs_EDGE); faceexplorer.More(); faceexplorer.Next())
        {
            TopoDS_Edge Edge = TopoDS.Edge(faceexplorer.Current());
            var c = new BRepAdaptor_Curve2d(Edge, face);
            if (c.Curve() is null)
            {
                continue;
            }

            c.D1((c.LastParameter() - c.FirstParameter()) * param_ + c.FirstParameter(), out P, out T);

            double x = T.X();
            double y = T.Y();
            if (Edge.Orientation() == TopAbs_FORWARD)
            {
                T.SetCoord(-y, x);
            }
            else
            {
                T.SetCoord(y, -x);
            }

            double ParamInit = global::Kerf.Engine.Foundation.Precision.Infinite();
            double TolInit = 0.00001;
            bool APointExist = false;

            var FClassifier = new BRepClass_FacePassiveClassifier();

            T.Normalize();
            P.SetCoord(P.X() + TolInit * T.X(), P.Y() + TolInit * T.Y());
            FClassifier.Reset(new Line2D(P, new Direction2D(T)), ParamInit, RealEpsilon()); //-- Length and Tolerance #######

            var otherfaceexplorer = new TopExp_Explorer();
            int aNbEdges = 0;
            for (otherfaceexplorer.Init(face, TopAbs_EDGE); otherfaceexplorer.More();
                 otherfaceexplorer.Next(), ++aNbEdges)
            {
                TopoDS_Edge OtherEdge = TopoDS.Edge(otherfaceexplorer.Current());
                if (OtherEdge.Orientation() != TopAbs_EXTERNAL && OtherEdge != Edge)
                {
                    var AEdge = new BRepClass_Edge(OtherEdge, face);
                    FClassifier.Compare(AEdge, OtherEdge.Orientation());
                    if (FClassifier.ClosestIntersection() != 0)
                    {
                        if (ParamInit > FClassifier.Parameter())
                        {
                            ParamInit = FClassifier.Parameter();
                            APointExist = true;
                        }
                    }
                }
            }

            if (aNbEdges == 1)
            {
                var AEdge = new BRepClass_Edge(Edge, face);
                FClassifier.Compare(AEdge, Edge.Orientation());
                if (FClassifier.ClosestIntersection() != 0)
                {
                    if (ParamInit > FClassifier.Parameter())
                    {
                        ParamInit = FClassifier.Parameter();
                        APointExist = true;
                    }
                }
            }

            while (APointExist)
            {
                ParamInit *= 0.41234;
                u_ = P.X() + ParamInit * T.X();
                v_ = P.Y() + ParamInit * T.Y();

                // Additional check
                var Classifier = new BRepTopAdaptor_FClass2d(face, global::Kerf.Engine.Foundation.Precision.Confusion());
                Point2D aPnt2d = new(u_, v_);
                TopAbs_State StateOfResultingPoint = Classifier.Perform(aPnt2d);
                if (StateOfResultingPoint != TopAbs_IN)
                {
                    return false;
                }

                var s = new BRepAdaptor_Surface();
                s.Initialize(face, false);
                s.D1(u_, v_, out APoint_, out theVecD1U, out theVecD1V);

                if (theVecD1U.CrossMagnitude(theVecD1V) > Constants.Resolution())
                {
                    return true;
                }

                if (ParamInit < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return false;
                }
            }
        }

        return false;
    }

    /// <summary>A point in the face, searched on a grid of the given UV bounds of <paramref name="surf"/> (see the overload with derivatives).</summary>
    public bool PointInTheFace(in TopoDS_Face Face,
                               ref Point3D APoint_,
                               ref double u_,
                               ref double v_,
                               ref double param_,
                               ref int IndexPoint,
                               BRepAdaptor_Surface surf,
                               double U1,
                               double V1,
                               double U2,
                               double V2)
    {
        Vector3D aVecD1U = new(), aVecD1V = new();
        return PointInTheFace(Face,
                              ref APoint_,
                              ref u_,
                              ref v_,
                              ref param_,
                              ref IndexPoint,
                              surf,
                              U1,
                              V1,
                              U2,
                              V2,
                              ref aVecD1U,
                              ref aVecD1V);
    }

    /// <summary>The state of a UV point in a face: ON within the tolerance of an edge or vertex, otherwise the intersector's.</summary>
    protected TopAbs_State ClassifyUVPoint(IntCurvesFace_Intersector theIntersector,
                                           BRepAdaptor_Surface theSurf,
                                           in Point2D theP2d)
    {
        // first find if the point is near an edge/vertex
        Point3D aP3d = theSurf.Value(theP2d.X(), theP2d.Y());
        var aSelectorPoint = new BRepClass3d_BndBoxTreeSelectorPoint(myMapEV);
        aSelectorPoint.SetCurrentPoint(aP3d);
        int aSelsVE = myTree.Select(aSelectorPoint);
        if (aSelsVE > 0)
        {
            // The point is inside the tolerance area of vertices/edges => return ON state.
            return TopAbs_ON;
        }

        return theIntersector.ClassifyUVPoint(theP2d);
    }

    /// <summary>
    /// A point in the face, searched on a grid of the given UV bounds of <paramref name="surf"/> from the point of
    /// index <paramref name="IndexPoint"/>, which returns the index of the point found; then by
    /// <see cref="FindAPointInTheFace(in TopoDS_Face, ref Point3D, ref double, ref double, ref double, ref Vector3D, ref Vector3D)"/>.
    /// </summary>
    public bool PointInTheFace(in TopoDS_Face Face,
                               ref Point3D APoint_,
                               ref double u_,
                               ref double v_,
                               ref double param_,
                               ref int IndexPoint,
                               BRepAdaptor_Surface surf,
                               double U1,
                               double V1,
                               double U2,
                               double V2,
                               ref Vector3D theVecD1U,
                               ref Vector3D theVecD1V)
    {
        double u, du = (U2 - U1) / 6.0;
        double v, dv = (V2 - V1) / 6.0;
        if (du < 1e-12)
        {
            du = 1e-12;
        }

        if (dv < 1e-12)
        {
            dv = 1e-12;
        }

        bool IsNotUper = !surf.IsUPeriodic(), IsNotVper = !surf.IsVPeriodic();
        int NbPntCalc = 0;
        if (myMapOfInter.IsBound(Face))
        {
            IntCurvesFace_Intersector? ptr = myMapOfInter.Find(Face);
            bool IsInside = true;
            if (IsNotUper)
            {
                IsInside = (u_ >= U1) && (u_ <= U2);
            }

            if (IsNotVper)
            {
                IsInside &= (v_ >= V1) && (v_ <= V2);
            }

            if (ptr is not null)
            {
                IntCurvesFace_Intersector TheIntersector = ptr;
                // Check if the point is already in the face
                if (IsInside && (ClassifyUVPoint(TheIntersector, surf, new Point2D(u_, v_)) == TopAbs_IN))
                {
                    surf.D1(u_, v_, out Point3D aPnt, out theVecD1U, out theVecD1V);
                    if (aPnt.SquareDistance(APoint_) < global::Kerf.Engine.Foundation.Precision.Confusion() * global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        return true;
                    }
                }

                //-- Take 4 points in each Quarter of surface
                //-- -> Index : 1 -> 16
                //--
                //--
                //--  Then take a matrix of points on a tight grid
                //--
                for (u = du + (U1 + U2) * 0.5; u < U2; u += du)
                {
                    //--  0  X    u increases
                    for (v = dv + (V1 + V2) * 0.5; v < V2; v += dv)
                    {
                        //--  0  0    v increases
                        if (++NbPntCalc >= IndexPoint)
                        {
                            if (ClassifyUVPoint(TheIntersector, surf, new Point2D(u, v)) == TopAbs_IN)
                            {
                                u_ = u;
                                v_ = v;
                                surf.D1(u, v, out APoint_, out theVecD1U, out theVecD1V);
                                IndexPoint = NbPntCalc;
                                return (true);
                            }
                        }
                    }
                }

                for (u = -du + (U1 + U2) * 0.5; u > U1; u -= du)
                {
                    //--  0  0    u decreases
                    for (v = -dv + (V1 + V2) * 0.5; v > V1; v -= dv)
                    {
                        //--  X  0    v decreases
                        if (++NbPntCalc >= IndexPoint)
                        {
                            if (ClassifyUVPoint(TheIntersector, surf, new Point2D(u, v)) == TopAbs_IN)
                            {
                                u_ = u;
                                v_ = v;
                                surf.D1(u, v, out APoint_, out theVecD1U, out theVecD1V);
                                IndexPoint = NbPntCalc;
                                return (true);
                            }
                        }
                    }
                }

                for (u = -du + (U1 + U2) * 0.5; u > U1; u -= du)
                {
                    //--  X  0    u decreases
                    for (v = dv + (V1 + V2) * 0.5; v < V2; v += dv)
                    {
                        //--  0  0    v increases
                        if (++NbPntCalc >= IndexPoint)
                        {
                            if (ClassifyUVPoint(TheIntersector, surf, new Point2D(u, v)) == TopAbs_IN)
                            {
                                u_ = u;
                                v_ = v;
                                surf.D1(u, v, out APoint_, out theVecD1U, out theVecD1V);
                                IndexPoint = NbPntCalc;
                                return (true);
                            }
                        }
                    }
                }

                for (u = du + (U1 + U2) * 0.5; u < U2; u += du)
                {
                    //--  0  0     u increases
                    for (v = -dv + (V1 + V2) * 0.5; v > V1; v -= dv)
                    {
                        //--  0  X     v decreases
                        if (++NbPntCalc >= IndexPoint)
                        {
                            if (ClassifyUVPoint(TheIntersector, surf, new Point2D(u, v)) == TopAbs_IN)
                            {
                                u_ = u;
                                v_ = v;
                                surf.D1(u, v, out APoint_, out theVecD1U, out theVecD1V);
                                IndexPoint = NbPntCalc;
                                return (true);
                            }
                        }
                    }
                }

                //-- the remainder
                du = (U2 - U1) / 37.0;
                dv = (V2 - V1) / 37.0;
                if (du < 1e-12)
                {
                    du = 1e-12;
                }

                if (dv < 1e-12)
                {
                    dv = 1e-12;
                }

                for (u = du + U1; u < U2; u += du)
                {
                    for (v = dv + V1; v < V2; v += dv)
                    {
                        if (++NbPntCalc >= IndexPoint)
                        {
                            if (ClassifyUVPoint(TheIntersector, surf, new Point2D(u, v)) == TopAbs_IN)
                            {
                                u_ = u;
                                v_ = v;
                                surf.D1(u, v, out APoint_, out theVecD1U, out theVecD1V);
                                IndexPoint = NbPntCalc;
                                return (true);
                            }
                        }
                    }
                }

                u = (U1 + U2) * 0.5;
                v = (V1 + V2) * 0.5;
                if (++NbPntCalc >= IndexPoint)
                {
                    if (ClassifyUVPoint(TheIntersector, surf, new Point2D(u, v)) == TopAbs_IN)
                    {
                        u_ = u;
                        v_ = v;
                        surf.D1(u, v, out APoint_, out theVecD1U, out theVecD1V);
                        IndexPoint = NbPntCalc;
                        return (true);
                    }
                }
            }

            IndexPoint = NbPntCalc;
        }
        else
        {
            // printf("BRepClass3d_SolidExplorer Face not found ds the map \n");
        }

        return FindAPointInTheFace(Face,
                                   ref APoint_,
                                   ref u_,
                                   ref v_,
                                   ref param_,
                                   ref theVecD1U,
                                   ref theVecD1V);
    }

    private static void LimitInfiniteUV(ref double U1, ref double V1, ref double U2, ref double V2)
    {
        bool infU1 = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(U1), infV1 = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(V1),
             infU2 = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(U2), infV2 = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(V2);

        if (infU1)
        {
            U1 = -1e10;
        }

        if (infV1)
        {
            V1 = -1e10;
        }

        if (infU2)
        {
            U2 = 1e10;
        }

        if (infV2)
        {
            V2 = 1e10;
        }
    }

    private static int IsInfiniteUV(double U1, double V1, double U2, double V2)
    {
        int aVal = 0;

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(U1))
        {
            aVal |= 1;
        }

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(V1))
        {
            aVal |= 2;
        }

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(U2))
        {
            aVal |= 4;
        }

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(V2))
        {
            aVal |= 8;
        }

        return aVal;
    }

    /// <summary>
    /// A line <paramref name="L"/> from <paramref name="P"/> that meets the shape's boundary, with in
    /// <paramref name="_Par"/> the parameter up to which to intersect: the first call aims at a point of the first
    /// face, the next at the second face, and so on. Returns 0 for a line, 1 if the point is on a face (or the face
    /// is infinite), 2 for a degenerate face (from a bad shell), 3 if the point is on the surface but outside the face.
    /// </summary>
    public int OtherSegment(in Point3D P, ref Line3D L, ref double _Par)
    {
        double TolU = global::Kerf.Engine.Foundation.Precision.PConfusion();
        double TolV = TolU;

        TopoDS_Face face;
        var faceexplorer = new TopExp_Explorer();
        Point3D APoint = new();
        Vector3D aVecD1U = new(), aVecD1V = new();
        double maxscal = 0;
        bool ptfound = false;
        double Par;
        double _u, _v;
        int IndexPoint = 0;
        int NbPointsOK = 0;
        int NbFacesInSolid = 0;
        bool aRestr = true;
        bool aTestInvert = false;

        for (;;)
        {
            myFirstFace++;
            faceexplorer.Init(myShape, TopAbs_FACE);
            // look for point on face starting from myFirstFace
            //  Modified by skv - Thu Sep  4 14:31:12 2003 OCC578 Begin
            //     while (faceexplorer.More()) {
            NbFacesInSolid = 0;
            for (; faceexplorer.More(); faceexplorer.Next())
            {
                //  Modified by skv - Thu Sep  4 14:31:12 2003 OCC578 End
                NbFacesInSolid++;
                if (myFirstFace > NbFacesInSolid)
                {
                    continue;
                }

                face = TopoDS.Face(faceexplorer.Current());

                var surf = new BRepAdaptor_Surface();
                if (aTestInvert)
                {
                    var aClass = new BRepTopAdaptor_FClass2d(face, global::Kerf.Engine.Foundation.Precision.Confusion());
                    if (aClass.PerformInfinitePoint() == TopAbs_IN)
                    {
                        aRestr = false;
                        if (myMapOfInter.IsBound(face))
                        {
                            myMapOfInter.UnBind(face);
                            var ptr = new IntCurvesFace_Intersector(face, global::Kerf.Engine.Foundation.Precision.Confusion(), aRestr, false);
                            myMapOfInter.Bind(face, ptr);
                        }
                    }
                    else
                    {
                        aRestr = true;
                    }
                }

                surf.Initialize(face, aRestr);
                double U1, V1, U2, V2;
                U1 = surf.FirstUParameter();
                V1 = surf.FirstVParameter();
                U2 = surf.LastUParameter();
                V2 = surf.LastVParameter();
                face.Orientation(TopAbs_FORWARD);
                //
                // avoid process faces from uncorrected shells
                double eps = global::Kerf.Engine.Foundation.Precision.PConfusion();
                double epsU = Max(eps * Max(Math.Abs(U2), Math.Abs(U1)), eps);
                double epsV = Max(eps * Max(Math.Abs(V2), Math.Abs(V1)), eps);
                if (Math.Abs(U2 - U1) < epsU || Math.Abs(V2 - V1) < epsV)
                {
                    return 2;
                }

                //
                double svmyparam = myParamOnEdge;
                //
                // Check if the point is on the face or the face is infinite.
                int anInfFlag = IsInfiniteUV(U1, V1, U2, V2);
                // default values
                _u = (U1 + U2) * 0.5;
                _v = (V1 + V2) * 0.5;

                var GA = new GeomAdaptor_Surface(BRep_Tool.Surface(face)!);
                var Ext = new Extrema_ExtPS(P, GA, TolU, TolV, Extrema_ExtFlag_MIN);
                //
                if (Ext.IsDone() && Ext.NbExt() > 0)
                {
                    int i, iNear, iEnd;
                    double aUx, aVx, Dist2, Dist2Min;
                    Extrema_POnSurf aPx;
                    //
                    iNear = 1;
                    Dist2Min = Ext.SquareDistance(1);
                    iEnd = Ext.NbExt();
                    for (i = 2; i <= iEnd; i++)
                    {
                        aPx = Ext.Point(i);
                        aPx.Parameter(out aUx, out aVx);
                        if (aUx >= U1 && aUx <= U2 && aVx >= V1 && aVx <= V2)
                        {
                            Dist2 = Ext.SquareDistance(i);
                            if (Dist2 < Dist2Min)
                            {
                                Dist2Min = Dist2;
                                iNear = i;
                            }
                        }
                    }

                    //
                    double aDist2Tresh = 1.0e-24;
                    //
                    if (Dist2Min < aDist2Tresh)
                    {
                        if (anInfFlag != 0)
                        {
                            return 1;
                        }
                        else
                        {
                            var classifier2d = new BRepClass_FaceClassifier();
                            (Ext.Point(iNear)).Parameter(out double aU, out double aV);

                            Point2D aPuv = new(aU, aV);

                            classifier2d.Perform(face, aPuv, global::Kerf.Engine.Foundation.Precision.PConfusion());

                            TopAbs_State aState = classifier2d.State();

                            if (aState == TopAbs_IN || aState == TopAbs_ON)
                            {
                                return 1;
                            }
                            else
                            {
                                return 3; // skv - the point is on surface but outside face.
                            }
                        }
                    }

                    if (anInfFlag != 0)
                    {
                        APoint = (Ext.Point(iNear)).Value();
                        Vector3D V = new(P, APoint);
                        _Par = V.Magnitude();
                        L = new Line3D(P, new Direction3D(V));
                        ptfound = true;
                        return 0;
                    }

                    // set the parameters found by extrema
                    aPx = Ext.Point(iNear);
                    aPx.Parameter(out _u, out _v);
                    APoint = aPx.Value();
                }

                // The point is not ON the face or surface. The face is restricted.
                //  find point in a face not too far from a projection of P on face
                do
                {
                    ++IndexPoint;
                    if (PointInTheFace(face,
                                       ref APoint,
                                       ref _u,
                                       ref _v,
                                       ref myParamOnEdge,
                                       ref IndexPoint,
                                       surf,
                                       U1,
                                       V1,
                                       U2,
                                       V2,
                                       ref aVecD1U,
                                       ref aVecD1V))
                    {
                        ++NbPointsOK;
                        Vector3D V = new(P, APoint);
                        Par = V.Magnitude();
                        if (Par > Constants.Resolution() && aVecD1U.Magnitude() > Constants.Resolution()
                            && aVecD1V.Magnitude() > Constants.Resolution())
                        {
                            Vector3D Norm = aVecD1U.Crossed(aVecD1V);
                            double tt = Norm.Magnitude();
                            if (tt > Constants.Resolution())
                            {
                                tt = Math.Abs(Norm.Dot(V)) / (tt * Par);
                                if (tt > maxscal)
                                {
                                    maxscal = tt;
                                    L = new Line3D(P, new Direction3D(V));
                                    _Par = Par;
                                    ptfound = true;
                                    if (maxscal > 0.2)
                                    {
                                        myParamOnEdge = svmyparam;
                                        return 0;
                                    }
                                }
                            }
                        }
                    }
                } while (IndexPoint < 200 && NbPointsOK < 16);

                myParamOnEdge = svmyparam;
                if (maxscal > 0.2)
                {
                    return 0;
                }

                //  Modified by skv - Thu Sep  4 14:32:14 2003 OCC578 Begin
                // Next is done in the main for(..) loop.
                //       faceexplorer.Next();
                //  Modified by skv - Thu Sep  4 14:32:14 2003 OCC578 End
                IndexPoint = 0;

                bool encoreuneface = faceexplorer.More();
                if (!ptfound && !encoreuneface)
                {
                    if (myParamOnEdge < 0.0001)
                    {
                        //-- This case takes place when the point is on the solid
                        //-- and this solid is reduced to a face
                        Point3D PBidon = new(P.X() + 1.0, P.Y(), P.Z());
                        Vector3D V = new(P, PBidon);
                        Par = 1.0;
                        _Par = Par;
                        L = new Line3D(P, new Direction3D(V));
                        return 0;
                    }
                }
            } //-- Exploration of the faces

            if (NbFacesInSolid == 0)
            {
                _Par = 0.0;
                myReject = true;
                return 0;
            }

            if (ptfound)
            {
                return 0;
            }

            myFirstFace = 0;
            if (myParamOnEdge == 0.512345)
            {
                myParamOnEdge = 0.4;
            }
            else if (myParamOnEdge == 0.4)
            {
                myParamOnEdge = 0.6;
            }
            else if (myParamOnEdge == 0.6)
            {
                myParamOnEdge = 0.3;
            }
            else if (myParamOnEdge == 0.3)
            {
                myParamOnEdge = 0.7;
            }
            else if (myParamOnEdge == 0.7)
            {
                myParamOnEdge = 0.2;
            }
            else if (myParamOnEdge == 0.2)
            {
                myParamOnEdge = 0.8;
            }
            else if (myParamOnEdge == 0.8)
            {
                myParamOnEdge = 0.1;
            }
            else if (myParamOnEdge == 0.1)
            {
                myParamOnEdge = 0.9;
                //
            }
            else
            {
                myParamOnEdge *= 0.5;
                if (myParamOnEdge < 0.0001)
                {
                    Point3D PBidon = new(P.X() + 1.0, P.Y(), P.Z());
                    Vector3D V = new(P, PBidon);
                    Par = 1.0;
                    _Par = Par;
                    L = new Line3D(P, new Direction3D(V));
                    return 0;
                }
            }

            aTestInvert = true;
        } //-- for(;;) { ...  }
    }

    //  Modified by skv - Thu Sep  4 12:30:14 2003 OCC578 Begin

    /// <summary>The index of the face the last line was computed for.</summary>
    public int GetFaceSegmentIndex() => myFirstFace;

    //  Modified by skv - Thu Sep  4 12:30:14 2003 OCC578 End

    /// <summary>A point in the face, searched on a grid of its whole surface (infinite bounds limited to 1e10).</summary>
    public bool PointInTheFace(in TopoDS_Face _face,
                               ref Point3D APoint_,
                               ref double u_,
                               ref double v_,
                               ref double param_,
                               ref int IndexPoint)
    {
        TopoDS_Face Face = _face;
        Face.Orientation(TopAbs_FORWARD);
        var surf = new BRepAdaptor_Surface();
        surf.Initialize(Face);
        double U1, V1, U2, V2; //,u,v;
        U1 = surf.FirstUParameter();
        V1 = surf.FirstVParameter();
        U2 = surf.LastUParameter();
        V2 = surf.LastVParameter();
        LimitInfiniteUV(ref U1, ref V1, ref U2, ref V2);
        return (PointInTheFace(Face, ref APoint_, ref u_, ref v_, ref param_, ref IndexPoint, surf, U1, V1, U2, V2));
    }

    /// <summary>A point in the face with its parameters, the search initialised with 0.1.</summary>
    public static bool FindAPointInTheFace(in TopoDS_Face _face, ref Point3D APoint_, ref double u_, ref double v_)
    {
        double param = 0.1;
        bool r = FindAPointInTheFace(_face, ref APoint_, ref u_, ref v_, ref param);
        return r;
    }

    /// <summary>A point in the face, the search initialised with 0.1.</summary>
    public static bool FindAPointInTheFace(in TopoDS_Face _face, ref Point3D APoint_)
    {
        double u = 0.0, v = 0.0;
        bool r = FindAPointInTheFace(_face, ref APoint_, ref u, ref v);
        return r;
    }

    /// <summary>The parameters of a point in the face, the search initialised with 0.1.</summary>
    public static bool FindAPointInTheFace(in TopoDS_Face _face, ref double u_, ref double v_)
    {
        Point3D APoint = new();
        bool r = FindAPointInTheFace(_face, ref APoint, ref u_, ref v_);
        return r;
    }

    /// <summary>An empty explorer.</summary>
    public BRepClass3d_SolidExplorer()
    {
        myReject = true;
        myFirstFace = 0;
        myParamOnEdge = 0.0;
    }

    /// <summary>An explorer of the shape <paramref name="S"/>.</summary>
    public BRepClass3d_SolidExplorer(in TopoDS_Shape S)
    {
        InitShape(S);
    }

    /// <summary>Forgets the faces' intersectors (OCCT's Destroy, also called by its destructor).</summary>
    public void Destroy()
    {
        myMapOfInter.Clear();
    }

    /// <summary>
    /// Loads the shape <paramref name="S"/>: an intersector for each face, and the tree of the boxes of the edges
    /// and vertices of its FORWARD and REVERSED faces and edges (degenerated edges left out).
    /// </summary>
    public void InitShape(in TopoDS_Shape S)
    {
        myMapEV.Clear();
        myTree.Clear();

        myShape = S;
        myFirstFace = 0;
        myParamOnEdge = 0.512345;
        //-- Exploring of the Map and removal of allocated objects

        myMapOfInter.Clear();

        myReject = true; //-- case of infinite solid (without any face)

        var Expl = new TopExp_Explorer();
        for (Expl.Init(S, TopAbs_FACE); Expl.More(); Expl.Next())
        {
            TopoDS_Face Face = TopoDS.Face(Expl.Current());
            var ptr = new IntCurvesFace_Intersector(Face, global::Kerf.Engine.Foundation.Precision.Confusion(), true, false);
            myMapOfInter.Bind(Face, ptr);
            myReject = false; //-- at least one face in the solid
        }

        // since the internal/external parts should be avoided in tree filler,
        // there is no need to add these parts in the EV map as well
        var aExpF = new TopExp_Explorer(myShape, TopAbs_FACE);
        for (; aExpF.More(); aExpF.Next())
        {
            TopoDS_Shape aF = aExpF.Current();
            //
            TopAbs_Orientation anOrF = aF.Orientation();
            if (anOrF == TopAbs_INTERNAL || anOrF == TopAbs_EXTERNAL)
            {
                continue;
            }

            //
            var aExpE = new TopExp_Explorer(aF, TopAbs_EDGE);
            for (; aExpE.More(); aExpE.Next())
            {
                TopoDS_Shape aE = aExpE.Current();
                //
                TopAbs_Orientation anOrE = aE.Orientation();
                if (anOrE == TopAbs_INTERNAL || anOrE == TopAbs_EXTERNAL)
                {
                    continue;
                }

                //
                if (BRep_Tool.Degenerated(TopoDS.Edge(aE)))
                {
                    continue;
                }

                //
                TopExp.MapShapes(aE, myMapEV);
            }
        }

        //
        // Fill mapEV with vertices and edges from shape
        var aTreeFiller = new SpatialTreeFiller<int, Box>(myTree);
        //
        int i, aNbEV = myMapEV.Extent();
        for (i = 1; i <= aNbEV; ++i)
        {
            TopoDS_Shape aS = myMapEV[i];
            //
            Box aBox = new();
            BRepBndLib.Add(aS, ref aBox);
            aTreeFiller.Add(i, aBox);
        }

        aTreeFiller.Fill();
    }

    /// <summary>True if the point is outside a bounding volume of the shape: true only for a shape without faces.</summary>
    public virtual bool Reject(in Point3D P) => (myReject); // case of solid without face

    /// <summary>Starts an exploration of the shells.</summary>
    public void InitShell() => myShellExplorer.Init(myShape, TopAbs_SHELL);

    /// <summary>True if there is a current shell.</summary>
    public bool MoreShell() => (myShellExplorer.More());

    /// <summary>Moves to the next shell.</summary>
    public void NextShell() => myShellExplorer.Next();

    /// <summary>The current shell.</summary>
    public TopoDS_Shell CurrentShell() => (TopoDS.Shell(myShellExplorer.Current()));

    /// <summary>True if the shell is rejected: always false.</summary>
    public virtual bool RejectShell(in Line3D L) => (false);

    /// <summary>Starts an exploration of the faces of the current shell.</summary>
    public void InitFace() => myFaceExplorer.Init(TopoDS.Shell(myShellExplorer.Current()), TopAbs_FACE);

    /// <summary>True if there is a current face in the current shell.</summary>
    public bool MoreFace() => (myFaceExplorer.More());

    /// <summary>Moves to the next face of the current shell.</summary>
    public void NextFace() => myFaceExplorer.Next();

    /// <summary>The current face.</summary>
    public TopoDS_Face CurrentFace() => (TopoDS.Face(myFaceExplorer.Current()));

    /// <summary>True if the face is rejected: always false.</summary>
    public virtual bool RejectFace(in Line3D L) => (false);

    /// <summary>A line from <paramref name="P"/> to the boundary, starting again from the first face (see <see cref="OtherSegment"/>).</summary>
    public int Segment(in Point3D P, ref Line3D L, ref double Par)
    {
        int bRetFlag;
        myFirstFace = 0;
        bRetFlag = OtherSegment(P, ref L, ref Par);
        return bRetFlag;
    }

    /// <summary>The intersector of the face <paramref name="F"/>.</summary>
    /// <exception cref="NoSuchObjectException">The face is not in the shape.</exception>
    public IntCurvesFace_Intersector Intersector(in TopoDS_Face F)
    {
        IntCurvesFace_Intersector? ptr = myMapOfInter.Find(F);
        IntCurvesFace_Intersector curr = ptr!;
        return curr;
    }

    /// <summary>Does nothing (OCCT's debug output).</summary>
    public virtual void DumpSegment(in Point3D P, in Line3D L, double Par, TopAbs_State S)
    {
    }

    /// <summary>The shape.</summary>
    public TopoDS_Shape GetShape() => (myShape);

    /// <summary>The tree of the boxes of the edges and vertices (objects are indices in <see cref="GetMapEV"/>).</summary>
    public SpatialTree<int, Box> GetTree() => myTree;

    /// <summary>The edges and vertices of the shape.</summary>
    public NCollection_IndexedMap<TopoDS_Shape> GetMapEV() => myMapEV;
}
