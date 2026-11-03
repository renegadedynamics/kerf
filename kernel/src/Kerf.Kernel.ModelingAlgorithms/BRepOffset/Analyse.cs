// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Analyse.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Analyse.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.Data.TypeOfConcavity;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Offsets;

/// <summary>
/// Analyses a shape to find the parts of its edges connecting convex, concave or tangent faces.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21): the lists it returns are its own, as OCCT's const references are. It never writes
/// into the analysed shape itself, and its results hold only the shape's own sub-shapes, but for the tangent faces'
/// treatment below.
/// </para>
/// <para>
/// The treatment of tangent faces with different offset values (<c>TreatTangentFaces</c>) runs only with a face-offset
/// map, which only the 3D offset sets: <see cref="SetFaceOffsetMap"/> and <see cref="SetOffsetValue"/> are internal.
/// It builds prisms on the tangent edges (<see cref="NewFaces"/>, <see cref="Generated"/>, <see cref="EdgeReplacement"/>),
/// whose sweep writes pcurves into those edges, as OCCT's does; BRepOffset_MakeOffset analyses its own copy of the
/// caller's shape (the offset family's copy at entry, unit 7E), so the edges written are the copy's. On a frozen shape
/// BRepPrimAPI_MakePrism sweeps a copy of the edges instead (its own copy at entry), so the shape is never written, but
/// the new faces then hold copies of the tangent edges where OCCT's hold the edges themselves.
/// </para>
/// </remarks>
public sealed class Analyse
{
    // Inputs
    private Shape _shape = new(); // Input shape to analyze
    private double _angle;               // Criteria angle to check tangency

    private double _offset; // Offset value
    // Map to store offset values for the faces. Should be set by the calling algorithm.
    private Dictionary<Shape, double> _faceOffsetMap = new(ShapeHasher.Instance);

    // Results
    private bool _done; // Status of the algorithm

    // Map containing the list of intervals on the edge
    private readonly Dictionary<Shape, NCollection_List<Interval>> _mapEdgeType = new(ShapeHasher.Instance);
    // Ancestors map
    private readonly NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> _ancestors = new(ShapeHasher.Instance);
    // Replacement of an edge in the face
    private readonly Dictionary<Shape, Dictionary<Shape, Shape>> _replacement = new(ShapeHasher.Instance);
    // Map of shapes descendants built on the base of Ancestors map. Filled on the first query.
    private readonly Dictionary<Shape, NCollection_List<Shape>> _descendants = new(ShapeHasher.Instance);

    // New faces generated to close the gaps between adjacent tangential faces having different offset values
    private readonly NCollection_List<Shape> _newFaces = new();
    // Binding between edge and face generated from the edge
    private readonly Dictionary<Shape, Shape> _generated = new(ShapeHasher.Instance);

    private static void CorrectOrientationOfTangent(ref Vector3D tangVec, in Vertex aVertex, in Edge anEdge)
    {
        Vertex vlast = Exploration.LastVertex(anEdge);
        if (aVertex.IsSame(vlast))
        {
            tangVec = tangVec.Reversed();
        }
    }

    //=================================================================================================

    /// <summary>An empty analysis.</summary>
    public Analyse()
    {
        _offset = 0.0;
        _done = false;
    }

    //=================================================================================================

    /// <summary>Analyses <paramref name="s"/> with the tangency angle <paramref name="angle"/>.</summary>
    public Analyse(in Shape s, double angle)
    {
        _offset = 0.0;
        _done = false;
        Perform(s, angle);
    }

    //=================================================================================================

    private static void EdgeAnalyse(in Edge e, in Face f1, in Face f2, double sinTol,
                                    NCollection_List<Interval> li)
    {
        Tool.Range(e, f1, out double f, out double l);
        Interval i = new();
        i.First(f);
        i.Last(l);
        //
        FaceSurfaceAdaptor bAsurf1 = new(f1, false);
        SurfaceKind surfType1 = bAsurf1.GetType();

        FaceSurfaceAdaptor bAsurf2 = new(f2, false);
        SurfaceKind surfType2 = bAsurf2.GetType();

        bool isTwoPlanes = (surfType1 == Plane && surfType2 == Plane);

        TypeOfConcavity connectType = TypeOfConcavity.Other;

        if (isTwoPlanes) // then use only strong condition
        {
            if (Tool.Continuity(e, f1, f2) > C0)
            {
                connectType = TypeOfConcavity.Tangential;
            }
            else
            {
                connectType = global::Kerf.Engine.Fillets.ChFi3d.DefineConnectType(e, f1, f2, sinTol, false);
            }
        }
        else
        {
            bool isTwoSplines =
                (surfType1 == SurfaceKind.BSplineSurface || surfType1 == SurfaceKind.BezierSurface)
                && (surfType2 == SurfaceKind.BSplineSurface || surfType2 == SurfaceKind.BezierSurface);
            bool isMixedConcavity = false;
            if (isTwoSplines)
            {
                double angTol = 0.1;
                isMixedConcavity = CheckMixedContinuity(e, f1, f2, angTol);
            }

            if (!isMixedConcavity)
            {
                if (global::Kerf.Engine.Fillets.ChFi3d.IsTangentFaces(e, f1, f2)) // weak condition
                {
                    connectType = TypeOfConcavity.Tangential;
                }
                else
                {
                    connectType = global::Kerf.Engine.Fillets.ChFi3d.DefineConnectType(e, f1, f2, sinTol, false);
                }
            }
            else
            {
                connectType = TypeOfConcavity.Mixed;
            }
        }

        i.Type(connectType);
        li.Append(i);
    }

    //=================================================================================================

    private static bool CheckMixedContinuity(in Edge edge, in Face face1, in Face face2,
                                             double angTol)
    {
        bool mixedCont = false;
        Continuity currOrder = Tool.Continuity(edge, face1, face2);
        if (currOrder > C0)
        {
            // Method BRep_Tool::Continuity(...) always returns minimal continuity between faces
            // so, if aCurrOrder > C0 it means that faces are tangent along whole edge.
            return mixedCont;
        }
        // But we cannot trust result, if it is C0, because this value is set by default.
        double tolC0 = Max(0.001, 1.5 * Tool.Tolerance(edge));

        double first;
        double last;

        Curve2D? c2d1, c2d2;

        if (!face1.IsSame(face2) && Tool.IsClosed(edge, face1)
            && Tool.IsClosed(edge, face2))
        {
            // Find the edge in the face 1: this edge will have correct orientation
            Edge edgeInFace1 = new();
            Face aFace1 = face1;
            aFace1.Orientation(Forward);
            Explorer explo = new(aFace1, ShapeType.Edge);
            for (; explo.More(); explo.Next())
            {
                Edge anEdge = ShapeCasts.Edge(explo.Current());
                if (anEdge.IsSame(edge))
                {
                    edgeInFace1 = anEdge;
                    break;
                }
            }
            if (edgeInFace1.IsNull)
            {
                return mixedCont;
            }

            c2d1 = Tool.CurveOnSurface(edgeInFace1, aFace1, out first, out last);
            Face aFace2 = face2;
            aFace2.Orientation(Forward);
            edgeInFace1.Reverse();
            c2d2 = Tool.CurveOnSurface(edgeInFace1, aFace2, out first, out last);
        }
        else
        {
            // Obtaining of pcurves of edge on two faces.
            c2d1 = Tool.CurveOnSurface(edge, face1, out first, out last);
            // For the case of seam edge
            Edge ee = edge;
            if (face1.IsSame(face2))
            {
                ee.Reverse();
            }
            c2d2 = Tool.CurveOnSurface(ee, face2, out first, out last);
        }

        if (c2d1 is null || c2d2 is null)
        {
            return mixedCont;
        }

        // Obtaining of two surfaces from adjacent faces.
        Surface? surf1 = Tool.Surface(face1);
        Surface? surf2 = Tool.Surface(face2);

        if (surf1 is null || surf2 is null)
        {
            return mixedCont;
        }

        int nbSamples = 23;

        // Check for mixed concavity: convex in some regions, concave in others.
        double delta = (last - first) / (nbSamples - 1);
        bool hasConvex = false;
        bool hasConcave = false;
        int nbValid = 0;

        for (int i = 1; i <= nbSamples; i++)
        {
            double par = (i == nbSamples) ? last : first + (i - 1) * delta;

            SurfaceContinuity cont = new(c2d1, c2d2, par, surf1, surf2, G1, 0.001, tolC0,
                                                        angTol, angTol, angTol);
            if (!cont.IsDone)
            {
                continue;
            }

            nbValid++;

            if (!cont.IsG1() && (!hasConvex || !hasConcave))
            {
                double angle = cont.C0Value();
                hasConvex = hasConvex || (angle > M_PI_2 + angTol);
                hasConcave = hasConcave || (angle < M_PI_2 - angTol);
            }
        }

        if (nbValid < nbSamples / 2)
        {
            return mixedCont;
        }

        // Mixed connectivity: both convex and concave regions exist.
        mixedCont = hasConvex && hasConcave;

        return mixedCont;
    }

    //=================================================================================================

    private static void BuildAncestors(in Shape s, NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> ma)
    {
        ma.Clear();
        Exploration.MapShapesAndUniqueAncestors(s, ShapeType.Vertex, ShapeType.Edge, ma);
        Exploration.MapShapesAndUniqueAncestors(s, ShapeType.Edge, ShapeType.Face, ma);
    }

    //=================================================================================================

    /// <summary>Performs the analysis of <paramref name="s"/> with the tangency angle <paramref name="angle"/>.</summary>
    public void Perform(in Shape s, double angle, ProgressRange range = default)
    {
        Shape S = s;
        double Angle = angle;
        _shape = S;
        _newFaces.Clear();
        _generated.Clear();
        _replacement.Clear();
        _descendants.Clear();

        _angle = Angle;
        double sinTol = Math.Abs(Math.Sin(Angle));

        // Build ancestors.
        BuildAncestors(S, _ancestors);

        NCollection_List<Shape> leTang = new();
        Explorer exp = new(S.Oriented(Forward), ShapeType.Edge);
        using ProgressScope psOuter = new(range, null, 2);
        using ProgressScope ps = new(psOuter.Next(), "Performing edges analysis", 1, true);
        for (; exp.More(); exp.Next(), ps.Next())
        {
            if (!ps.More())
            {
                return;
            }
            Edge e = ShapeCasts.Edge(exp.Current());
            if (!_mapEdgeType.ContainsKey(e))
            {
                NCollection_List<Interval> li = new();
                _mapEdgeType[e] = li;

                NCollection_List<Shape> l = Ancestors(e);
                if (l.IsEmpty())
                {
                    continue;
                }

                if (l.Extent() == 2)
                {
                    Face f1 = ShapeCasts.Face(l.First());
                    Face f2 = ShapeCasts.Face(l.Last());
                    EdgeAnalyse(e, f1, f2, sinTol, _mapEdgeType.GetValue(e));

                    // For tangent faces add artificial perpendicular face
                    // to close the gap between them (if they have different offset values)
                    if (_mapEdgeType.GetValue(e).Last().Type() == TypeOfConcavity.Tangential)
                    {
                        leTang.Append(e);
                    }
                }
                else if (l.Extent() == 1)
                {
                    Face f = ShapeCasts.Face(l.First());
                    Tool.Range(e, f, out double u1, out double u2);
                    Interval inter = new(u1, u2, TypeOfConcavity.Other);

                    if (!BrepTools.IsReallyClosed(e, f))
                    {
                        inter.Type(TypeOfConcavity.FreeBound);
                    }
                    _mapEdgeType.GetValue(e).Append(inter);
                }
                else
                {
                    // edge shared by more than two faces (OCCT_DEBUG trace left out)
                }
            }
        }

        TreatTangentFaces(leTang, psOuter.Next());
        if (!psOuter.More())
        {
            return;
        }
        _done = true;
    }

    //=================================================================================================

    // Treatment of the tangential cases: theLE are the edges connecting tangent faces
    private void TreatTangentFaces(NCollection_List<Shape> le, ProgressRange range)
    {
        if (le.IsEmpty() || _faceOffsetMap.Count == 0)
        {
            // Noting to do: either there are no tangent faces in the shape or
            //               the face offset map has not been provided
            return;
        }

        // Select the edges which connect faces with different offset values
        Compound ceTangent = new();
        new BrepBuilder().MakeCompound(ref ceTangent);
        // Bind to each tangent edge a max offset value of its faces
        Dictionary<Shape, double> edgeOffsetMap = new(ShapeHasher.Instance);
        // Bind vertices of the tangent edges with connected edges
        // of the face with smaller offset value
        Dictionary<Shape, Shape> dmveMin = new(ShapeHasher.Instance);
        using ProgressScope psOuter = new(range, null, 3);
        using ProgressScope ps1 = new(psOuter.Next(), "Binding vertices with connected edges", le.Extent());
        for (NCollection_List<Shape>.Iterator it = new(le); it.More(); it.Next(), ps1.Next())
        {
            if (!ps1.More())
            {
                return;
            }
            Shape e = it.Value();
            NCollection_List<Shape> la = Ancestors(e);

            Shape f1 = la.First(), f2 = la.Last();

            bool pOffsetVal1 = _faceOffsetMap.TryGetValue(f1, out double offsetFound1);
            bool pOffsetVal2 = _faceOffsetMap.TryGetValue(f2, out double offsetFound2);
            double offsetVal1 = pOffsetVal1 ? Math.Abs(offsetFound1) : _offset;
            double offsetVal2 = pOffsetVal2 ? Math.Abs(offsetFound2) : _offset;
            if (offsetVal1 != offsetVal2)
            {
                new BrepBuilder().Add(ceTangent, e);
                edgeOffsetMap[e] = Max(offsetVal1, offsetVal2);

                Shape fMin = offsetVal1 < offsetVal2 ? f1 : f2;
                for (Iterator itV = new(e); itV.More(); itV.Next())
                {
                    Shape v = itV.Value();
                    if (Ancestors(v).Extent() == 3)
                    {
                        for (Explorer expE = new(fMin, ShapeType.Edge); expE.More(); expE.Next())
                        {
                            Shape eMin = expE.Current();
                            if (eMin.IsSame(e))
                            {
                                continue;
                            }
                            for (Iterator itV1 = new(eMin); itV1.More(); itV1.Next())
                            {
                                Shape vx = itV1.Value();
                                if (v.IsSame(vx))
                                {
                                    dmveMin[v] = eMin;
                                }
                            }
                        }
                    }
                }
            }
        }

        if (edgeOffsetMap.Count == 0)
        {
            return;
        }

        // Create map of Face ancestors for the vertices on tangent edges
        Dictionary<Shape, NCollection_List<Shape>> dmvfAnc = new(ShapeHasher.Instance);

        using ProgressScope ps2 = new(psOuter.Next(), "Creating map of Face ancestors", le.Extent());
        for (NCollection_List<Shape>.Iterator itE = new(le); itE.More(); itE.Next(), ps2.Next())
        {
            if (!ps2.More())
            {
                return;
            }
            Shape e = itE.Value();
            if (!edgeOffsetMap.ContainsKey(e))
            {
                continue;
            }

            HashSet<Shape> mFence = new(ShapeHasher.Instance);
            {
                NCollection_List<Shape> lea = Ancestors(e);
                for (NCollection_List<Shape>.Iterator itLEA = new(lea); itLEA.More(); itLEA.Next())
                {
                    mFence.Add(itLEA.Value());
                }
            }

            for (Iterator itV = new(e); itV.More(); itV.Next())
            {
                Shape v = itV.Value();
                NCollection_List<Shape> pLFA = dmvfAnc.SetValueRef(v, new NCollection_List<Shape>());
                NCollection_List<Shape> lva = Ancestors(v);
                for (NCollection_List<Shape>.Iterator itLVA = new(lva); itLVA.More(); itLVA.Next())
                {
                    Edge ea = ShapeCasts.Edge(itLVA.Value());
                    if (!_mapEdgeType.TryGetValue(ea, out NCollection_List<Interval>? pIntervals)
                        || pIntervals.IsEmpty())
                    {
                        continue;
                    }
                    if (pIntervals.First().Type() == TypeOfConcavity.Tangential)
                    {
                        continue;
                    }

                    NCollection_List<Shape> lea = Ancestors(ea);
                    for (NCollection_List<Shape>.Iterator itLEA = new(lea); itLEA.More(); itLEA.Next())
                    {
                        Shape fa = itLEA.Value();
                        if (mFence.Add(fa))
                        {
                            pLFA.Append(fa);
                        }
                    }
                }
            }
        }

        IntersectionContext ctx = new();
        // Tangency criteria
        double sinTol = Math.Abs(Math.Sin(_angle));

        // Make blocks of connected edges
        NCollection_List<NCollection_List<Shape>> lcb = new();
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mveMap = new(ShapeHasher.Instance);

        AlgoTools.MakeConnexityBlocks(ceTangent, ShapeType.Vertex, ShapeType.Edge, lcb, mveMap);

        // Analyze each block to find co-planar edges
        using ProgressScope ps3 = new(psOuter.Next(), "Analyzing blocks to find co-planar edges", lcb.Extent());
        for (NCollection_List<NCollection_List<Shape>>.Iterator itLCB = new(lcb); itLCB.More();
             itLCB.Next(), ps3.Next())
        {
            if (!ps3.More())
            {
                return;
            }
            NCollection_List<Shape> cb = itLCB.Value();

            HashSet<Shape> mFence = new(ShapeHasher.Instance);
            for (NCollection_List<Shape>.Iterator itCB1 = new(cb); itCB1.More(); itCB1.Next())
            {
                Edge e1 = ShapeCasts.Edge(itCB1.Value());
                if (!mFence.Add(e1))
                {
                    continue;
                }

                Compound block = new();
                new BrepBuilder().MakeCompound(ref block);
                new BrepBuilder().Add(block, e1.Oriented(Forward));

                double offset = edgeOffsetMap.GetValue(e1);
                NCollection_List<Shape> lf1 = Ancestors(e1);

                AlgoTools3D.GetNormalToFaceOnEdge(e1, ShapeCasts.Face(lf1.First()), out Direction3D dn1);

                NCollection_List<Shape>.Iterator itCB2 = new();
                itCB2.Reset(cb, itCB1.Index); // itCB2 = itCB1
                for (itCB2.Next(); itCB2.More(); itCB2.Next())
                {
                    Edge e2 = ShapeCasts.Edge(itCB2.Value());
                    if (mFence.Contains(e2))
                    {
                        continue;
                    }

                    NCollection_List<Shape> lf2 = Ancestors(e2);

                    AlgoTools3D.GetNormalToFaceOnEdge(e2, ShapeCasts.Face(lf2.First()), out Direction3D dn2);

                    if (dn1.XYZ().Crossed(dn2.XYZ()).Modulus() < sinTol)
                    {
                        new BrepBuilder().Add(block, e2.Oriented(Forward));
                        mFence.Add(e2);
                        offset = Max(offset, edgeOffsetMap.GetValue(e2));
                    }
                }

                // Make the prism
                ApiMakePrism mp = new(block, new Vector3D(dn1.XYZ()) * offset);
                if (!mp.IsDone)
                {
                    continue;
                }

                Shape prism = mp.InternalShape();
                NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> prismAncestors = new(ShapeHasher.Instance);
                Exploration.MapShapesAndAncestors(prism, ShapeType.Edge, ShapeType.Face, prismAncestors);
                Exploration.MapShapesAndAncestors(prism, ShapeType.Vertex, ShapeType.Edge, prismAncestors);

                for (Iterator itE = new(block); itE.More(); itE.Next())
                {
                    Edge e = ShapeCasts.Edge(itE.Value());
                    NCollection_List<Shape> lg = mp.InternalGenerated(e);
                    Face fNew = ShapeCasts.Face(lg.First());

                    NCollection_List<Shape> la = _ancestors.ChangeFromKey(e);

                    Shape f1 = la.First();
                    Shape f2 = la.Last();

                    bool pOffsetVal1 = _faceOffsetMap.TryGetValue(f1, out double offsetFound1);
                    bool pOffsetVal2 = _faceOffsetMap.TryGetValue(f2, out double offsetFound2);
                    double offsetVal1 = pOffsetVal1 ? Math.Abs(offsetFound1) : _offset;
                    double offsetVal2 = pOffsetVal2 ? Math.Abs(offsetFound2) : _offset;

                    Shape fToRemove = offsetVal1 > offsetVal2 ? f1 : f2;
                    Shape fOpposite = offsetVal1 > offsetVal2 ? f2 : f1;

                    // Orient the face so its normal is directed to smaller offset face
                    {
                        // get normal of the new face
                        AlgoTools3D.GetNormalToFaceOnEdge(e, fNew, out Direction3D dn);

                        // get bi-normal for the aFOpposite
                        Edge eInF = new();
                        for (Explorer aExpE = new(fOpposite, ShapeType.Edge); aExpE.More(); aExpE.Next())
                        {
                            if (e.IsSame(aExpE.Current()))
                            {
                                eInF = ShapeCasts.Edge(aExpE.Current());
                                break;
                            }
                        }

                        Point2D p2d = new();
                        Point3D pInF = new();
                        Curve3D c3D = Tool.Curve(eInF, out double f, out double l)!;
                        Point3D pOnE = c3D.Value((f + l) / 2.0);
                        AlgoTools3D.PointNearEdge(eInF, ShapeCasts.Face(fOpposite), (f + l) / 2.0, 1.0e-5,
                                                           ref p2d, ref pInF);

                        Vector3D bn = new(pOnE, pInF);

                        if (bn.Dot(new Vector3D(dn)) < 0)
                        {
                            fNew.Reverse();
                        }
                    }

                    // Remove the face with bigger offset value from edge ancestors
                    for (NCollection_List<Shape>.Iterator itA = new(la); itA.More(); itA.Next())
                    {
                        if (itA.Value().IsSame(fToRemove))
                        {
                            la.Remove(itA);
                            break;
                        }
                    }
                    la.Append(fNew);

                    _mapEdgeType.GetValue(e).Clear();
                    // Analyze edge again
                    EdgeAnalyse(e, ShapeCasts.Face(fOpposite), fNew, sinTol, _mapEdgeType.GetValue(e));

                    // Analyze vertices
                    HashSet<Shape> fNewEdgeMap = new(ShapeHasher.Instance);
                    fNewEdgeMap.Add(e);
                    for (Iterator itV = new(e); itV.More(); itV.Next())
                    {
                        Shape v = itV.Value();
                        // Add Side edge to map of Ancestors with the correct orientation
                        Edge eg = ShapeCasts.Edge(mp.InternalGenerated(v).First());
                        _generated[v] = eg;
                        {
                            for (Explorer expEg = new(fNew, ShapeType.Edge); expEg.More(); expEg.Next())
                            {
                                if (expEg.Current().IsSame(eg))
                                {
                                    eg = ShapeCasts.Edge(expEg.Current());
                                    break;
                                }
                            }
                        }

                        if (dmveMin.ContainsKey(v))
                        {
                            if (dmvfAnc.TryGetValue(v, out NCollection_List<Shape>? pSA) && pSA.Extent() == 1)
                            {
                                // Adjust orientation of generated edge to its new ancestor
                                Edge eMin = ShapeCasts.Edge(dmveMin.GetValue(v));
                                for (Explorer expEx = new(pSA.First(), ShapeType.Edge); expEx.More(); expEx.Next())
                                {
                                    if (expEx.Current().IsSame(eMin))
                                    {
                                        eMin = ShapeCasts.Edge(expEx.Current());
                                        break;
                                    }
                                }

                                Orientation oriInEMin = Forward, oriInEG = Forward;

                                for (Iterator itx = new(eMin); itx.More(); itx.Next())
                                {
                                    if (itx.Value().IsSame(v))
                                    {
                                        oriInEMin = itx.Value().Orientation();
                                        break;
                                    }
                                }

                                for (Iterator itx = new(eg); itx.More(); itx.Next())
                                {
                                    if (itx.Value().IsSame(v))
                                    {
                                        oriInEG = itx.Value().Orientation();
                                        break;
                                    }
                                }

                                if (oriInEG == oriInEMin)
                                {
                                    eg.Reverse();
                                }
                            }
                        }

                        NCollection_List<Shape> lva = _ancestors.ChangeFromKey(v);
                        if (!lva.Contains(eg))
                        {
                            lva.Append(eg);
                        }
                        fNewEdgeMap.Add(eg);

                        NCollection_List<Shape> lega =
                            _ancestors[_ancestors.Add(eg, new NCollection_List<Shape>(prismAncestors.FindFromKey(eg)))];
                        {
                            // Add ancestors from the shape
                            if (dmvfAnc.TryGetValue(v, out NCollection_List<Shape>? pSA) && !pSA.IsEmpty())
                            {
                                NCollection_List<Shape> lsa = new(pSA);
                                lega.Append(lsa);
                            }
                        }

                        _mapEdgeType[eg] = new NCollection_List<Interval>();
                        if (lega.Extent() == 2)
                        {
                            EdgeAnalyse(eg, ShapeCasts.Face(lega.First()), ShapeCasts.Face(lega.Last()), sinTol,
                                        _mapEdgeType.GetValue(eg));
                        }
                    }

                    // Find an edge opposite to tangential one and add ancestors for it
                    Edge eOpposite = new();
                    for (Explorer anExpE = new(fNew, ShapeType.Edge); anExpE.More(); anExpE.Next())
                    {
                        if (!fNewEdgeMap.Contains(anExpE.Current()))
                        {
                            eOpposite = ShapeCasts.Edge(anExpE.Current());
                            break;
                        }
                    }

                    {
                        // Find it in aFOpposite
                        for (Explorer anExpE = new(fToRemove, ShapeType.Edge); anExpE.More(); anExpE.Next())
                        {
                            Shape eInFToRem = anExpE.Current();
                            if (e.IsSame(eInFToRem))
                            {
                                if (AlgoTools.IsSplitToReverse(eOpposite, eInFToRem, ctx))
                                {
                                    eOpposite.Reverse();
                                }
                                break;
                            }
                        }
                    }

                    NCollection_List<Shape> lfOpposite = new();
                    lfOpposite.Append(fNew);
                    lfOpposite.Append(fToRemove);
                    _ancestors.Add(eOpposite, lfOpposite);
                    _mapEdgeType[eOpposite] = new NCollection_List<Interval>();
                    EdgeAnalyse(eOpposite, fNew, ShapeCasts.Face(fToRemove), sinTol, _mapEdgeType.GetValue(eOpposite));

                    if (!_replacement.TryGetValue(fToRemove, out Dictionary<Shape, Shape>? pEEMap))
                    {
                        pEEMap = _replacement.SetValueRef(fToRemove,
                                                     new Dictionary<Shape, Shape>(ShapeHasher.Instance));
                    }
                    pEEMap[e] = eOpposite;

                    // Add ancestors for the vertices
                    for (Iterator itV = new(eOpposite); itV.More(); itV.Next())
                    {
                        Shape v = itV.Value();
                        NCollection_List<Shape> lva = prismAncestors.FindFromKey(v);
                        _ancestors.Add(v, new NCollection_List<Shape>(lva));
                    }

                    _newFaces.Append(fNew);
                    _generated[e] = fNew;
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>True if the analysis is done.</summary>
    public bool IsDone => _done;

    /// <summary>
    /// The replacement of the edge <paramref name="edge"/> in the face <paramref name="face"/>: the edge itself
    /// if it has none.
    /// </summary>
    public Edge EdgeReplacement(in Face face, in Edge edge)
    {
        if (!_replacement.TryGetValue(face, out Dictionary<Shape, Shape>? pEE))
        {
            return edge;
        }

        if (!pEE.TryGetValue(edge, out Shape pE))
        {
            return edge;
        }

        return ShapeCasts.Edge(pE);
    }

    //=================================================================================================

    /// <summary>
    /// The face generated for the edge connecting two tangent faces with different offset values, or the edge
    /// generated from a vertex of such an edge; a null shape if none.
    /// </summary>
    public Shape Generated(in Shape s)
    {
        return _generated.TryGetValue(s, out Shape pGenS) ? pGenS : new Shape();
    }

    /// <summary>True if the shape has generated a new face (or edge).</summary>
    public bool HasGenerated(in Shape s) => _generated.ContainsKey(s);

    /// <summary>
    /// The new faces built between tangent faces with different offset values (the 3D offset's per-face values).
    /// </summary>
    public NCollection_List<Shape> NewFaces() => _newFaces;

    //=================================================================================================

    /// <summary>
    /// The shape's descendants (the inverse of <see cref="Ancestors"/>), or null if it has none; the map is built on the
    /// first query, and again with <paramref name="update"/>.
    /// </summary>
    public NCollection_List<Shape>? Descendants(in Shape s, bool update = false)
    {
        if (_descendants.Count == 0 || update)
        {
            _descendants.Clear();
            int nbA = _ancestors.Extent();
            for (int i = 1; i <= nbA; ++i)
            {
                Shape ss = _ancestors.FindKey(i);
                NCollection_List<Shape> la = _ancestors[i];

                for (NCollection_List<Shape>.Iterator it = new(la); it.More(); it.Next())
                {
                    Shape sa = it.Value();

                    if (!_descendants.TryGetValue(sa, out NCollection_List<Shape>? pLD))
                    {
                        pLD = _descendants.SetValueRef(sa, new NCollection_List<Shape>());
                    }
                    if (!pLD.Contains(ss))
                    {
                        pLD.Append(ss);
                    }
                }
            }
        }

        return _descendants.TryGetValue(s, out NCollection_List<Shape>? ld) ? ld : null;
    }

    //=================================================================================================

    /// <summary>Clears the content of the algorithm.</summary>
    public void Clear()
    {
        _done = false;
        _shape = new Shape();
        _mapEdgeType.Clear();
        _ancestors.Clear();
        _faceOffsetMap.Clear();
        _replacement.Clear();
        _descendants.Clear();
        _newFaces.Clear();
        _generated.Clear();
    }

    //=======================================================================
    // function : NCollection_List<BRepOffset_Interval>&
    // purpose  :
    //=======================================================================

    /// <summary>The connectivity type of the edge: its intervals (throws if the edge wasn't analysed).</summary>
    public NCollection_List<Interval> Type(in Edge e)
    {
        return _mapEdgeType.GetValue(e);
    }

    //=================================================================================================

    /// <summary>Stores in <paramref name="le"/> all the edges of type <paramref name="t"/> on the vertex <paramref name="v"/>.</summary>
    public void Edges(in Vertex v, TypeOfConcavity t, NCollection_List<Shape> le)
    {
        le.Clear();
        NCollection_List<Shape> l = Ancestors(v);
        NCollection_List<Shape>.Iterator it = new(l);

        for (; it.More(); it.Next())
        {
            Edge e = ShapeCasts.Edge(it.Value());
            if (_mapEdgeType.TryGetValue(e, out NCollection_List<Interval>? pIntervals) && pIntervals.Extent() > 0)
            {
                OffsetTool.EdgeVertices(e, out Vertex v1, out Vertex v2);
                if (v1.IsSame(v))
                {
                    if (pIntervals.Last().Type() == t)
                    {
                        le.Append(e);
                    }
                }
                if (v2.IsSame(v))
                {
                    if (pIntervals.First().Type() == t)
                    {
                        le.Append(e);
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Stores in <paramref name="le"/> all the edges of type <paramref name="t"/> on the face <paramref name="f"/>.</summary>
    public void Edges(in Face f, TypeOfConcavity t, NCollection_List<Shape> le)
    {
        le.Clear();
        Explorer exp = new(f, ShapeType.Edge);

        for (; exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());

            NCollection_List<Interval> lint = Type(e);
            NCollection_List<Interval>.Iterator it = new(lint);
            for (; it.More(); it.Next())
            {
                if (it.Value().Type() == t)
                {
                    le.Append(e);
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Sets in <paramref name="edges"/> all the edges of the shape tangent to <paramref name="edge"/> at the vertex
    /// <paramref name="vertex"/>.
    /// </summary>
    public void TangentEdges(in Edge edge, in Vertex vertex, NCollection_List<Shape> edges)
    {
        Vector3D v, vRef;

        double u, uRef;
        EdgeCurveAdaptor c3d, c3dRef;

        uRef = Tool.Parameter(vertex, edge);
        c3dRef = new EdgeCurveAdaptor(edge);
        vRef = c3dRef.DN(uRef, 1);
        CorrectOrientationOfTangent(ref vRef, vertex, edge);
        if (vRef.SquareMagnitude() < Constants.Resolution())
        {
            return;
        }

        edges.Clear();

        NCollection_List<Shape> anc = Ancestors(vertex);
        NCollection_List<Shape>.Iterator it = new(anc);
        for (; it.More(); it.Next())
        {
            Edge curE = ShapeCasts.Edge(it.Value());
            if (curE.IsSame(edge))
            {
                continue;
            }
            u = Tool.Parameter(vertex, curE);
            c3d = new EdgeCurveAdaptor(curE);
            v = c3d.DN(u, 1);
            CorrectOrientationOfTangent(ref v, vertex, curE);
            if (v.SquareMagnitude() < Constants.Resolution())
            {
                continue;
            }
            if (v.IsOpposite(vRef, _angle))
            {
                edges.Append(curE);
            }
        }
    }

    /// <summary>True if the shape has ancestors.</summary>
    public bool HasAncestor(in Shape s) => _ancestors.Contains(s);

    /// <summary>The ancestors of the shape (throws if it has none).</summary>
    public NCollection_List<Shape> Ancestors(in Shape s) => _ancestors.FindFromKey(s);

    //=================================================================================================

    /// <summary>
    /// Explodes the shape into compounds of faces where all the connecting edges are of type <paramref name="t"/>.
    /// </summary>
    public void Explode(NCollection_List<Shape> list, TypeOfConcavity t)
    {
        list.Clear();
        BrepBuilder b = new();
        HashSet<Shape> map = new(ShapeHasher.Instance);

        Explorer fexp = new();
        for (fexp.Init(_shape, ShapeType.Face); fexp.More(); fexp.Next())
        {
            if (map.Add(fexp.Current()))
            {
                Face face = ShapeCasts.Face(fexp.Current());
                Compound co = new();
                b.MakeCompound(ref co);
                b.Add(co, face);
                // add to Co all faces from the cloud of faces
                // G1 created from <Face>
                AddFaces(face, co, map, t);
                list.Append(co);
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Explodes the shape into compounds of faces where all the connecting edges are of type <paramref name="t1"/> or
    /// <paramref name="t2"/>.
    /// </summary>
    public void Explode(NCollection_List<Shape> list, TypeOfConcavity t1, TypeOfConcavity t2)
    {
        list.Clear();
        BrepBuilder b = new();
        HashSet<Shape> map = new(ShapeHasher.Instance);

        Explorer fexp = new();
        for (fexp.Init(_shape, ShapeType.Face); fexp.More(); fexp.Next())
        {
            if (map.Add(fexp.Current()))
            {
                Face face = ShapeCasts.Face(fexp.Current());
                Compound co = new();
                b.MakeCompound(ref co);
                b.Add(co, face);
                // add to Co all faces from the cloud of faces
                // G1 created from  <Face>
                AddFaces(face, co, map, t1, t2);
                list.Append(co);
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Adds to <paramref name="co"/> the faces of the shell holding <paramref name="face"/> connected to it through
    /// edges of type <paramref name="t"/>.
    /// </summary>
    public void AddFaces(in Face face, in Compound co, HashSet<Shape> map,
                         TypeOfConcavity t)
    {
        BrepBuilder b = new();
        NCollection_List<Shape>? pLE = Descendants(face);
        if (pLE is null)
        {
            return;
        }
        for (NCollection_List<Shape>.Iterator it = new(pLE); it.More(); it.Next())
        {
            Edge e = ShapeCasts.Edge(it.Value());
            NCollection_List<Interval> li = Type(e);
            if (!li.IsEmpty() && li.First().Type() == t)
            {
                // so <NewFace> is attached to G1 by <Face>
                NCollection_List<Shape> l = Ancestors(e);
                if (l.Extent() == 2)
                {
                    Face f1 = ShapeCasts.Face(l.First());
                    if (f1.IsSame(face))
                    {
                        f1 = ShapeCasts.Face(l.Last());
                    }
                    if (map.Add(f1))
                    {
                        b.Add(co, f1);
                        AddFaces(f1, co, map, t);
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Adds to <paramref name="co"/> the faces of the shell holding <paramref name="face"/> connected to it through
    /// edges of type <paramref name="t1"/> or <paramref name="t2"/>.
    /// </summary>
    public void AddFaces(in Face face, in Compound co, HashSet<Shape> map,
                         TypeOfConcavity t1, TypeOfConcavity t2)
    {
        BrepBuilder b = new();
        NCollection_List<Shape>? pLE = Descendants(face);
        if (pLE is null)
        {
            return;
        }
        for (NCollection_List<Shape>.Iterator it = new(pLE); it.More(); it.Next())
        {
            Edge e = ShapeCasts.Edge(it.Value());
            NCollection_List<Interval> li = Type(e);
            if (!li.IsEmpty() && (li.First().Type() == t1 || li.First().Type() == t2))
            {
                // so <NewFace> is attached to G1 by <Face>
                NCollection_List<Shape> l = Ancestors(e);
                if (l.Extent() == 2)
                {
                    Face f1 = ShapeCasts.Face(l.First());
                    if (f1.IsSame(face))
                    {
                        f1 = ShapeCasts.Face(l.Last());
                    }
                    if (map.Add(f1))
                    {
                        b.Add(co, f1);
                        AddFaces(f1, co, map, t1, t2);
                    }
                }
            }
        }
    }

    /// <summary>Sets the offset value, for the treatment of tangent faces (the 3D offset only).</summary>
    internal void SetOffsetValue(double offset) => _offset = offset;

    /// <summary>
    /// Sets the faces' offset values, to treat tangent faces with different values (the 3D offset only). The map is
    /// copied, as OCCT assigns it.
    /// </summary>
    internal void SetFaceOffsetMap(Dictionary<Shape, double> map)
    {
        _faceOffsetMap = new Dictionary<Shape, double>(ShapeHasher.Instance);
        foreach (KeyValuePair<Shape, double> pair in map)
        {
            _faceOffsetMap[pair.Key] = pair.Value;
        }
    }
}
