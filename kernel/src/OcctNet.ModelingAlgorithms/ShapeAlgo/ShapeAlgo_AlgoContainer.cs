// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAlgo/ShapeAlgo_AlgoContainer.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAlgo/ShapeAlgo_AlgoContainer.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAlgo/ShapeAlgo_AlgoContainer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Occt.ShapeExtend_Status;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>
/// Shape Healing's algorithms as the data exchange translators call them (through <see cref="ShapeAlgo.AlgoContainer"/>):
/// connecting wires, approximating degree-1 B-splines, converting C0 geometry, outer wires, periodic surfaces, ...
/// </summary>
/// <remarks>
/// A working object (decision 21), to be derived from to substitute other algorithms. The public results built here
/// (shapes, wires) are frozen; geometry given is never changed (see
/// <see cref="ShapeUpgrade.C0BSplineToSequenceOfC1BSplineCurve(BSplineCurve3D?, ref NCollection_HSequence{BoundedCurve3D}?)"/>).
/// </remarks>
public class ShapeAlgo_AlgoContainer : Transient
{
    private ShapeAlgo_ToolContainer? myTC;

    /// <summary>Empty constructor (with a default ToolContainer).</summary>
    public ShapeAlgo_AlgoContainer()
    {
        myTC = new ShapeAlgo_ToolContainer();
    }

    /// <summary>Sets the ToolContainer.</summary>
    public void SetToolContainer(ShapeAlgo_ToolContainer? TC)
    {
        myTC = TC;
    }

    /// <summary>Returns the ToolContainer.</summary>
    public ShapeAlgo_ToolContainer? ToolContainer() => myTC;

    /// <summary>
    /// Finds the best way to connect <paramref name="nextsewd"/> to the wire already built in <paramref name="saw"/>
    /// and connects it (reversing either if needed); false if it can't be connected within <paramref name="maxtol"/>.
    /// <paramref name="distmin"/> receives the minimal distance between them.
    /// </summary>
    public virtual bool ConnectNextWire(ShapeAnalysis_Wire saw,
                                        ShapeExtend_WireData nextsewd,
                                        double maxtol,
                                        out double distmin,
                                        out bool revsewd,
                                        out bool revnextsewd)
    {
        distmin = 0;
        revsewd = revnextsewd = false;
        if (nextsewd.NbEdges() == 0)
        {
            return true;
        }

        ShapeExtend_WireData sewd = saw.WireData()!;
        // add edges into empty WireData
        if (sewd.NbEdges() == 0)
        {
            sewd.Add(nextsewd);
            return true;
        }

        saw.CheckShapeConnect(out double tailhead, out double tailtail, out double headtail, out double headhead,
                              nextsewd.InternalWire(), maxtol);
        distmin = tailhead;
        double precision = saw.Precision();

        if (tailhead > precision && tailtail > precision
            && (saw.LastCheckStatus(ShapeExtend_DONE4) || saw.LastCheckStatus(ShapeExtend_DONE3)))
        {
            sewd.Reverse();
            distmin = headhead;
            revsewd = true;
            if (saw.LastCheckStatus(ShapeExtend_DONE3))
            {
                nextsewd.Reverse();
                revnextsewd = true;
                distmin = headtail;
            }
        }
        else if (!saw.LastCheckStatus(ShapeExtend_FAIL) && !saw.LastCheckStatus(ShapeExtend_DONE5))
        {
            nextsewd.Reverse();
            revnextsewd = true;
            distmin = tailtail;
        }
        bool OK = !saw.LastCheckStatus(ShapeExtend_FAIL);
        if (OK)
        {
            sewd.Add(nextsewd);
        }
        return OK;
    }

    /// <summary>
    /// Approximates a degree-1 B-spline curve: it is cut where it has repeated poles and each part with more than two
    /// poles is approximated (GeomInt_WLApprox); other curves are returned as they are.
    /// </summary>
    public virtual void ApproxBSplineCurve(BSplineCurve3D bspline, NCollection_Sequence<Curve3D> seq)
    {
        seq.Clear();
        BSplineCurve3D modifCurve;
        var SCurve = new NCollection_Sequence<Curve3D>();

        // si la BSpline est de degre 1 , on approxime .
        // on passe par le programme des intersections ou tout le travail
        // est deja fait !!! ( il faut faire des paquets de 30 points
        // maximum , travailler dans un espace 0,1 pour tenir la precision)

        if (bspline.Degree() != 1)
        {
            seq.Append(bspline);
            return;
        }

        // on detecte d`eventuelles cassures par la multiplicite des poles.
        // Puis on approxime chaque "partie" de BSpline

        int NbPoles = bspline.PoleCount;
        NCollection_Array1<Point3D> Poles = bspline.InternalPoles();
        NCollection_Array1<double> Weigs = bspline.InternalWeightsArray();
        NCollection_Array1<double> Knots = bspline.InternalKnots();
        NCollection_Array1<int> Mults = bspline.InternalMultiplicities();
        int deg = bspline.Degree();

        int jpole = 1;
        int j, PoleIndex, I1;
        PoleIndex = 1;
        I1 = 1;
        for (int ipole = 1; ipole < NbPoles; ipole++)
        {
            if (Poles[ipole].IsEqual(Poles[ipole + 1], Kerf.Engine.Foundation.Precision.Confusion()))
            {
                if (jpole == 1)
                {
                    PoleIndex++;
                }
                else
                {
                    var newPoles = new NCollection_Array1<Point3D>(1, jpole);
                    var newWeigs = new NCollection_Array1<double>(1, jpole);
                    newWeigs.Init(1.0);
                    int NbNew = jpole - deg + 1;
                    var newKnots = new NCollection_Array1<double>(1, NbNew);
                    var newMults = new NCollection_Array1<int>(1, NbNew);
                    for (j = 1; j <= NbNew; j++)
                    {
                        newKnots[j] = Knots[I1 + j - 1];
                        newMults[j] = Mults[I1 + j - 1];
                    }
                    newMults[1] = newMults[NbNew] = deg + 1;
                    for (j = 1; j <= jpole; j++)
                    {
                        newWeigs[j] = Weigs[PoleIndex];
                        newPoles[j] = Poles[PoleIndex++];
                    }

                    var newC = new BSplineCurve3D(newPoles, newWeigs, newKnots, newMults, deg);
                    SCurve.Append(newC);
                    I1 = ipole + 1;
                    jpole = 1;
                }
            }
            else
            {
                jpole++;
            }
        }

        BSplineCurve3D mycurve;
        int nbcurves = SCurve.Length();
        if (nbcurves == 0)
        {
            nbcurves = 1;
            SCurve.Append(bspline);
        }

        for (int itab = 1; itab <= nbcurves; itab++)
        {
            mycurve = (BSplineCurve3D)SCurve.Value(itab);
            jpole = mycurve.PoleCount;
            if (jpole > 2)
            {
                NCollection_Array1<Point3D> newP = mycurve.InternalPoles();
                var R = new IntSurf_LineOn2S();
                double u1, v1, u2, v2;
                u1 = v1 = 0.0;
                u2 = v2 = 1.0;
                for (j = 1; j <= jpole; j++)
                {
                    var POn2S = new IntSurf_PntOn2S();
                    POn2S.SetValue(newP[j], u1, v1, u2, v2);
                    R.Add(POn2S);
                }
                var theapp3d = new GeomInt_WLApprox();
                double Tol = Kerf.Engine.Foundation.Precision.Approximation();
                theapp3d.SetParameters(Tol, Tol, 4, 8, 0, 30, true);
                var WL = new IntPatch_WLine(R, false);
                int indicemin = 1;
                int indicemax = jpole;
                theapp3d.Perform(WL, true, false, false, indicemin, indicemax);
                if (!theapp3d.IsDone())
                {
                    modifCurve = mycurve;
                }
                else if (theapp3d.NbMultiCurves() != 1)
                {
                    modifCurve = mycurve;
                }
                else
                {
                    MultiBSpCurve mbspc = theapp3d.Value(1);
                    int nbpoles = mbspc.NbPoles();
                    var tpoles = new NCollection_Array1<Point3D>(1, nbpoles);
                    mbspc.Curve(1, tpoles);
                    modifCurve = new BSplineCurve3D(tpoles, mbspc.Knots(), mbspc.Multiplicities(), mbspc.Degree());
                }
            }
            else
            {
                modifCurve = mycurve;
            }

            seq.Append(modifCurve);
        }
    }

    /// <summary>The 2D form of <see cref="ApproxBSplineCurve(BSplineCurve3D, NCollection_Sequence{Curve3D})"/> (parts with more than ten poles).</summary>
    public virtual void ApproxBSplineCurve(BSplineCurve2D bspline, NCollection_Sequence<Curve2D> seq)
    {
        seq.Clear();
        BSplineCurve2D modifCurve;
        var SCurve = new NCollection_Sequence<Curve2D>();

        // si la BSpline est de degre 1 , on approxime .
        // on passe par le programme des intersections ou tout le travail
        // est deja fait !!! ( il faut faire des paquets de 30 points
        // maximum , travailler dans un espace 0,1 pour tenir la precision
        // puis reconstruire une BSpline somme des toutes les Bspline).

        if (bspline.Degree() != 1)
        {
            seq.Append(bspline);
            return;
        }

        // on detecte d`eventuelles cassures par la multiplicite des poles.
        // Puis on approxime chaque "partie" de BSpline et on reconstruit
        // une BSpline = somme des BSplines traitees

        int NbPoles = bspline.PoleCount;
        NCollection_Array1<Point2D> Poles = bspline.InternalPoles();
        NCollection_Array1<double> Weigs = bspline.InternalWeightsArray();
        NCollection_Array1<double> Knots = bspline.InternalKnots();
        NCollection_Array1<int> Mults = bspline.InternalMultiplicities();
        int deg = bspline.Degree();

        int jpole = 1;
        int j, PoleIndex, I1;
        PoleIndex = 1;
        I1 = 1;
        for (int ipole = 1; ipole < NbPoles; ipole++)
        {
            if (Poles[ipole].IsEqual(Poles[ipole + 1], Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                if (jpole == 1)
                {
                    PoleIndex++;
                }
                else
                {
                    var newPoles = new NCollection_Array1<Point2D>(1, jpole);
                    var newWeigs = new NCollection_Array1<double>(1, jpole);
                    newWeigs.Init(1.0);
                    int NbNew = jpole - deg + 1;
                    var newKnots = new NCollection_Array1<double>(1, NbNew);
                    var newMults = new NCollection_Array1<int>(1, NbNew);
                    for (j = 1; j <= NbNew; j++)
                    {
                        newKnots[j] = Knots[I1 + j - 1];
                        newMults[j] = Mults[I1 + j - 1];
                    }
                    newMults[1] = newMults[NbNew] = deg + 1;
                    for (j = 1; j <= jpole; j++)
                    {
                        newWeigs[j] = Weigs[PoleIndex];
                        newPoles[j] = Poles[PoleIndex++];
                    }

                    var newC = new BSplineCurve2D(newPoles, newWeigs, newKnots, newMults, deg);
                    SCurve.Append(newC);
                    I1 = ipole + 1;
                    jpole = 1;
                }
            }
            else
            {
                jpole++;
            }
        }

        BSplineCurve2D mycurve;
        int nbcurves = SCurve.Length();
        if (nbcurves == 0)
        {
            nbcurves = 1;
            SCurve.Append(bspline);
        }

        for (int itab = 1; itab <= nbcurves; itab++)
        {
            mycurve = (BSplineCurve2D)SCurve.Value(itab);
            jpole = mycurve.PoleCount;
            if (jpole > 10)
            {
                var P = new NCollection_Array1<Point3D>(1, jpole);
                NCollection_Array1<Point2D> newP = mycurve.InternalPoles();
                var R = new IntSurf_LineOn2S();
                double u2, v2;
                u2 = v2 = 1.0;
                for (j = 1; j <= jpole; j++)
                {
                    var POn2S = new IntSurf_PntOn2S();
                    POn2S.SetValue(P[j], newP[j].X(), newP[j].Y(), u2, v2);
                    R.Add(POn2S);
                }
                var theapp3d = new GeomInt_WLApprox();
                double Tol = Kerf.Engine.Foundation.Precision.PApproximation();
                theapp3d.SetParameters(Tol, Tol, 4, 8, 0, 30, true);
                var WL = new IntPatch_WLine(R, false);
                int indicemin = 1;
                int indicemax = jpole;
                theapp3d.Perform(WL, false, true, false, indicemin, indicemax);
                if (!theapp3d.IsDone())
                {
                    modifCurve = mycurve;
                }
                else if (theapp3d.NbMultiCurves() != 1)
                {
                    modifCurve = mycurve;
                }
                else
                {
                    MultiBSpCurve mbspc = theapp3d.Value(1);
                    int nbpoles = mbspc.NbPoles();
                    var tpoles = new NCollection_Array1<Point2D>(1, nbpoles);
                    mbspc.Curve(1, tpoles);
                    modifCurve = new BSplineCurve2D(tpoles, mbspc.Knots(), mbspc.Multiplicities(), mbspc.Degree());
                }
            }
            else
            {
                modifCurve = mycurve;
            }

            seq.Append(modifCurve);
        }
    }

    /// <summary>Converts a C0 B-spline curve into a sequence of C1 B-spline curves (ShapeUpgrade's package function).</summary>
    public virtual bool C0BSplineToSequenceOfC1BSplineCurve(BSplineCurve3D BS,
                                                            ref NCollection_HSequence<BoundedCurve3D>? seqBS)
    {
        return ShapeUpgrade.C0BSplineToSequenceOfC1BSplineCurve(BS, ref seqBS);
    }

    /// <summary>Converts a C0 2D B-spline curve into a sequence of C1 B-spline curves (ShapeUpgrade's package function).</summary>
    public virtual bool C0BSplineToSequenceOfC1BSplineCurve(BSplineCurve2D BS,
                                                            ref NCollection_HSequence<BoundedCurve2D>? seqBS)
    {
        return ShapeUpgrade.C0BSplineToSequenceOfC1BSplineCurve(BS, ref seqBS);
    }

    /// <summary>Converts a shape on C0 geometry into a shape on C1 geometry (ShapeUpgrade_ShapeDivideContinuity), frozen.</summary>
    public virtual Shape C0ShapeToC1Shape(in Shape shape, double tol)
    {
        var sdc = new ShapeUpgrade_ShapeDivideContinuity(shape);
        sdc.SetTolerance(tol);
        sdc.SetBoundaryCriterion(C1);
        sdc.SetSurfaceCriterion(C1);
        sdc.Perform();
        return sdc.Result();
    }

    /// <summary>Converts a surface to a B-spline (ShapeConstruct).</summary>
    public virtual BSplineSurface? ConvertSurfaceToBSpline(Surface surf, double UF, double UL, double VF, double VL)
    {
        return ShapeConstruct.ConvertSurfaceToBSpline(surf,
                                                      UF,
                                                      UL,
                                                      VF,
                                                      VL,
                                                      Kerf.Engine.Foundation.Precision.Confusion(),
                                                      C1,
                                                      100,
                                                      BSplineSurface.MaxDegree());
    }

    /// <summary>
    /// Makes two wires with the same number of edges, the edges of the same rank having the same ratio between their
    /// parameter length and their wire's; the results are frozen. False if it failed.
    /// </summary>
    public virtual bool HomoWires(in Wire wireIn1,
                                  in Wire wireIn2,
                                  out Wire wireOut1,
                                  out Wire wireOut2,
                                  bool byParam)
    {
        wireOut1 = new Wire();
        wireOut2 = new Wire();
        Iterator Cook = new(), Perry = new();
        Edge edge1, edge2;
        Placement loc1, loc2;
        //  BRepBuilderAPI_MakeWire makeWire1, makeWire2;
        var makeWire1 = new ShapeExtend_WireData();
        var makeWire2 = new ShapeExtend_WireData();
        bool iterCook, iterPerry;
        int nEdges1, nEdges2;
        double length1, length2;
        double first1, first2;
        double last1, last2;
        double delta1, delta2;

        Curve3D? crv1;
        Curve3D? crv2;

        int nbCreatedEdges = 0;
        // gka
        // TopoDS_Vertex v11,v12,v21,v22
        var sfw = new ShapeFix_Wire();
        sfw.Load(wireIn1);
        sfw.FixReorder();
        Wire wireIn11 = sfw.Wire();
        sfw.Load(wireIn2);
        sfw.FixReorder();
        Wire wireIn22 = sfw.Wire();

        iterCook = iterPerry = true;
        length1 = length2 = 0.0;
        nEdges1 = nEdges2 = 0;
        for (Cook.Initialize(wireIn11); Cook.More(); Cook.Next())
        {
            nEdges1++;
            edge1 = ShapeCasts.Edge(Cook.Value());
            crv1 = Tool.Curve(edge1, out loc1, out first1, out last1);
            length1 += last1 - first1;
        }
        for (Perry.Initialize(wireIn22); Perry.More(); Perry.Next())
        {
            nEdges2++;
            edge2 = ShapeCasts.Edge(Perry.Value());
            crv2 = Tool.Curve(edge2, out loc2, out first2, out last2);
            length2 += last2 - first2;
        }
        double epsilon = Kerf.Engine.Foundation.Precision.PConfusion() * (length1 + length2);
        if (nEdges1 == 1 && nEdges2 == 1)
        {
            wireOut1 = wireIn11;
            wireOut2 = wireIn22;
            return true; // szv#4:S4163:12Mar99 `res=` not needed
        }

        if (length1 < epsilon)
        {
            Cook.Initialize(wireIn11);
            for (Perry.Initialize(wireIn22); Perry.More(); Perry.Next())
            {
                edge1 = ShapeCasts.Edge(Cook.Value());
                makeWire1.Add(edge1);
            }
            wireOut1 = makeWire1.Wire();
            wireOut2 = wireIn22;
            return true; // szv#4:S4163:12Mar99 `res=` not needed
        }
        if (length2 < epsilon)
        {
            Perry.Initialize(wireIn22);
            for (Cook.Initialize(wireIn11); Cook.More(); Cook.Next())
            {
                edge2 = ShapeCasts.Edge(Perry.Value());
                makeWire2.Add(edge2);
            }
            wireOut1 = wireIn11;
            wireOut2 = makeWire2.Wire();
            return true; // szv#4:S4163:12Mar99 `res=` not needed
        }

        double ratio = length2 / length1;

        Cook.Initialize(wireIn11);
        int cookIndex = 1;
        Perry.Initialize(wireIn22);
        edge1 = ShapeCasts.Edge(Cook.Value());
        edge2 = ShapeCasts.Edge(Perry.Value());
        // essai mjm du 22/05/97
        bool IsToReverse1 = false;
        bool IsToReverse2 = false;
        if (edge1.Orientation() == Reversed)
        {
            IsToReverse1 = true;
        }
        if (edge2.Orientation() == Reversed)
        {
            IsToReverse2 = true;
        }
        crv1 = Tool.Curve(edge1, out loc1, out first1, out last1);
        crv2 = Tool.Curve(edge2, out loc2, out first2, out last2);
        delta1 = last1 - first1;
        delta2 = last2 - first2;
        while (nbCreatedEdges < (nEdges1 + nEdges2 - 1))
        { /*just a security. */

            if ((delta1 * ratio - delta2) > epsilon)
            {
                var makeEdge1 = new BRepBuilderAPI_MakeEdge();
                if (!IsToReverse1)
                {
                    makeEdge1.Init(crv1!, first1, first1 + delta2 / ratio);
                    first1 += delta2 / ratio;
                }
                else
                { // gka BUC60685
                    makeEdge1.Init(crv1!, last1 - delta2 / ratio, last1);
                    last1 -= delta2 / ratio;
                }
                var makeEdge2 = new BRepBuilderAPI_MakeEdge(crv2!, first2, last2);
                edge1 = makeEdge1.InternalEdge();
                edge2 = makeEdge2.InternalEdge();
                // essai mjm du 22/05/97
                iterCook = false;
                // first1   += delta2/ratio;
                delta1 = last1 - first1;
                iterPerry = true;
                nbCreatedEdges++;
            }
            else if (Math.Abs(delta1 * ratio - delta2) <= epsilon)
            {
                var makeEdge1 = new BRepBuilderAPI_MakeEdge(crv1!, first1, last1);
                var makeEdge2 = new BRepBuilderAPI_MakeEdge(crv2!, first2, last2);
                edge1 = makeEdge1.InternalEdge();
                edge2 = makeEdge2.InternalEdge();
                iterCook = true;
                iterPerry = true;
                nbCreatedEdges += 2;
            }
            else /*((delta1*ratio - delta2) < -epsilon)*/
            {
                var makeEdge1 = new BRepBuilderAPI_MakeEdge(crv1!, first1, last1);
                edge1 = makeEdge1.InternalEdge();
                var makeEdge2 = new BRepBuilderAPI_MakeEdge();
                if (!IsToReverse2)
                {
                    makeEdge2.Init(crv2!, first2, first2 + delta1 * ratio);
                    first2 += delta1 * ratio;
                }
                else
                { // gka BUC60685
                    makeEdge2.Init(crv2!, last2 - delta1 * ratio, last2);
                    last2 -= delta1 * ratio;
                }
                edge1 = makeEdge1.InternalEdge();
                edge2 = makeEdge2.InternalEdge();
                iterCook = true;
                iterPerry = false;
                // first2   += delta1*ratio;
                delta2 = last2 - first2;
                nbCreatedEdges++;
            }
            edge1.Move(loc1);
            edge2.Move(loc2);
            if (IsToReverse1)
            {
                edge1.Reverse();
            }
            if (IsToReverse2)
            {
                edge2.Reverse();
            }
            makeWire1.Add(edge1);
            makeWire2.Add(edge2);

            if (iterCook && iterPerry)
            {
                // OCCT copies the iterator and looks one step ahead (TopoDS_Iterator has no copy here): Cook is on its
                // wire's last edge, or past it
                if (cookIndex >= nEdges1)
                {
                    wireOut1 = makeWire1.Wire();
                    wireOut2 = makeWire2.Wire();
                    return true; // szv#4:S4163:12Mar99 `res=` not needed
                }
            }
            if (iterCook)
            {
                Cook.Next();
                cookIndex++;
                edge1 = ShapeCasts.Edge(Cook.Value());
                IsToReverse1 = edge1.Orientation() == Reversed;
                crv1 = Tool.Curve(edge1, out loc1, out first1, out last1);
                delta1 = last1 - first1;
            }
            if (iterPerry)
            {
                Perry.Next();
                edge2 = ShapeCasts.Edge(Perry.Value());
                IsToReverse2 = edge2.Orientation() == Reversed;
                crv2 = Tool.Curve(edge2, out loc2, out first2, out last2);
                delta2 = last2 - first2;
            }
        }
        return false; // szv#4:S4163:12Mar99 `res=` not needed
    }

    /// <summary>Returns the outer wire of the face.</summary>
    public virtual Wire OuterWire(in Face face)
    {
        return ShapeAnalysis.OuterWire(face);
    }

    /// <summary>Converts a surface to its periodic form (ShapeCustom_Surface); null if it isn't converted.</summary>
    public virtual Surface? ConvertToPeriodic(Surface surf)
    {
        var scs = new ShapeCustom_Surface(surf);
        return scs.ConvertToPeriodic(false);
    }

    /// <summary>Computes the exact UV bounds of all the wires of the face.</summary>
    public virtual void GetFaceUVBounds(in Face F, out double Umin, out double Umax, out double Vmin, out double Vmax)
    {
        ShapeAnalysis.GetFaceUVBounds(F, out Umin, out Umax, out Vmin, out Vmax);
    }

    /// <summary>Converts a curve to a B-spline (ShapeConstruct).</summary>
    public virtual BSplineCurve3D? ConvertCurveToBSpline(Curve3D C3D,
                                                            double First,
                                                            double Last,
                                                            double Tol3d,
                                                            Continuity Continuity,
                                                            int MaxSegments,
                                                            int MaxDegree)
    {
        return ShapeConstruct.ConvertCurveToBSpline(C3D, First, Last, Tol3d, Continuity, MaxSegments, MaxDegree);
    }
}
