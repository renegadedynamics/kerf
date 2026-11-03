// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_FixSmallFace.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_FixSmallFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ShapeExtend_Status;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Fixes faces of small size: spot faces (all vertices within tolerance of a point) are removed and their vertices
/// merged; strip faces (two long edges within tolerance) are removed and their two long edges merged into one; faces
/// with a splitting vertex are split. The faces touched are then fixed with <see cref="ShapeFix_Face"/>.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). <see cref="Init"/> loads the shape; <see cref="Perform"/> (spot and strip faces),
/// <see cref="FixSpotFace"/>, <see cref="FixStripFace"/>, <see cref="FixSplitFace"/> and <see cref="FixShape"/> fix it,
/// recording the changes in the context. They also write into the shape in place (a merged strip's vertex tolerances,
/// the faces' fixes), so a shape holding frozen shapes (any shape a caller holds) is copied at the first fix
/// (<see cref="CopyAtEntry"/>, decision 583): set the context before it. <see cref="Shape"/> gives back the
/// caller's own sub-shapes wherever they are unchanged (the shape itself when nothing changed), re-keys the context to
/// the caller's shapes and freezes the result; a fix after it copies again.
/// </para>
/// <para>
/// OCCT's public per-face steps (ReplaceVerticesInCaseOfSpot, RemoveFacesInCaseOfSpot, ReplaceInCaseOfStrip,
/// RemoveFacesInCaseOfStrip, ComputeSharedEdgeForStripFace, SplitOneFace, FixFace) work on the faces of the working
/// copy and write into them: they are internal here.
/// </para>
/// </remarks>
public class ShapeFix_FixSmallFace : ShapeFix_Root
{
    // OCCT declares its own private myShape, hiding ShapeFix_Root's (which the messages are attached to).
    private new Shape myShape;
    private Shape myResult;
    private int myStatus;
    private readonly ShapeAnalysis_CheckSmallFace myAnalyzer;

    // The copy at the entry, from the first fix to the public result (null when the shape holds no frozen shape).
    private CopyAtEntry? myEntry;
    // True when the loaded shape must be checked (and copied if frozen) at the next fix.
    private bool myToEnter;

    /// <summary>Constructor (precision Confusion).</summary>
    public ShapeFix_FixSmallFace()
    {
        myShape    = new Shape();
        myResult   = new Shape();
        myAnalyzer = new ShapeAnalysis_CheckSmallFace();
        myStatus   = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        SetPrecision(Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>Loads the shape (creates a context if none is set).</summary>
    public void Init(in Shape S)
    {
        myShape = S;
        if (Context() is null)
        {
            SetContext(new ShapeBuild_ReShape());
        }
        myResult = myShape;
        Context()!.InternalApply(myShape);
        myEntry   = null;
        myToEnter = true;
    }

    /// <summary>Fixes the spot faces, then the strip faces.</summary>
    public void Perform()
    {
        FixSpotFace();
        FixStripFace();
    }

    /// <summary>
    /// Fixes the spot faces (removed, their vertices merged into one) and returns the shape, unfrozen until
    /// <see cref="Shape"/> publishes it.
    /// </summary>
    public Shape FixSpotFace()
    {
        enterWork();
        // gp_Pnt spot;
        // double spotol;
        bool done = false;
        ShapeType st = myShape.ShapeType();
        if (st == ShapeType.Compound || st == ShapeType.CompSolid || st == ShapeType.Solid || st == ShapeType.Shell
            || st == ShapeType.Face)
        {
            for (var itf = new Explorer(myShape, ShapeType.Face); itf.More(); itf.Next())
            {
                // smh#8
                Shape tmpFace = Context()!.InternalApply(itf.Current());
                Face F = ShapeCasts.Face(tmpFace);
                if (F.IsNull)
                {
                    continue;
                }
                if (myAnalyzer.CheckSpotFace(F, Precision()))
                {
                    ReplaceVerticesInCaseOfSpot(ref F, Precision());
                    RemoveFacesInCaseOfSpot(F);
                    myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                    done = true;
                }
            }
            myShape = Context()!.InternalApply(myShape);
            if (done)
            {
                if (myShape.IsNull)
                {
                    return myShape;
                }
                myShape = FixShape();
            }
            // myShape = Context()->Apply(myShape);
            myResult = myShape;
        }
        return myShape;
    }

    /// <summary>Computes the mean vertex of the face and records the replacement of its vertices by it.</summary>
    internal bool ReplaceVerticesInCaseOfSpot(ref Face F, double tol)
    {
        var thePositions = new NCollection_Sequence<Xyz>();
        Xyz thePosition;
        var theBuilder = new BrepBuilder();
        double theMaxDev;
        double theMaxTol = 0.0;
        thePositions.Clear();
        Point3D thePoint;
        // smh#8
        Shape tmpFace = Context()!.InternalApply(F);
        F = ShapeCasts.Face(tmpFace);
        // gka Mar2000 Protection against faces without wires
        // but they occur due to bugs in the algorithm itself, it needs to be fixed
        bool isWir = false;
        for (var itw = new Iterator(F, false); itw.More(); itw.Next())
        {
            if (itw.Value().ShapeType() != ShapeType.Wire)
            {
                continue;
            }
            Wire w1 = ShapeCasts.Wire(itw.Value());
            if (!w1.IsNull)
            {
                isWir = true;
                break;
            }
        }
        if (!isWir)
        {
            return true;
        }

        // Accumulating positions and maximal vertex tolerance
        for (var iter_vertex = new Explorer(F, ShapeType.Vertex); iter_vertex.More(); iter_vertex.Next())
        {
            Vertex V = ShapeCasts.Vertex(iter_vertex.Current());
            thePoint = Tool.Pnt(V);
            if (theMaxTol <= (Tool.Tolerance(V)))
            {
                theMaxTol = Tool.Tolerance(V);
            }
            thePositions.Append(thePoint.XYZ());
        }

        // Calculate common vertex position
        thePosition = new Xyz(0.0, 0.0, 0.0);
        int theNbPos = thePositions.Length();
        int i; // svv Jan11 2000 : porting on DEC
        for (i = 1; i <= theNbPos; i++)
        {
            thePosition += thePositions.Value(i);
        }
        if (theNbPos > 1)
        {
            thePosition /= theNbPos;
        }

        // Calculate maximal deviation
        theMaxDev = 0.0;
        for (i = 1; i <= theNbPos; i++)
        {
            double theDeviation = (thePosition - thePositions.Value(i)).Modulus();
            if (theDeviation > theMaxDev)
            {
                theMaxDev = theDeviation;
            }
        }
        theMaxDev *= 1.00001;

        // Create new vertex with mean point
        var theSharedVertex = new Vertex();
        theBuilder.MakeVertex(ref theSharedVertex);
        theBuilder.UpdateVertex(theSharedVertex, new Point3D(thePosition), theMaxDev + theMaxTol / 2);
        // Use external tolerance
        //   if (tol!=-1.0) theBuilder.UpdateVertex( theSharedVertex, tol);

        // Replacing all vertices in the face by new one
        Vertex theNewVertex;
        for (var iter_vert = new Explorer(F, ShapeType.Vertex); iter_vert.More(); iter_vert.Next())
        {
            Vertex V = ShapeCasts.Vertex(iter_vert.Current());
            if (V.Orientation() == Forward)
            // smh#8
            {
                Shape tmpVertexFwd = theSharedVertex.Oriented(Forward);
                theNewVertex = ShapeCasts.Vertex(tmpVertexFwd);
            }
            else
            // smh#8
            {
                Shape tmpVertexRev = theSharedVertex.Oriented(Reversed);
                theNewVertex = ShapeCasts.Vertex(tmpVertexRev);
            }
            Context()!.Replace(V, theNewVertex);
        }
        return true;
    }

    /// <summary>Records the removal of the spot face and its edges.</summary>
    internal bool RemoveFacesInCaseOfSpot(in Face F)
    {
        for (var iter_vert = new Explorer(F, ShapeType.Edge); iter_vert.More(); iter_vert.Next())
        {
            Edge Ed = ShapeCasts.Edge(iter_vert.Current());
            Context()!.Remove(Ed);
        }
        Context()!.Remove(F);
        SendWarning(F, new Msg("FixAdvFace.FixSpotFace.MSG0"));
        return true;
    }

    /// <summary>
    /// Fixes the strip faces (removed, their two long edges merged into one) and returns the shape, unfrozen until
    /// <see cref="Shape"/> publishes it.
    /// </summary>
    public Shape FixStripFace(bool wasdone = false)
    {
        enterWork();
        if (myShape.IsNull)
        {
            return myShape;
        }
        ShapeType st = myShape.ShapeType();
        // BRep_Builder theBuilder;
        bool done = wasdone;
        if (st == ShapeType.Compound || st == ShapeType.CompSolid || st == ShapeType.Solid || st == ShapeType.Shell
            || st == ShapeType.Face)
        {
            for (var itf = new Explorer(myShape, ShapeType.Face); itf.More(); itf.Next())
            {
                Face F = ShapeCasts.Face(itf.Current());
                // smh#8
                Shape tmpFace = Context()!.InternalApply(F);
                F = ShapeCasts.Face(tmpFace);
                if (F.IsNull)
                {
                    continue;
                }
                // double dmax = 1;
                Edge E1 = new(), E2 = new();
                if (myAnalyzer.CheckStripFace(F, ref E1, ref E2, Precision()))
                {
                    if (ReplaceInCaseOfStrip(ref F, ref E1, ref E2, Precision()))
                    {
                        RemoveFacesInCaseOfStrip(F);
                    }
                    myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                    done = true;
                }
            }
            myShape = Context()!.InternalApply(myShape);
            // Particular case of empty shell
            if (!myShape.IsNull)
            {
                for (var exp_s = new Explorer(myShape, ShapeType.Shell); exp_s.More(); exp_s.Next())
                {
                    Shell Sh = ShapeCasts.Shell(exp_s.Current());
                    var ex_sh = new Explorer(Sh, ShapeType.Face);
                    if (!ex_sh.More())
                    {
                        Context()!.Remove(Sh);
                        // std::cout << "Empty shell was removed" << std::endl;
                    }
                }
                myShape = Context()!.InternalApply(myShape);
                // Fixing of missing pcurves on new edges, if they were inserted
                if (done)
                {
                    if (myShape.IsNull)
                    {
                        return myShape;
                    }
                    myShape = FixShape();
                    // myShape = Context()->Apply(myShape);
                    myResult = myShape;
                }
            }
        }
        return myShape;
    }

    /// <summary>Records the replacement of the strip face's two long edges by one and the removal of its short ones.</summary>
    internal bool ReplaceInCaseOfStrip(ref Face F, ref Edge E1, ref Edge E2, double tol)
    {
        if (E1.IsNull || E2.IsNull)
        {
            return false;
        }
        Edge theSharedEdge;
        Face F1 = new(), F2 = new();
        // smh#8
        Shape tmpFace = Context()!.InternalApply(F);
        F = ShapeCasts.Face(tmpFace);
        for (var expf = new Explorer(myShape, ShapeType.Face); expf.More(); expf.Next())
        {
            // smh#8
            Shape tmpShape = Context()!.InternalApply(expf.Current());
            Face tempF = ShapeCasts.Face(tmpShape);
            if (tempF.IsNull || tempF.IsSame(F))
            {
                continue;
            }
            for (var expe = new Explorer(tempF, ShapeType.Edge); expe.More(); expe.Next())
            {
                Edge tempE = ShapeCasts.Edge(expe.Current());
                if (tempE.IsSame(E1))
                {
                    F1 = tempF;
                }
                if (tempE.IsSame(E2))
                {
                    F2 = tempF;
                }
                if (!F1.IsNull)
                {
                    break; // && !F2.IsNull()) break;
                }
            }
        }

        // Compute shared edge for this face
        if (F1.IsNull && F2.IsNull)
        {
            return true;
        }
        Edge E1tmp = E1;
        Edge E2tmp = E2;
        if (F1.IsNull)
        {
            E1tmp = E2;
            E2tmp = E1;
            F1 = F2;
        }
        theSharedEdge = ComputeSharedEdgeForStripFace(F, E1tmp, E2tmp, F1, tol);

        // Replace two long edges by new one
        if (theSharedEdge.IsNull)
        {
            return false;
        }
        if (E1.Orientation() == Reversed)
        {
            Context()!.Replace(E1tmp, theSharedEdge.Oriented(Reversed));
            if (F.Orientation() == F1.Orientation())
            {
                Context()!.Replace(E2tmp, theSharedEdge);
            }
            else
            {
                Context()!.Replace(E2tmp, theSharedEdge.Oriented(Reversed));
            }
        }
        else
        {
            Context()!.Replace(E1tmp, theSharedEdge);
            if (F.Orientation() == F1.Orientation())
            {
                Context()!.Replace(E2tmp, theSharedEdge.Oriented(Reversed));
            }
            else
            {
                Context()!.Replace(E2tmp, theSharedEdge);
            }
        }

        // Remove short edges
        for (var exp_e = new Explorer(F, ShapeType.Edge); exp_e.More(); exp_e.Next())
        {
            Edge shortedge = ShapeCasts.Edge(exp_e.Current());
            if (!shortedge.IsSame(E1tmp) && !shortedge.IsSame(E2tmp))
            {
                Context()!.Remove(shortedge);
            }
        }

        return true;
    }

    /// <summary>Records the removal of the strip face.</summary>
    internal bool RemoveFacesInCaseOfStrip(in Face F)
    {
        Context()!.Remove(F);
        SendWarning(F, new Msg("FixAdvFace.FixStripFace.MSG0"));
        return true;
    }

    /// <summary>
    /// Computes the edge replacing the two long edges of a strip face, recording the replacement of their vertices;
    /// a kept vertex's tolerance is raised in place.
    /// </summary>
    internal Edge ComputeSharedEdgeForStripFace(in Face F,
                                                       in Edge E1,
                                                       in Edge E2,
                                                       in Face F1,
                                                       double tol)
    {
        var theBuilder = new BrepBuilder();
        // Compute deviation between two vertices and create new vertices
        var theNewEdge = new Edge();
        Exploration.Vertices(E1, out Vertex V1, out Vertex V2);
        Exploration.Vertices(E2, out Vertex V3, out Vertex V4);
        Point3D p1, p2;
        double dev;
        p1 = Tool.Pnt(V1);
        p2 = Tool.Pnt(V3);
        dev = p1.Distance(p2);
        var theFirstVer = new Vertex();
        var theSecondVer = new Vertex();
        theBuilder.MakeVertex(ref theFirstVer);
        theBuilder.MakeVertex(ref theSecondVer);
        Xyz thePosition;

        if ((dev <= Tool.Tolerance(V1)) || (dev <= Tool.Tolerance(V3)) || (dev <= tol))
        {
            if (V1.IsSame(V3))
            {
                // #ifdef AIX  CKY : applies to all platforms
                theFirstVer = V1;
                //	theFirstVer = TopoDS::Vertex(V1);
            }
            else
            {
                dev = (dev / 2) * 1.0001;
                thePosition = (p1.XYZ() + p2.XYZ()) / 2;
                theBuilder.UpdateVertex(theFirstVer, new Point3D(thePosition), dev);

                if (V1.Orientation() == Forward)
                {
                    Context()!.Replace(V1, theFirstVer.Oriented(Forward));
                }
                else
                {
                    Context()!.Replace(V1, theFirstVer.Oriented(Reversed));
                }
                if (V3.Orientation() == Forward)
                {
                    Context()!.Replace(V3, theFirstVer.Oriented(Forward));
                }
                else
                {
                    Context()!.Replace(V3, theFirstVer.Oriented(Reversed));
                }
            }
            if (V1.IsSame(V2) || V3.IsSame(V4))
            {
                theSecondVer = theFirstVer;
            }
            else
            {
                if (!V2.IsSame(V4))
                {
                    // #ifdef AIX  CKY : applies to all platforms
                    p1 = Tool.Pnt(V2);
                    p2 = Tool.Pnt(V4);
                    dev = p1.Distance(p2);
                    thePosition = (p1.XYZ() + p2.XYZ()) / 2;
                    theBuilder.UpdateVertex(theSecondVer, new Point3D(thePosition), dev);
                }
                else
                {
                    theSecondVer = V2;
                }
            }
            if (!V2.IsSame(theSecondVer))
            {
                if (V2.Orientation() == Forward)
                {
                    Context()!.Replace(V2, theSecondVer.Oriented(Forward));
                }
                else
                {
                    Context()!.Replace(V2, theSecondVer.Oriented(Reversed));
                }
                if (V4.Orientation() == Forward)
                {
                    Context()!.Replace(V4, theSecondVer.Oriented(Forward));
                }
                else
                {
                    Context()!.Replace(V4, theSecondVer.Oriented(Reversed));
                }
            }
        }
        else
        {
            p2 = Tool.Pnt(V4);
            dev = p1.Distance(p2);
            if ((dev <= Tool.Tolerance(V1)) || (dev <= Tool.Tolerance(V4)) || (dev <= tol))
            {
                if (V1.IsSame(V4))
                {
                    // #ifdef AIX  CKY : applies to all platforms
                    theFirstVer = V1;
                    //	    theFirstVer = TopoDS::Vertex(V1);
                }
                else
                {
                    dev = (dev / 2) * 1.0001;
                    thePosition = (p1.XYZ() + p2.XYZ()) / 2;
                    theBuilder.UpdateVertex(theFirstVer, new Point3D(thePosition), dev);
                    if (V1.Orientation() == Forward)
                    {
                        Context()!.Replace(V1, theFirstVer.Oriented(Forward));
                    }
                    else
                    {
                        Context()!.Replace(V1, theFirstVer.Oriented(Reversed));
                    }
                    if (V4.Orientation() == Forward)
                    {
                        Context()!.Replace(V4, theFirstVer.Oriented(Forward));
                    }
                    else
                    {
                        Context()!.Replace(V4, theFirstVer.Oriented(Reversed));
                    }
                }
                if (V1.IsSame(V2) || V3.IsSame(V4))
                {
                    theSecondVer = theFirstVer;
                }
                else
                {
                    if (!V2.IsSame(V3))
                    {
                        p1 = Tool.Pnt(V2);
                        p2 = Tool.Pnt(V3);
                        dev = p1.Distance(p2);
                        thePosition = (p1.XYZ() + p2.XYZ()) / 2;
                        theBuilder.UpdateVertex(theSecondVer, new Point3D(thePosition), dev);
                    }
                    else
                    {
                        theSecondVer = V2;
                    }
                }
                if (!V2.IsSame(theSecondVer))
                {
                    if (V2.Orientation() == Forward)
                    {
                        Context()!.Replace(V2, theSecondVer.Oriented(Forward));
                    }
                    else
                    {
                        Context()!.Replace(V2, theSecondVer.Oriented(Reversed));
                    }
                    if (V3.Orientation() == Forward)
                    {
                        Context()!.Replace(V3, theSecondVer.Oriented(Forward));
                    }
                    else
                    {
                        Context()!.Replace(V3, theSecondVer.Oriented(Reversed));
                    }
                }
            }
            else
            {
                // The face is not strip face (OCCT_DEBUG message)
                return theNewEdge;
            }
        }
        if (theFirstVer.IsNull || theSecondVer.IsNull)
        {
            return theNewEdge;
        }

        // Create new edge
        theBuilder.MakeEdge(ref theNewEdge);
        double f, l, fp1 = 0.0, lp1 = 0.0 /*, fp2, lp2*/;
        Curve3D? the3dcurve;
        the3dcurve = Tool.Curve(E1, out f, out l);
        Curve2D? the2dcurve1, thenew1 = null;
        if (!F1.IsNull)
        {
            the2dcurve1 = Tool.CurveOnSurface(E1, F1, out fp1, out lp1);
            if (the2dcurve1 is not null && fp1 != f && lp1 != l)
            {
                global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(Kerf.Engine.Foundation.Precision.Confusion(), the2dcurve1, fp1, lp1, f, l, ref thenew1);
            }
        }

        double maxdev;
        if ((Tool.Tolerance(theFirstVer)) <= (Tool.Tolerance(theSecondVer)))
        {
            maxdev = (Tool.Tolerance(theSecondVer));
        }
        else
        {
            maxdev = (Tool.Tolerance(theFirstVer));
        }
        // In place: when theFirstVer (theSecondVer) is V1 (V2), the strip's own vertex.
        theBuilder.UpdateVertex(theFirstVer, maxdev);
        theBuilder.UpdateVertex(theSecondVer, maxdev);
        // bool IsFree = true;
        theBuilder.SameParameter(theNewEdge, false);
        the3dcurve = Tool.Curve(E1, out f, out l);
        theBuilder.UpdateEdge(theNewEdge, the3dcurve, maxdev);
        theBuilder.Range(theNewEdge, f, l);
        if (!F1.IsNull && thenew1 is not null)
        {
            theBuilder.UpdateEdge(theNewEdge, thenew1, F1, maxdev);
            // IsFree = false;
        }
        theBuilder.Add(theNewEdge, theFirstVer.Oriented(Forward));
        theBuilder.Add(theNewEdge, theSecondVer.Oriented(Reversed));
        return theNewEdge;
    }

    /// <summary>
    /// Splits the faces of the shape at their splitting vertices (<paramref name="S"/> is ignored, as in OCCT) and
    /// returns the shape, unfrozen until <see cref="Shape"/> publishes it.
    /// </summary>
    public Shape FixSplitFace(in Shape S)
    {
        enterWork();
        if (myShape.IsNull)
        {
            return myShape;
        }
        ShapeType st = myShape.ShapeType();
        bool done = false;
        var theBuilder = new BrepBuilder();
        if (st == ShapeType.Compound || st == ShapeType.CompSolid || st == ShapeType.Solid || st == ShapeType.Shell
            || st == ShapeType.Face)
        {
            for (var itf = new Explorer(myShape, ShapeType.Face); itf.More(); itf.Next())
            {
                Face F = ShapeCasts.Face(itf.Current());
                var CompSplittedFaces = new Compound();
                theBuilder.MakeCompound(ref CompSplittedFaces);
                if (SplitOneFace(ref F, CompSplittedFaces))
                {
                    done = true;
                    Context()!.Replace(F, CompSplittedFaces);
                }
            }
        }
        if (done)
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
        }
        myShape = Context()!.InternalApply(myShape);
        myResult = myShape;
        return myShape;
    }

    /// <summary>
    /// Splits the face at its first splitting vertex, recursively, adding the faces to
    /// <paramref name="theSplittedFaces"/> (an unfrozen compound the caller owns); false if it isn't split.
    /// </summary>
    internal bool SplitOneFace(ref Face F, in Compound theSplittedFaces)
    {
        var MapEdges = new NCollection_DataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        var MapParam = new NCollection_DataMap<Shape, NCollection_List<double>>(ShapeHasher.Instance);
        var theAllVert = new Compound();
        var theBuilder = new BrepBuilder();
        theBuilder.MakeCompound(ref theAllVert);
        // smh#8
        Shape tmpShape = Context()!.InternalApply(F);
        F = ShapeCasts.Face(tmpShape);
        // OCCT.NET: the in-place form, into the compound this method owns (each splitting vertex comes twice, as in OCCT)
        if (myAnalyzer.InternalCheckSplittingVertices(F, MapEdges, MapParam, theAllVert) != 0)
        {
            Wire tempwire = new();
            // Take information about splitting vertices
            if (theAllVert.IsNull)
            {
                return false;
            }
            // int i;
            Vertex V;
            var itc = new Explorer(theAllVert, ShapeType.Vertex);
            V = ShapeCasts.Vertex(itc.Current());
            if (V.IsNull)
            {
                return false;
            }
            Point3D proj = new();
            Point3D vp = Tool.Pnt(V);
            var theNewVertex = new Vertex();
            var E = new Edge();
            var theFirstEdge = new Edge();
            var theSecondEdge = new Edge();
            {
                // If one vertex presents do splitting by two faces
                var SAC = new ShapeAnalysis_Curve();
                for (var ite = new Explorer(F, ShapeType.Edge); ite.More(); ite.Next())
                {
                    E = ShapeCasts.Edge(ite.Current());
                    Exploration.Vertices(E, out Vertex V1, out Vertex V2);
                    Curve3D? C3D = Tool.Curve(E, out double cf, out double cl);
                    if (C3D is null)
                    {
                        continue;
                    }
                    if (V.IsSame(V1) || V.IsSame(V2))
                    {
                        continue;
                    }
                    double vt = Tool.Tolerance(V);
                    double dist = SAC.Project(C3D, vp, vt * 10.0, out proj, out double param, cf, cl);
                    if (dist == 0)
                    {
                        continue; // Projection on same curve but on other edge ?
                    }
                    if (dist <= vt)
                    {
                        theBuilder.MakeVertex(ref theNewVertex);
                        theBuilder.UpdateVertex(theNewVertex, proj, Kerf.Engine.Foundation.Precision.Confusion());
                        theBuilder.MakeEdge(ref theFirstEdge);
                        theBuilder.MakeEdge(ref theSecondEdge);
                        Curve3D? the3dcurve = Tool.Curve(E, out double f, out double l);
                        theBuilder.UpdateEdge(theFirstEdge, the3dcurve, Kerf.Engine.Foundation.Precision.Confusion());
                        theBuilder.UpdateEdge(theSecondEdge, the3dcurve, Kerf.Engine.Foundation.Precision.Confusion());
                        if (V1.Orientation() == Forward)
                        {
                            theBuilder.Add(theFirstEdge, V1);
                            theBuilder.Add(theFirstEdge, theNewVertex.Oriented(Reversed));
                            theBuilder.Add(theSecondEdge, theNewVertex.Oriented(Forward));
                            theBuilder.Add(theSecondEdge, V2);
                        }
                        else
                        {
                            theBuilder.Add(theFirstEdge, V2);
                            theBuilder.Add(theFirstEdge, theNewVertex.Oriented(Reversed));
                            theBuilder.Add(theSecondEdge, theNewVertex.Oriented(Forward));
                            theBuilder.Add(theSecondEdge, V1);
                        }
                        theBuilder.Range(theFirstEdge, cf, param);
                        theBuilder.Range(theSecondEdge, param, cl);

                        // Replace old edge by two new edges
                        var twoedges = new Wire();
                        theBuilder.MakeWire(ref twoedges);
                        if (E.Orientation() == Forward)
                        {
                            theBuilder.Add(twoedges, theFirstEdge.Oriented(Forward));
                            theBuilder.Add(twoedges, theSecondEdge.Oriented(Forward));
                        }
                        else
                        {
                            theBuilder.Add(twoedges, theFirstEdge.Oriented(Reversed));
                            theBuilder.Add(twoedges, theSecondEdge.Oriented(Reversed));
                        }
                        Context()!.Replace(E, twoedges);
                        break;
                    }
                }
                if (theNewVertex.IsNull)
                {
                    return false;
                }

                // Create split edge
                var theSplitEdge = new Edge();
                var lin = new Line3D(vp, new Direction3D(new Vector3D(vp, proj)));
                double firstparam = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, vp);
                double lastparam = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, proj);
                var L = new LineCurve3D(vp, new Direction3D(new Vector3D(vp, proj))); // C++ converts the gp_Vec implicitly
                Curve3D the3dc = L;
                theBuilder.MakeEdge(ref theSplitEdge, the3dc, Kerf.Engine.Foundation.Precision.Confusion());
                theBuilder.Add(theSplitEdge, V.Oriented(Forward));
                theBuilder.Add(theSplitEdge, theNewVertex.Oriented(Reversed));
                theBuilder.Range(theSplitEdge, firstparam, lastparam);

                // Add pcurve in new edge
                var sfe = new ShapeFix_Edge();
                sfe.FixAddPCurve(theSplitEdge, F, false);

                // Reorder the wire
                Wire wireonface;
                // Inher loop is not support yet !!!
                var itw = new Explorer(F, ShapeType.Wire);
                wireonface = ShapeCasts.Wire(itw.Current());
                itw.Next();
                if (itw.More())
                {
                    return false; // if face contains more than one wire
                }
                var sfw = new ShapeFix_Wire();
                sfw.Init(wireonface, F, Kerf.Engine.Foundation.Precision.Confusion());
                sfw.FixReorder();
                wireonface = sfw.InternalWire(); // OCCT.NET: written below

                // Create two new wires
                Wire w1 = new(), w2 = new();
                theBuilder.MakeWire(ref w1);
                theBuilder.MakeWire(ref w2);
                theBuilder.MakeWire(ref tempwire);
                // OCCT removes and adds edges of the wire while exploring it (its list iterator then reads the removed,
                // freed node to go on); the port explores the edges the wire had first, then the two it appends, which
                // OCCT visits too (occt-issues.md).
                var anEdges = new List<Edge>();
                for (var itnew = new Explorer(wireonface, ShapeType.Edge); itnew.More(); itnew.Next())
                {
                    anEdges.Add(ShapeCasts.Edge(itnew.Current()));
                }
                foreach (Edge ce in anEdges)
                {
                    if (ce.IsSame(E))
                    {
                        theBuilder.Remove(wireonface, ce);
                        theBuilder.Add(wireonface, theFirstEdge.Oriented(Forward));
                        theBuilder.Add(wireonface, theSecondEdge.Oriented(Forward));
                    }
                }
                sfw.Init(wireonface, F, Kerf.Engine.Foundation.Precision.Confusion());
                sfw.FixReorder();
                wireonface = sfw.InternalWire();
                for (var itere = new Explorer(wireonface, ShapeType.Edge); itere.More(); itere.Next())
                {
                    Edge ce = ShapeCasts.Edge(itere.Current());
                    Vertex thecontrol;
                    if (ce.Orientation() == Forward)
                    {
                        thecontrol = Exploration.LastVertex(ce);
                    }
                    else
                    {
                        thecontrol = Exploration.FirstVertex(ce);
                    }
                    theBuilder.Add(w1, ce);
                    if (thecontrol.IsSame(V))
                    {
                        theBuilder.Add(w1, theSplitEdge.Oriented(Forward));
                        Wire wtemp = w1;
                        w1 = w2;
                        w2 = wtemp;
                    }
                    if (thecontrol.IsSame(theNewVertex))
                    {
                        theBuilder.Add(w1, theSplitEdge.Oriented(Reversed));
                        Wire wtemp = w1;
                        w1 = w2;
                        w2 = wtemp;
                    }
                }
                if (w1.IsNull || w2.IsNull)
                {
                    return false;
                }

                // Create two new faces and replace old one
                var F1 = new Face();
                var F2 = new Face();
                theBuilder.MakeFace(ref F1, Tool.Surface(F), Kerf.Engine.Foundation.Precision.Confusion());
                theBuilder.MakeFace(ref F2, Tool.Surface(F), Kerf.Engine.Foundation.Precision.Confusion());
                theBuilder.Add(F1, w1);
                theBuilder.Add(F2, w2);
                var tf = new Compound();
                theBuilder.MakeCompound(ref tf);
                theBuilder.Add(tf, F1);
                theBuilder.Add(tf, F2);

                // Call recursive spliteoneface() for each face
                if (!SplitOneFace(ref F1, theSplittedFaces))
                {
                    theBuilder.Add(theSplittedFaces, F1);
                }
                if (!SplitOneFace(ref F2, theSplittedFaces))
                {
                    theBuilder.Add(theSplittedFaces, F2);
                }
            }
            return true;
        }
        return false;
    }

    /// <summary>Fixes a face with ShapeFix_Face (sharing the context); returns the fixed face, unfrozen.</summary>
    internal Face FixFace(in Face F)
    {
        // smh#8
        Shape emptyCopied = F.EmptyCopied();
        Face theFixedFace = ShapeCasts.Face(emptyCopied);
        var sff = new ShapeFix_Face();
        sff.SetContext(Context());
        sff.Init(F);
        sff.Perform();
        // sff->Init(theFixedFace);
        // sff->FixOrientation();
        theFixedFace = sff.InternalFace(); // OCCT.NET: the face of the shape being fixed
        return theFixedFace;
    }

    /// <summary>
    /// Fixes every face of the shape with ShapeFix_Face and returns the shape with the context applied, unfrozen
    /// until <see cref="Shape"/> publishes it.
    /// </summary>
    public Shape FixShape()
    {
        enterWork();
        Shape FixSh = new();
        if (myShape.IsNull)
        {
            return FixSh;
        }
        for (var expf = new Explorer(myShape, ShapeType.Face); expf.More(); expf.Next())
        {
            Face F = ShapeCasts.Face(expf.Current());
            // smh#8
            Shape tmpFace = Context()!.InternalApply(F);
            F = ShapeCasts.Face(tmpFace);
            Face newF = FixFace(F);
            Context()!.Replace(F, newF);
        }
        FixSh = Context()!.InternalApply(myShape);
        return FixSh;
    }

    /// <summary>
    /// The resulting shape, frozen: the caller's own sub-shapes wherever the fixes left them unchanged (the loaded
    /// shape itself when nothing changed). The context then answers for the caller's shapes.
    /// </summary>
    public Shape Shape()
    {
        endWork();
        myShape.TShape()?.FreezeGraph();
        return myShape;
    }

    /// <summary>The resulting shape, unfrozen (ported code fixing a shape it owns).</summary>
    internal Shape InternalShape() => myShape;

    /// <summary>Fixes a pin face: does nothing (OCCT's stub).</summary>
    public bool FixPinFace(ref Face F) => true;

    // At the first fix after Init or a public result: copies the loaded shape when it holds a frozen shape.
    private void enterWork()
    {
        if (!myToEnter)
        {
            return;
        }
        myToEnter = false;
        myEntry = new CopyAtEntry(myShape, Context());
        if (!myEntry.IsCopied)
        {
            myEntry = null;
            return;
        }
        myShape = myEntry.Shape();
        myResult = myEntry.Copied(myResult);
    }

    // Before a public result: the caller's own sub-shapes back where unchanged, the context re-keyed.
    private void endWork()
    {
        myToEnter = true;
        CopyAtEntry? anEntry = myEntry;
        myEntry = null;
        if (anEntry is null)
        {
            return;
        }
        myShape = anEntry.Finish(myShape, [myResult]);
        myResult = anEntry.Final(myResult);
    }
}
