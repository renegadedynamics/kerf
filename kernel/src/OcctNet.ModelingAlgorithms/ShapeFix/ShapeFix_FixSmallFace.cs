// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_FixSmallFace.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_FixSmallFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

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
/// (<see cref="BRepTools_CopyAtEntry"/>, decision 583): set the context before it. <see cref="Shape"/> gives back the
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
    private new TopoDS_Shape myShape;
    private TopoDS_Shape myResult;
    private int myStatus;
    private readonly ShapeAnalysis_CheckSmallFace myAnalyzer;

    // The copy at the entry, from the first fix to the public result (null when the shape holds no frozen shape).
    private BRepTools_CopyAtEntry? myEntry;
    // True when the loaded shape must be checked (and copied if frozen) at the next fix.
    private bool myToEnter;

    /// <summary>Constructor (precision Confusion).</summary>
    public ShapeFix_FixSmallFace()
    {
        myShape    = new TopoDS_Shape();
        myResult   = new TopoDS_Shape();
        myAnalyzer = new ShapeAnalysis_CheckSmallFace();
        myStatus   = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        SetPrecision(Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>Loads the shape (creates a context if none is set).</summary>
    public void Init(in TopoDS_Shape S)
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
    public TopoDS_Shape FixSpotFace()
    {
        enterWork();
        // gp_Pnt spot;
        // double spotol;
        bool done = false;
        TopAbs_ShapeEnum st = myShape.ShapeType();
        if (st == TopAbs_COMPOUND || st == TopAbs_COMPSOLID || st == TopAbs_SOLID || st == TopAbs_SHELL
            || st == TopAbs_FACE)
        {
            for (var itf = new TopExp_Explorer(myShape, TopAbs_FACE); itf.More(); itf.Next())
            {
                // smh#8
                TopoDS_Shape tmpFace = Context()!.InternalApply(itf.Current());
                TopoDS_Face F = TopoDS.Face(tmpFace);
                if (F.IsNull())
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
                if (myShape.IsNull())
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
    internal bool ReplaceVerticesInCaseOfSpot(ref TopoDS_Face F, double tol)
    {
        var thePositions = new NCollection_Sequence<Xyz>();
        Xyz thePosition;
        var theBuilder = new BRep_Builder();
        double theMaxDev;
        double theMaxTol = 0.0;
        thePositions.Clear();
        Point3D thePoint;
        // smh#8
        TopoDS_Shape tmpFace = Context()!.InternalApply(F);
        F = TopoDS.Face(tmpFace);
        // gka Mar2000 Protection against faces without wires
        // but they occur due to bugs in the algorithm itself, it needs to be fixed
        bool isWir = false;
        for (var itw = new TopoDS_Iterator(F, false); itw.More(); itw.Next())
        {
            if (itw.Value().ShapeType() != TopAbs_WIRE)
            {
                continue;
            }
            TopoDS_Wire w1 = TopoDS.Wire(itw.Value());
            if (!w1.IsNull())
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
        for (var iter_vertex = new TopExp_Explorer(F, TopAbs_VERTEX); iter_vertex.More(); iter_vertex.Next())
        {
            TopoDS_Vertex V = TopoDS.Vertex(iter_vertex.Current());
            thePoint = BRep_Tool.Pnt(V);
            if (theMaxTol <= (BRep_Tool.Tolerance(V)))
            {
                theMaxTol = BRep_Tool.Tolerance(V);
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
        var theSharedVertex = new TopoDS_Vertex();
        theBuilder.MakeVertex(ref theSharedVertex);
        theBuilder.UpdateVertex(theSharedVertex, new Point3D(thePosition), theMaxDev + theMaxTol / 2);
        // Use external tolerance
        //   if (tol!=-1.0) theBuilder.UpdateVertex( theSharedVertex, tol);

        // Replacing all vertices in the face by new one
        TopoDS_Vertex theNewVertex;
        for (var iter_vert = new TopExp_Explorer(F, TopAbs_VERTEX); iter_vert.More(); iter_vert.Next())
        {
            TopoDS_Vertex V = TopoDS.Vertex(iter_vert.Current());
            if (V.Orientation() == TopAbs_FORWARD)
            // smh#8
            {
                TopoDS_Shape tmpVertexFwd = theSharedVertex.Oriented(TopAbs_FORWARD);
                theNewVertex = TopoDS.Vertex(tmpVertexFwd);
            }
            else
            // smh#8
            {
                TopoDS_Shape tmpVertexRev = theSharedVertex.Oriented(TopAbs_REVERSED);
                theNewVertex = TopoDS.Vertex(tmpVertexRev);
            }
            Context()!.Replace(V, theNewVertex);
        }
        return true;
    }

    /// <summary>Records the removal of the spot face and its edges.</summary>
    internal bool RemoveFacesInCaseOfSpot(in TopoDS_Face F)
    {
        for (var iter_vert = new TopExp_Explorer(F, TopAbs_EDGE); iter_vert.More(); iter_vert.Next())
        {
            TopoDS_Edge Ed = TopoDS.Edge(iter_vert.Current());
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
    public TopoDS_Shape FixStripFace(bool wasdone = false)
    {
        enterWork();
        if (myShape.IsNull())
        {
            return myShape;
        }
        TopAbs_ShapeEnum st = myShape.ShapeType();
        // BRep_Builder theBuilder;
        bool done = wasdone;
        if (st == TopAbs_COMPOUND || st == TopAbs_COMPSOLID || st == TopAbs_SOLID || st == TopAbs_SHELL
            || st == TopAbs_FACE)
        {
            for (var itf = new TopExp_Explorer(myShape, TopAbs_FACE); itf.More(); itf.Next())
            {
                TopoDS_Face F = TopoDS.Face(itf.Current());
                // smh#8
                TopoDS_Shape tmpFace = Context()!.InternalApply(F);
                F = TopoDS.Face(tmpFace);
                if (F.IsNull())
                {
                    continue;
                }
                // double dmax = 1;
                TopoDS_Edge E1 = new(), E2 = new();
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
            if (!myShape.IsNull())
            {
                for (var exp_s = new TopExp_Explorer(myShape, TopAbs_SHELL); exp_s.More(); exp_s.Next())
                {
                    TopoDS_Shell Sh = TopoDS.Shell(exp_s.Current());
                    var ex_sh = new TopExp_Explorer(Sh, TopAbs_FACE);
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
                    if (myShape.IsNull())
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
    internal bool ReplaceInCaseOfStrip(ref TopoDS_Face F, ref TopoDS_Edge E1, ref TopoDS_Edge E2, double tol)
    {
        if (E1.IsNull() || E2.IsNull())
        {
            return false;
        }
        TopoDS_Edge theSharedEdge;
        TopoDS_Face F1 = new(), F2 = new();
        // smh#8
        TopoDS_Shape tmpFace = Context()!.InternalApply(F);
        F = TopoDS.Face(tmpFace);
        for (var expf = new TopExp_Explorer(myShape, TopAbs_FACE); expf.More(); expf.Next())
        {
            // smh#8
            TopoDS_Shape tmpShape = Context()!.InternalApply(expf.Current());
            TopoDS_Face tempF = TopoDS.Face(tmpShape);
            if (tempF.IsNull() || tempF.IsSame(F))
            {
                continue;
            }
            for (var expe = new TopExp_Explorer(tempF, TopAbs_EDGE); expe.More(); expe.Next())
            {
                TopoDS_Edge tempE = TopoDS.Edge(expe.Current());
                if (tempE.IsSame(E1))
                {
                    F1 = tempF;
                }
                if (tempE.IsSame(E2))
                {
                    F2 = tempF;
                }
                if (!F1.IsNull())
                {
                    break; // && !F2.IsNull()) break;
                }
            }
        }

        // Compute shared edge for this face
        if (F1.IsNull() && F2.IsNull())
        {
            return true;
        }
        TopoDS_Edge E1tmp = E1;
        TopoDS_Edge E2tmp = E2;
        if (F1.IsNull())
        {
            E1tmp = E2;
            E2tmp = E1;
            F1 = F2;
        }
        theSharedEdge = ComputeSharedEdgeForStripFace(F, E1tmp, E2tmp, F1, tol);

        // Replace two long edges by new one
        if (theSharedEdge.IsNull())
        {
            return false;
        }
        if (E1.Orientation() == TopAbs_REVERSED)
        {
            Context()!.Replace(E1tmp, theSharedEdge.Oriented(TopAbs_REVERSED));
            if (F.Orientation() == F1.Orientation())
            {
                Context()!.Replace(E2tmp, theSharedEdge);
            }
            else
            {
                Context()!.Replace(E2tmp, theSharedEdge.Oriented(TopAbs_REVERSED));
            }
        }
        else
        {
            Context()!.Replace(E1tmp, theSharedEdge);
            if (F.Orientation() == F1.Orientation())
            {
                Context()!.Replace(E2tmp, theSharedEdge.Oriented(TopAbs_REVERSED));
            }
            else
            {
                Context()!.Replace(E2tmp, theSharedEdge);
            }
        }

        // Remove short edges
        for (var exp_e = new TopExp_Explorer(F, TopAbs_EDGE); exp_e.More(); exp_e.Next())
        {
            TopoDS_Edge shortedge = TopoDS.Edge(exp_e.Current());
            if (!shortedge.IsSame(E1tmp) && !shortedge.IsSame(E2tmp))
            {
                Context()!.Remove(shortedge);
            }
        }

        return true;
    }

    /// <summary>Records the removal of the strip face.</summary>
    internal bool RemoveFacesInCaseOfStrip(in TopoDS_Face F)
    {
        Context()!.Remove(F);
        SendWarning(F, new Msg("FixAdvFace.FixStripFace.MSG0"));
        return true;
    }

    /// <summary>
    /// Computes the edge replacing the two long edges of a strip face, recording the replacement of their vertices;
    /// a kept vertex's tolerance is raised in place.
    /// </summary>
    internal TopoDS_Edge ComputeSharedEdgeForStripFace(in TopoDS_Face F,
                                                       in TopoDS_Edge E1,
                                                       in TopoDS_Edge E2,
                                                       in TopoDS_Face F1,
                                                       double tol)
    {
        var theBuilder = new BRep_Builder();
        // Compute deviation between two vertices and create new vertices
        var theNewEdge = new TopoDS_Edge();
        TopExp.Vertices(E1, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
        TopExp.Vertices(E2, out TopoDS_Vertex V3, out TopoDS_Vertex V4);
        Point3D p1, p2;
        double dev;
        p1 = BRep_Tool.Pnt(V1);
        p2 = BRep_Tool.Pnt(V3);
        dev = p1.Distance(p2);
        var theFirstVer = new TopoDS_Vertex();
        var theSecondVer = new TopoDS_Vertex();
        theBuilder.MakeVertex(ref theFirstVer);
        theBuilder.MakeVertex(ref theSecondVer);
        Xyz thePosition;

        if ((dev <= BRep_Tool.Tolerance(V1)) || (dev <= BRep_Tool.Tolerance(V3)) || (dev <= tol))
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

                if (V1.Orientation() == TopAbs_FORWARD)
                {
                    Context()!.Replace(V1, theFirstVer.Oriented(TopAbs_FORWARD));
                }
                else
                {
                    Context()!.Replace(V1, theFirstVer.Oriented(TopAbs_REVERSED));
                }
                if (V3.Orientation() == TopAbs_FORWARD)
                {
                    Context()!.Replace(V3, theFirstVer.Oriented(TopAbs_FORWARD));
                }
                else
                {
                    Context()!.Replace(V3, theFirstVer.Oriented(TopAbs_REVERSED));
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
                    p1 = BRep_Tool.Pnt(V2);
                    p2 = BRep_Tool.Pnt(V4);
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
                if (V2.Orientation() == TopAbs_FORWARD)
                {
                    Context()!.Replace(V2, theSecondVer.Oriented(TopAbs_FORWARD));
                }
                else
                {
                    Context()!.Replace(V2, theSecondVer.Oriented(TopAbs_REVERSED));
                }
                if (V4.Orientation() == TopAbs_FORWARD)
                {
                    Context()!.Replace(V4, theSecondVer.Oriented(TopAbs_FORWARD));
                }
                else
                {
                    Context()!.Replace(V4, theSecondVer.Oriented(TopAbs_REVERSED));
                }
            }
        }
        else
        {
            p2 = BRep_Tool.Pnt(V4);
            dev = p1.Distance(p2);
            if ((dev <= BRep_Tool.Tolerance(V1)) || (dev <= BRep_Tool.Tolerance(V4)) || (dev <= tol))
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
                    if (V1.Orientation() == TopAbs_FORWARD)
                    {
                        Context()!.Replace(V1, theFirstVer.Oriented(TopAbs_FORWARD));
                    }
                    else
                    {
                        Context()!.Replace(V1, theFirstVer.Oriented(TopAbs_REVERSED));
                    }
                    if (V4.Orientation() == TopAbs_FORWARD)
                    {
                        Context()!.Replace(V4, theFirstVer.Oriented(TopAbs_FORWARD));
                    }
                    else
                    {
                        Context()!.Replace(V4, theFirstVer.Oriented(TopAbs_REVERSED));
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
                        p1 = BRep_Tool.Pnt(V2);
                        p2 = BRep_Tool.Pnt(V3);
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
                    if (V2.Orientation() == TopAbs_FORWARD)
                    {
                        Context()!.Replace(V2, theSecondVer.Oriented(TopAbs_FORWARD));
                    }
                    else
                    {
                        Context()!.Replace(V2, theSecondVer.Oriented(TopAbs_REVERSED));
                    }
                    if (V3.Orientation() == TopAbs_FORWARD)
                    {
                        Context()!.Replace(V3, theSecondVer.Oriented(TopAbs_FORWARD));
                    }
                    else
                    {
                        Context()!.Replace(V3, theSecondVer.Oriented(TopAbs_REVERSED));
                    }
                }
            }
            else
            {
                // The face is not strip face (OCCT_DEBUG message)
                return theNewEdge;
            }
        }
        if (theFirstVer.IsNull() || theSecondVer.IsNull())
        {
            return theNewEdge;
        }

        // Create new edge
        theBuilder.MakeEdge(ref theNewEdge);
        double f, l, fp1 = 0.0, lp1 = 0.0 /*, fp2, lp2*/;
        Geom_Curve? the3dcurve;
        the3dcurve = BRep_Tool.Curve(E1, out f, out l);
        Geom2d_Curve? the2dcurve1, thenew1 = null;
        if (!F1.IsNull())
        {
            the2dcurve1 = BRep_Tool.CurveOnSurface(E1, F1, out fp1, out lp1);
            if (the2dcurve1 is not null && fp1 != f && lp1 != l)
            {
                GeomLib.SameRange(Kerf.Engine.Foundation.Precision.Confusion(), the2dcurve1, fp1, lp1, f, l, ref thenew1);
            }
        }

        double maxdev;
        if ((BRep_Tool.Tolerance(theFirstVer)) <= (BRep_Tool.Tolerance(theSecondVer)))
        {
            maxdev = (BRep_Tool.Tolerance(theSecondVer));
        }
        else
        {
            maxdev = (BRep_Tool.Tolerance(theFirstVer));
        }
        // In place: when theFirstVer (theSecondVer) is V1 (V2), the strip's own vertex.
        theBuilder.UpdateVertex(theFirstVer, maxdev);
        theBuilder.UpdateVertex(theSecondVer, maxdev);
        // bool IsFree = true;
        theBuilder.SameParameter(theNewEdge, false);
        the3dcurve = BRep_Tool.Curve(E1, out f, out l);
        theBuilder.UpdateEdge(theNewEdge, the3dcurve, maxdev);
        theBuilder.Range(theNewEdge, f, l);
        if (!F1.IsNull() && thenew1 is not null)
        {
            theBuilder.UpdateEdge(theNewEdge, thenew1, F1, maxdev);
            // IsFree = false;
        }
        theBuilder.Add(theNewEdge, theFirstVer.Oriented(TopAbs_FORWARD));
        theBuilder.Add(theNewEdge, theSecondVer.Oriented(TopAbs_REVERSED));
        return theNewEdge;
    }

    /// <summary>
    /// Splits the faces of the shape at their splitting vertices (<paramref name="S"/> is ignored, as in OCCT) and
    /// returns the shape, unfrozen until <see cref="Shape"/> publishes it.
    /// </summary>
    public TopoDS_Shape FixSplitFace(in TopoDS_Shape S)
    {
        enterWork();
        if (myShape.IsNull())
        {
            return myShape;
        }
        TopAbs_ShapeEnum st = myShape.ShapeType();
        bool done = false;
        var theBuilder = new BRep_Builder();
        if (st == TopAbs_COMPOUND || st == TopAbs_COMPSOLID || st == TopAbs_SOLID || st == TopAbs_SHELL
            || st == TopAbs_FACE)
        {
            for (var itf = new TopExp_Explorer(myShape, TopAbs_FACE); itf.More(); itf.Next())
            {
                TopoDS_Face F = TopoDS.Face(itf.Current());
                var CompSplittedFaces = new TopoDS_Compound();
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
    internal bool SplitOneFace(ref TopoDS_Face F, in TopoDS_Compound theSplittedFaces)
    {
        var MapEdges = new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        var MapParam = new NCollection_DataMap<TopoDS_Shape, NCollection_List<double>>(TopTools_ShapeMapHasher.Instance);
        var theAllVert = new TopoDS_Compound();
        var theBuilder = new BRep_Builder();
        theBuilder.MakeCompound(ref theAllVert);
        // smh#8
        TopoDS_Shape tmpShape = Context()!.InternalApply(F);
        F = TopoDS.Face(tmpShape);
        // OCCT.NET: the in-place form, into the compound this method owns (each splitting vertex comes twice, as in OCCT)
        if (myAnalyzer.InternalCheckSplittingVertices(F, MapEdges, MapParam, theAllVert) != 0)
        {
            TopoDS_Wire tempwire = new();
            // Take information about splitting vertices
            if (theAllVert.IsNull())
            {
                return false;
            }
            // int i;
            TopoDS_Vertex V;
            var itc = new TopExp_Explorer(theAllVert, TopAbs_VERTEX);
            V = TopoDS.Vertex(itc.Current());
            if (V.IsNull())
            {
                return false;
            }
            Point3D proj = new();
            Point3D vp = BRep_Tool.Pnt(V);
            var theNewVertex = new TopoDS_Vertex();
            var E = new TopoDS_Edge();
            var theFirstEdge = new TopoDS_Edge();
            var theSecondEdge = new TopoDS_Edge();
            {
                // If one vertex presents do splitting by two faces
                var SAC = new ShapeAnalysis_Curve();
                for (var ite = new TopExp_Explorer(F, TopAbs_EDGE); ite.More(); ite.Next())
                {
                    E = TopoDS.Edge(ite.Current());
                    TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                    Geom_Curve? C3D = BRep_Tool.Curve(E, out double cf, out double cl);
                    if (C3D is null)
                    {
                        continue;
                    }
                    if (V.IsSame(V1) || V.IsSame(V2))
                    {
                        continue;
                    }
                    double vt = BRep_Tool.Tolerance(V);
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
                        Geom_Curve? the3dcurve = BRep_Tool.Curve(E, out double f, out double l);
                        theBuilder.UpdateEdge(theFirstEdge, the3dcurve, Kerf.Engine.Foundation.Precision.Confusion());
                        theBuilder.UpdateEdge(theSecondEdge, the3dcurve, Kerf.Engine.Foundation.Precision.Confusion());
                        if (V1.Orientation() == TopAbs_FORWARD)
                        {
                            theBuilder.Add(theFirstEdge, V1);
                            theBuilder.Add(theFirstEdge, theNewVertex.Oriented(TopAbs_REVERSED));
                            theBuilder.Add(theSecondEdge, theNewVertex.Oriented(TopAbs_FORWARD));
                            theBuilder.Add(theSecondEdge, V2);
                        }
                        else
                        {
                            theBuilder.Add(theFirstEdge, V2);
                            theBuilder.Add(theFirstEdge, theNewVertex.Oriented(TopAbs_REVERSED));
                            theBuilder.Add(theSecondEdge, theNewVertex.Oriented(TopAbs_FORWARD));
                            theBuilder.Add(theSecondEdge, V1);
                        }
                        theBuilder.Range(theFirstEdge, cf, param);
                        theBuilder.Range(theSecondEdge, param, cl);

                        // Replace old edge by two new edges
                        var twoedges = new TopoDS_Wire();
                        theBuilder.MakeWire(ref twoedges);
                        if (E.Orientation() == TopAbs_FORWARD)
                        {
                            theBuilder.Add(twoedges, theFirstEdge.Oriented(TopAbs_FORWARD));
                            theBuilder.Add(twoedges, theSecondEdge.Oriented(TopAbs_FORWARD));
                        }
                        else
                        {
                            theBuilder.Add(twoedges, theFirstEdge.Oriented(TopAbs_REVERSED));
                            theBuilder.Add(twoedges, theSecondEdge.Oriented(TopAbs_REVERSED));
                        }
                        Context()!.Replace(E, twoedges);
                        break;
                    }
                }
                if (theNewVertex.IsNull())
                {
                    return false;
                }

                // Create split edge
                var theSplitEdge = new TopoDS_Edge();
                var lin = new Line3D(vp, new Direction3D(new Vector3D(vp, proj)));
                double firstparam = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, vp);
                double lastparam = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, proj);
                var L = new Geom_Line(vp, new Direction3D(new Vector3D(vp, proj))); // C++ converts the gp_Vec implicitly
                Geom_Curve the3dc = L;
                theBuilder.MakeEdge(ref theSplitEdge, the3dc, Kerf.Engine.Foundation.Precision.Confusion());
                theBuilder.Add(theSplitEdge, V.Oriented(TopAbs_FORWARD));
                theBuilder.Add(theSplitEdge, theNewVertex.Oriented(TopAbs_REVERSED));
                theBuilder.Range(theSplitEdge, firstparam, lastparam);

                // Add pcurve in new edge
                var sfe = new ShapeFix_Edge();
                sfe.FixAddPCurve(theSplitEdge, F, false);

                // Reorder the wire
                TopoDS_Wire wireonface;
                // Inher loop is not support yet !!!
                var itw = new TopExp_Explorer(F, TopAbs_WIRE);
                wireonface = TopoDS.Wire(itw.Current());
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
                TopoDS_Wire w1 = new(), w2 = new();
                theBuilder.MakeWire(ref w1);
                theBuilder.MakeWire(ref w2);
                theBuilder.MakeWire(ref tempwire);
                // OCCT removes and adds edges of the wire while exploring it (its list iterator then reads the removed,
                // freed node to go on); the port explores the edges the wire had first, then the two it appends, which
                // OCCT visits too (occt-issues.md).
                var anEdges = new List<TopoDS_Edge>();
                for (var itnew = new TopExp_Explorer(wireonface, TopAbs_EDGE); itnew.More(); itnew.Next())
                {
                    anEdges.Add(TopoDS.Edge(itnew.Current()));
                }
                foreach (TopoDS_Edge ce in anEdges)
                {
                    if (ce.IsSame(E))
                    {
                        theBuilder.Remove(wireonface, ce);
                        theBuilder.Add(wireonface, theFirstEdge.Oriented(TopAbs_FORWARD));
                        theBuilder.Add(wireonface, theSecondEdge.Oriented(TopAbs_FORWARD));
                    }
                }
                sfw.Init(wireonface, F, Kerf.Engine.Foundation.Precision.Confusion());
                sfw.FixReorder();
                wireonface = sfw.InternalWire();
                for (var itere = new TopExp_Explorer(wireonface, TopAbs_EDGE); itere.More(); itere.Next())
                {
                    TopoDS_Edge ce = TopoDS.Edge(itere.Current());
                    TopoDS_Vertex thecontrol;
                    if (ce.Orientation() == TopAbs_FORWARD)
                    {
                        thecontrol = TopExp.LastVertex(ce);
                    }
                    else
                    {
                        thecontrol = TopExp.FirstVertex(ce);
                    }
                    theBuilder.Add(w1, ce);
                    if (thecontrol.IsSame(V))
                    {
                        theBuilder.Add(w1, theSplitEdge.Oriented(TopAbs_FORWARD));
                        TopoDS_Wire wtemp = w1;
                        w1 = w2;
                        w2 = wtemp;
                    }
                    if (thecontrol.IsSame(theNewVertex))
                    {
                        theBuilder.Add(w1, theSplitEdge.Oriented(TopAbs_REVERSED));
                        TopoDS_Wire wtemp = w1;
                        w1 = w2;
                        w2 = wtemp;
                    }
                }
                if (w1.IsNull() || w2.IsNull())
                {
                    return false;
                }

                // Create two new faces and replace old one
                var F1 = new TopoDS_Face();
                var F2 = new TopoDS_Face();
                theBuilder.MakeFace(ref F1, BRep_Tool.Surface(F), Kerf.Engine.Foundation.Precision.Confusion());
                theBuilder.MakeFace(ref F2, BRep_Tool.Surface(F), Kerf.Engine.Foundation.Precision.Confusion());
                theBuilder.Add(F1, w1);
                theBuilder.Add(F2, w2);
                var tf = new TopoDS_Compound();
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
    internal TopoDS_Face FixFace(in TopoDS_Face F)
    {
        // smh#8
        TopoDS_Shape emptyCopied = F.EmptyCopied();
        TopoDS_Face theFixedFace = TopoDS.Face(emptyCopied);
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
    public TopoDS_Shape FixShape()
    {
        enterWork();
        TopoDS_Shape FixSh = new();
        if (myShape.IsNull())
        {
            return FixSh;
        }
        for (var expf = new TopExp_Explorer(myShape, TopAbs_FACE); expf.More(); expf.Next())
        {
            TopoDS_Face F = TopoDS.Face(expf.Current());
            // smh#8
            TopoDS_Shape tmpFace = Context()!.InternalApply(F);
            F = TopoDS.Face(tmpFace);
            TopoDS_Face newF = FixFace(F);
            Context()!.Replace(F, newF);
        }
        FixSh = Context()!.InternalApply(myShape);
        return FixSh;
    }

    /// <summary>
    /// The resulting shape, frozen: the caller's own sub-shapes wherever the fixes left them unchanged (the loaded
    /// shape itself when nothing changed). The context then answers for the caller's shapes.
    /// </summary>
    public TopoDS_Shape Shape()
    {
        endWork();
        myShape.TShape()?.FreezeGraph();
        return myShape;
    }

    /// <summary>The resulting shape, unfrozen (ported code fixing a shape it owns).</summary>
    internal TopoDS_Shape InternalShape() => myShape;

    /// <summary>Fixes a pin face: does nothing (OCCT's stub).</summary>
    public bool FixPinFace(ref TopoDS_Face F) => true;

    // At the first fix after Init or a public result: copies the loaded shape when it holds a frozen shape.
    private void enterWork()
    {
        if (!myToEnter)
        {
            return;
        }
        myToEnter = false;
        myEntry = new BRepTools_CopyAtEntry(myShape, Context());
        if (!myEntry.IsCopied())
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
        BRepTools_CopyAtEntry? anEntry = myEntry;
        myEntry = null;
        if (anEntry is null)
        {
            return;
        }
        myShape = anEntry.Finish(myShape, [myResult]);
        myResult = anEntry.Final(myResult);
    }
}
