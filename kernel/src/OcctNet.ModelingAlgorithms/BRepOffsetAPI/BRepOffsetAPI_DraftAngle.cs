// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_DraftAngle.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_DraftAngle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Taper-adding transformations on a shape: faces turned by a draft angle around their intersection with a neutral
/// plane (planes, cylinders and cones), with their tangent neighbours, and the shape rebuilt around them.
/// </summary>
/// <remarks>
/// <para>
/// Give the shape, <see cref="Add"/> each face with its direction, angle and neutral plane (check
/// <see cref="AddDone"/>; on a failure <see cref="ProblematicShape"/> is the face and <see cref="Remove"/> takes it
/// back), then <see cref="Build"/>. The result is a <see cref="BRepTools_Modifier"/>'s, corrected around seams
/// (<see cref="CorrectWires"/>) and for the vertex tolerances.
/// </para>
/// <para>
/// A maker (decision 414). The caller's shape is never written: the modifier rebuilds what changes and shares the rest
/// (decision 973); OCCT's <c>CorrectWires</c> then edits the modifier's result in place, freeing and changing wires,
/// faces, vertices and pcurves that may be the caller's, so when it has something to change it works on a copy of the
/// result (<see cref="BRepTools_CopyAtEntry"/>) and gives back the caller's sub-shapes it left alone (decision 9401).
/// </para>
/// </remarks>
public class BRepOffsetAPI_DraftAngle : BRepBuilderAPI_ModifyShape
{
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myVtxToReplace = new(TopTools_ShapeMapHasher.Instance);
    private readonly BRepTools_ReShape mySubs = new();

    // OCCT.NET: the copy of the modifier's result that CorrectWires works on (null when it had nothing to change).
    private BRepTools_CopyAtEntry? myResultCopy;

    //=================================================================================================

    /// <summary>An empty algorithm; <see cref="Init"/> gives the shape.</summary>
    public BRepOffsetAPI_DraftAngle()
    {
    }

    //=================================================================================================

    /// <summary>The algorithm on the shape <paramref name="S"/>.</summary>
    public BRepOffsetAPI_DraftAngle(in TopoDS_Shape S)
    {
        myInitialShape = S;
        myModification = new Draft_Modification(S);
    }

    private Draft_Modification draft() => (Draft_Modification)myModification!;

    //=================================================================================================

    /// <summary>Cancels the faces added, for the same shape.</summary>
    public void Clear()
    {
        if (myModification is not null)
        {
            draft().Clear();
        }
    }

    //=================================================================================================

    /// <summary>Changes the shape and cancels the faces added.</summary>
    public void Init(in TopoDS_Shape S)
    {
        myInitialShape = S;
        NotDone();
        if (myModification is not null)
        {
            draft().Init(S);
        }
        else
        {
            myModification = new Draft_Modification(S);
        }
    }

    //=================================================================================================

    /// <summary>
    /// Adds the face <paramref name="F"/> of the shape, to be turned by <paramref name="Angle"/> (radians; ignored
    /// up to 1e-4) about its intersection with <paramref name="Plane"/>, with the pull direction <paramref name="D"/>.
    /// Its tangent neighbours are added too. <paramref name="Flag"/> false keeps cylindrical neighbours' tangency.
    /// Check <see cref="AddDone"/> after.
    /// </summary>
    /// <exception cref="Standard_NullObject">No shape was given.</exception>
    /// <exception cref="Standard_ConstructionError">An earlier Add failed and its face wasn't removed.</exception>
    public void Add(in TopoDS_Face F, in gp_Dir D, double Angle, in gp_Pln Plane, bool Flag = true)
    {
        // POP-DPF : protection
        if (Math.Abs(Angle) <= 1.0e-04)
        {
            return;
        }
        Standard_NullObject_Raise_if(myInitialShape.IsNull(), "BRepOffsetAPI_DraftAngle::Add() - initial shape is not set");
        draft().Add(F, D, Angle, Plane, Flag);
    }

    //=================================================================================================

    /// <summary>True if the last <see cref="Add"/> succeeded.</summary>
    /// <exception cref="Standard_NullObject">No shape was given.</exception>
    public bool AddDone()
    {
        Standard_NullObject_Raise_if(myInitialShape.IsNull(), "BRepOffsetAPI_DraftAngle::AddDone() - initial shape is not set");
        return draft().ProblematicShape().IsNull();
    }

    //=================================================================================================

    /// <summary>Cancels the face <paramref name="F"/> and the faces added with it (after a failed Add).</summary>
    /// <exception cref="Standard_NullObject">No shape was given.</exception>
    /// <exception cref="Standard_NoSuchObject">The face wasn't added, or the draft was built.</exception>
    public void Remove(in TopoDS_Face F)
    {
        Standard_NullObject_Raise_if(myInitialShape.IsNull(), "BRepOffsetAPI_DraftAngle::Remove() - initial shape is not set");
        draft().Remove(F);
    }

    //=================================================================================================

    /// <summary>The face, edge or vertex on which the draft failed (null if none).</summary>
    /// <exception cref="Standard_NullObject">No shape was given.</exception>
    public TopoDS_Shape ProblematicShape()
    {
        Standard_NullObject_Raise_if(myInitialShape.IsNull(),
                                     "BRepOffsetAPI_DraftAngle::ProblematicShape() - initial shape is not set");
        return draft().ProblematicShape();
    }

    //=================================================================================================

    /// <summary>The error status.</summary>
    /// <exception cref="Standard_NullObject">No shape was given.</exception>
    public Draft_ErrorStatus Status()
    {
        Standard_NullObject_Raise_if(myInitialShape.IsNull(), "BRepOffsetAPI_DraftAngle::Status() - initial shape is not set");
        return draft().Error();
    }

    //=================================================================================================

    /// <summary>The faces of the shape added together with the face <paramref name="F"/>.</summary>
    /// <exception cref="Standard_NullObject">No shape was given.</exception>
    /// <exception cref="Standard_NoSuchObject">The face wasn't added.</exception>
    /// <exception cref="StdFail_NotDone">The draft isn't built.</exception>
    public IBentoList<TopoDS_Shape> ConnectedFaces(in TopoDS_Face F)
    {
        Standard_NullObject_Raise_if(myInitialShape.IsNull(),
                                     "BRepOffsetAPI_DraftAngle::ConnectedFaces() - initial shape is not set");
        return PublishedList(draft().ConnectedFaces(F));
    }

    //=================================================================================================

    /// <summary>The faces of the shape on which a draft was given.</summary>
    /// <exception cref="Standard_NullObject">No shape was given.</exception>
    /// <exception cref="StdFail_NotDone">An Add failed.</exception>
    public IBentoList<TopoDS_Shape> ModifiedFaces()
    {
        Standard_NullObject_Raise_if(myInitialShape.IsNull(),
                                     "BRepOffsetAPI_DraftAngle::ModifiedFaces() - initial shape is not set");
        return PublishedList(draft().ModifiedFaces());
    }

    //=================================================================================================

    internal override NCollection_List<TopoDS_Shape> InternalGenerated(in TopoDS_Shape S)
    {
        myGenerated.Clear();
        Standard_NullObject_Raise_if(myInitialShape.IsNull(), "BRepOffsetAPI_DraftAngle::Generated() - initial shape is not set");
        Draft_Modification DMod = draft();

        if (S.ShapeType() == TopAbs_FACE)
        {
            if (DMod.NewSurface(TopoDS.Face(S), out _, out _, out _, out _, out _))
            {
                if (myVtxToReplace.IsEmpty())
                {
                    myGenerated.Append(InternalModifiedShape(S));
                }
                else
                {
                    myGenerated.Append(mySubs.InternalValue(InternalModifiedShape(S)));
                }
            }
        }
        return myGenerated;
    }

    //=================================================================================================

    internal override NCollection_List<TopoDS_Shape> InternalModified(in TopoDS_Shape S)
    {
        myGenerated.Clear();
        Standard_NullObject_Raise_if(myInitialShape.IsNull(), "BRepOffsetAPI_DraftAngle::Modified() - initial shape is not set");
        Draft_Modification DMod = draft();

        if (S.ShapeType() == TopAbs_FACE)
        {
            if (!DMod.NewSurface(TopoDS.Face(S), out _, out _, out _, out _, out _))
            {
                // Ce n est pas une generation => peut etre une  modif
                if (myVtxToReplace.IsEmpty())
                {
                    myGenerated.Append(InternalModifiedShape(S));
                }
                else
                {
                    myGenerated.Append(mySubs.InternalValue(InternalModifiedShape(S)));
                }
                if (myGenerated.Extent() == 1 && myGenerated.First().IsSame(S))
                {
                    myGenerated.Clear();
                }
            }
        }
        return myGenerated;
    }

    //=================================================================================================

    internal override TopoDS_Shape InternalModifiedShape(in TopoDS_Shape S)
    {
        if (S.ShapeType() == TopAbs_VERTEX)
        {
            if (myVtxToReplace.IsBound(S))
            {
                return myVtxToReplace.Find(S);
            }
        }
        // OCCT.NET: through the copy CorrectWires worked on, if it made one.
        TopoDS_Shape aNS = myModifier.InternalModifiedShape(S);
        if (myResultCopy is not null)
        {
            aNS = myResultCopy.Final(myResultCopy.Copied(aNS));
        }
        if (myVtxToReplace.IsEmpty())
        {
            return aNS;
        }
        else
        {
            return mySubs.InternalValue(aNS);
        }
    }

    //=================================================================================================

    /// <summary>Builds the draft. Check IsDone, then <see cref="Status"/> and <see cref="ProblematicShape"/>.</summary>
    /// <exception cref="Standard_ConstructionError">An Add failed and its face wasn't removed.</exception>
    public override void Build(Message_ProgressRange theRange = default)
    {
        myResultCopy = null;
        draft().Perform();
        if (!draft().IsDone())
        {
            NotDone();
        }
        else
        {
            DoModif(myInitialShape);
            CorrectWires();
            CorrectVertexTol();
        }
    }

    //=================================================================================================

    /// <summary>
    /// Splits the edges of the faces' other wires where they cross a seam, and the seams there, so that the wires on
    /// periodic faces stay valid after the draft. <see cref="Build"/> calls it.
    /// </summary>
    public void CorrectWires()
    {
        double TolInter = 1.0e-7;
        int i, j, k;

        NCollection_Sequence<TopoDS_Shape> Eseq = new();
        NCollection_Sequence<TopoDS_Shape> Wseq = new();
        NCollection_Sequence<TopoDS_Shape> Fseq = new();
        TopoDS_Shape CurEdge, CurWire, CurFace;
        TopoDS_Iterator wit = new(), eit = new();

        TopExp_Explorer fexp = new(myShape, TopAbs_FACE);
        for (; fexp.More(); fexp.Next())
        {
            CurFace = fexp.Current();
            wit.Initialize(CurFace);
            for (; wit.More(); wit.Next())
            {
                CurWire = wit.Value();
                // OCCT iterates a map of the wire's edges (by address); the port in the wire's order.
                NCollection_IndexedMap<TopoDS_Shape> emap = new(TopTools_ShapeMapHasher.Instance);
                eit.Initialize(CurWire);
                for (; eit.More(); eit.Next())
                {
                    emap.Add(eit.Value());
                }
                for (int m = 1; m <= emap.Extent(); m++)
                {
                    CurEdge = emap[m];
                    if (BRepTools.IsReallyClosed(TopoDS.Edge(CurEdge), TopoDS.Face(CurFace)))
                    {
                        Eseq.Append(CurEdge);
                        Wseq.Append(CurWire);
                        Fseq.Append(CurFace);
                    }
                }
            }
        }

        NCollection_DataMap<TopoDS_Shape, NCollection_Sequence<double>> Emap = new(TopTools_ShapeMapHasher.Instance);

        NCollection_Sequence<TopoDS_Shape> NonSeam = new();
        NCollection_Sequence<TopoDS_Shape> NonSeamWires = new();
        NCollection_Sequence<NCollection_Sequence<double>> ParsNonSeam = new();
        NCollection_Sequence<NCollection_Sequence<TopoDS_Shape>> Seam = new();
        NCollection_Sequence<NCollection_Sequence<double>> ParsSeam = new();

        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> WFmap = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> WWmap = new(TopTools_ShapeMapHasher.Instance);
        for (i = 1; i <= Eseq.Length(); i++)
        {
            CurEdge = Eseq[i];
            CurWire = Wseq[i];
            CurFace = Fseq[i];
            //
            TopoDS_Face aFace = TopoDS.Face(CurFace);
            //
            // Prepare 2D adaptors for intersection.
            // The seam edge has two 2D curve, thus we have to create 2 adaptors
            BRepAdaptor_Curve2d aBAC2D1 = new(TopoDS.Edge(CurEdge), aFace);
            BRepAdaptor_Curve2d aBAC2D1R = new(TopoDS.Edge(CurEdge.Reversed()), aFace);
            // Get surface of the face to get 3D intersection point
            Geom_Surface aSurf = BRep_Tool.Surface(aFace, out TopLoc_Location aLoc)!;
            // Get the tolerance of the current edge to compare intersection points
            double aTolCurE = BRep_Tool.Tolerance(TopoDS.Edge(CurEdge));
            //
            wit.Initialize(CurFace);
            for (; wit.More(); wit.Next())
            {
                TopoDS_Shape aWire = wit.Value();
                if (!aWire.IsSame(CurWire))
                {
                    NCollection_Sequence<gp_Pnt> pts = new();
                    bool Wadd = false;
                    eit.Initialize(aWire);
                    for (; eit.More(); eit.Next())
                    {
                        TopoDS_Edge anEdge = TopoDS.Edge(eit.Value());
                        //
                        // Prepare 2D adaptor for intersection
                        BRepAdaptor_Curve2d aBAC2D2 = new(anEdge, aFace);
                        // Perform intersection
                        Geom2dInt_GInter aGInter = new();
                        aGInter.Perform(aBAC2D1, aBAC2D2, TolInter, TolInter);
                        if (!aGInter.IsDone() || aGInter.IsEmpty())
                        {
                            // If first intersection is empty try intersection with reversed edge
                            aGInter.Perform(aBAC2D1R, aBAC2D2, TolInter, TolInter);
                            if (!aGInter.IsDone() || aGInter.IsEmpty())
                            {
                                continue;
                            }
                        }
                        //
                        Wadd = true;
                        if (!WFmap.IsBound(aWire))
                        {
                            WFmap.Bind(aWire, CurFace);
                        }
                        int ind = 0;
                        for (j = 1; j <= NonSeam.Length(); j++)
                        {
                            if (anEdge.IsSame(NonSeam[j]))
                            {
                                ind = j;
                                break;
                            }
                        }
                        if (ind == 0)
                        {
                            NonSeam.Append(anEdge);
                            NonSeamWires.Append(aWire);
                            ind = NonSeam.Length();
                            ParsNonSeam.Append(new NCollection_Sequence<double>());
                            Seam.Append(new NCollection_Sequence<TopoDS_Shape>());
                            ParsSeam.Append(new NCollection_Sequence<double>());
                        }
                        if (!Emap.IsBound(CurEdge))
                        {
                            Emap.Bind(CurEdge, new NCollection_Sequence<double>());
                        }
                        //
                        // Get the tolerance of edge to compare intersection points
                        double aTolE = BRep_Tool.Tolerance(anEdge);
                        // Tolerance to compare the intersection points is the maximal
                        // tolerance of intersecting edges
                        double aTolCmp = Max(aTolCurE, aTolE);
                        //
                        int aNbIntPnt = aGInter.NbPoints();
                        for (k = 1; k <= aNbIntPnt; ++k)
                        {
                            IntRes2d_IntersectionPoint aP2DInt = aGInter.Point(k);
                            gp_Pnt2d aP2D = aP2DInt.Value();
                            gp_Pnt aP3D = aSurf.Value(aP2D.X(), aP2D.Y());
                            //
                            // Check if the intersection point is new
                            int ied = 0;
                            for (j = 1; j <= pts.Length(); j++)
                            {
                                if (aP3D.IsEqual(pts[j], aTolCmp))
                                {
                                    ied = j;
                                    break;
                                }
                            }
                            if (ied == 0)
                            {
                                pts.Append(aP3D);
                                Emap.ChangeFind(CurEdge).Append(aP2DInt.ParamOnFirst());
                                ParsNonSeam[ind].Append(aP2DInt.ParamOnSecond());
                                Seam[ind].Append(CurEdge);
                                ParsSeam[ind].Append(aP2DInt.ParamOnFirst());
                            }
                        }
                    } // for (; eit.More(); eit.Next())
                    if (Wadd)
                    {
                        if (!WWmap.IsBound(CurWire))
                        {
                            WWmap.Bind(CurWire, new NCollection_List<TopoDS_Shape>());
                        }
                        WWmap.ChangeFind(CurWire).Append(aWire);
                    }
                } // if (! aWire.IsSame( CurWire ))
            } // for (; wit.More(); wit.Next())
        } // for (i = 1; i <= Eseq.Length(); i++)

        // OCCT.NET: the rest writes into the result: vertices, wires and faces it frees and edits, pcurves it moves. Where
        // the result holds the caller's sub-shapes (frozen), the result is copied first and the analysis made again on
        // the copy (decision 9401).
        if (!Emap.IsEmpty() && myResultCopy is null && BRepTools.HoldsFrozen(myShape))
        {
            myResultCopy = new BRepTools_CopyAtEntry(myShape);
            myShape = myResultCopy.Shape();
            CorrectWires();
            myShape = myResultCopy.Finish(myShape);
            return;
        }

        // Sorting
        for (i = 1; i <= NonSeam.Length(); i++)
        {
            for (j = 1; j < ParsNonSeam[i].Length(); j++)
            {
                for (k = j + 1; k <= ParsNonSeam[i].Length(); k++)
                {
                    if (ParsNonSeam[i][k] < ParsNonSeam[i][j])
                    {
                        double temp = ParsNonSeam[i][j];
                        ParsNonSeam[i][j] = ParsNonSeam[i][k];
                        ParsNonSeam[i][k] = temp;
                        TopoDS_Shape tmp = Seam[i][j];
                        Seam[i][j] = Seam[i][k];
                        Seam[i][k] = tmp;
                        temp = ParsSeam[i][j];
                        ParsSeam[i][j] = ParsSeam[i][k];
                        ParsSeam[i][k] = temp;
                    }
                }
            }
        }
        // OCCT iterates Emap (and EPmap below) by the edges' addresses; the port in insertion order.
        foreach (TopoDS_Shape aKey in Emap.Keys.ToList())
        {
            NCollection_Sequence<double> Seq = new(Emap.Find(aKey));
            for (i = 1; i < Seq.Length(); i++)
            {
                for (j = i + 1; j <= Seq.Length(); j++)
                {
                    if (Seq[j] < Seq[i])
                    {
                        double temp = Seq[i];
                        Seq[i] = Seq[j];
                        Seq[j] = temp;
                    }
                }
            }
            Emap.ChangeFind(aKey) = Seq;
        }
        NCollection_DataMap<TopoDS_Shape, NCollection_Sequence<double>> EPmap = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<TopoDS_Shape, NCollection_Sequence<TopoDS_Shape>> EVmap =
            new(TopTools_ShapeMapHasher.Instance); // Seam
        NCollection_DataMap<TopoDS_Shape, NCollection_Sequence<TopoDS_Shape>> EWmap =
            new(TopTools_ShapeMapHasher.Instance); // Seam and wires intersecting it
        foreach (KeyValuePair<TopoDS_Shape, NCollection_Sequence<double>> anEntry in Emap)
        {
            EPmap.Bind(anEntry.Key, new NCollection_Sequence<double>());
            EVmap.Bind(anEntry.Key, new NCollection_Sequence<TopoDS_Shape>());
            EWmap.Bind(anEntry.Key, new NCollection_Sequence<TopoDS_Shape>());
        }

        // Reconstruction of non-seam edges
        BRepTools_Substitution aSub = new();
        BRep_Builder BB = new();
        for (i = 1; i <= NonSeam.Length(); i++)
        {
            TopoDS_Edge anEdge = TopoDS.Edge(NonSeam[i]);
            NCollection_List<TopoDS_Shape> NewEdges = new();
            TopoDS_Edge NewE;
            TopExp.Vertices(anEdge, out TopoDS_Vertex Vfirst, out TopoDS_Vertex Vlast);
            double par;
            BRep_Tool.Range(anEdge, out double FirstPar, out double LastPar);
            int firstind = 1;
            par = ParsNonSeam[i][1];
            TopoDS_Edge SeamEdge = TopoDS.Edge(Seam[i][1]);
            // Find the face
            for (j = 1; j <= Eseq.Length(); j++)
            {
                if (SeamEdge.IsSame(Eseq[j]))
                {
                    break;
                }
            }
            TopoDS_Face theFace = TopoDS.Face(Fseq[j]);
            Geom_Surface? theSurf = BRep_Tool.Surface(theFace, out TopLoc_Location L);
            if (Math.Abs(par - FirstPar) <= Precision.Confusion())
            {
                BB.UpdateVertex(Vfirst, ParsSeam[i][1], SeamEdge, BRep_Tool.Tolerance(Vfirst));
                EPmap.ChangeFind(SeamEdge).Append(ParsSeam[i][1]);
                EVmap.ChangeFind(SeamEdge).Append(Vfirst);
                EWmap.ChangeFind(SeamEdge).Append(NonSeamWires[i]);
                firstind = 2;
            }
            double prevpar = FirstPar;
            TopoDS_Vertex PrevV = Vfirst;
            for (j = firstind; j <= ParsNonSeam[i].Length(); j++)
            {
                TopoDS_Shape aLocalShape = anEdge.EmptyCopied();
                NewE = TopoDS.Edge(aLocalShape);
                // NewE = TopoDS::Edge( anEdge.EmptyCopied() );
                TopoDS_Vertex NewV;
                par = ParsNonSeam[i][j];
                BB.Range(NewE, prevpar, par);
                SeamEdge = TopoDS.Edge(Seam[i][j]);
                if (j == ParsNonSeam[i].Length() && Math.Abs(par - LastPar) <= Precision.Confusion())
                {
                    NewV = Vlast;
                    if (firstind == 2 && j == 2)
                    {
                        BB.UpdateVertex(Vlast, ParsSeam[i][j], SeamEdge, BRep_Tool.Tolerance(Vlast));
                        EPmap.ChangeFind(SeamEdge).Append(ParsSeam[i][j]);
                        EVmap.ChangeFind(SeamEdge).Append(Vlast);
                        EWmap.ChangeFind(SeamEdge).Append(NonSeamWires[i]);
                        break;
                    }
                }
                else
                {
                    BRepAdaptor_Curve bcur = new(NewE);
                    gp_Pnt Point = bcur.Value(par);
                    NewV = new BRepLib_MakeVertex(Point).InternalVertex();
                    BB.UpdateVertex(NewV, par, NewE, 10.0 * Precision.Confusion());
                }
                BB.UpdateVertex(NewV, ParsSeam[i][j], SeamEdge, 10.0 * Precision.Confusion());
                NewE.Orientation(TopAbs_FORWARD);
                BB.Add(NewE, PrevV.Oriented(TopAbs_FORWARD));
                BB.Add(NewE, NewV.Oriented(TopAbs_REVERSED));

                NewEdges.Append(NewE);
                EPmap.ChangeFind(SeamEdge).Append(ParsSeam[i][j]);
                EVmap.ChangeFind(SeamEdge).Append(NewV);
                EWmap.ChangeFind(SeamEdge).Append(NonSeamWires[i]);

                prevpar = par;
                PrevV = NewV;
            }
            {
                // The last edge
                TopoDS_Shape aLocalShape = anEdge.EmptyCopied();
                NewE = TopoDS.Edge(aLocalShape);
                // NewE = TopoDS::Edge( anEdge.EmptyCopied() );
                par = LastPar;
                if (Math.Abs(prevpar - par) > Precision.Confusion())
                {
                    BB.Range(NewE, prevpar, par);
                    NewE.Orientation(TopAbs_FORWARD);
                    BB.Add(NewE, PrevV.Oriented(TopAbs_FORWARD));
                    BB.Add(NewE, Vlast.Oriented(TopAbs_REVERSED));
                    NewEdges.Append(NewE);
                }
            }

            // Substitute anEdge by NewEdges
            aSub.Substitute(anEdge, NewEdges);
        }

        // Sorting of EPmap and EVmap and removing repeating points from them
        foreach (TopoDS_Shape aKey in EPmap.Keys.ToList())
        {
            NCollection_Sequence<double> Seq = new(EPmap.Find(aKey));
            NCollection_Sequence<TopoDS_Shape> SeqShape = new(EVmap.Find(aKey));
            NCollection_Sequence<TopoDS_Shape> SeqShape2 = new(EWmap.Find(aKey));
            for (i = 1; i < Seq.Length(); i++)
            {
                for (j = i + 1; j <= Seq.Length(); j++)
                {
                    if (Seq[j] < Seq[i])
                    {
                        double temp = Seq[i];
                        Seq[i] = Seq[j];
                        Seq[j] = temp;
                        TopoDS_Shape tmp = SeqShape[i];
                        SeqShape[i] = SeqShape[j];
                        SeqShape[j] = tmp;
                        tmp = SeqShape2[i];
                        SeqShape2[i] = SeqShape2[j];
                        SeqShape2[j] = tmp;
                    }
                }
            }

            EPmap.ChangeFind(aKey) = Seq;
            EVmap.ChangeFind(aKey) = SeqShape;
            EWmap.ChangeFind(aKey) = SeqShape2;
        }
        foreach (TopoDS_Shape aKey in EPmap.Keys.ToList())
        {
            NCollection_Sequence<double> Seq = new(EPmap.Find(aKey));
            NCollection_Sequence<TopoDS_Shape> SeqShape = new(EVmap.Find(aKey));
            NCollection_Sequence<TopoDS_Shape> SeqShape2 = new(EWmap.Find(aKey));
            bool remove = true;
            while (remove)
            {
                remove = false;
                for (i = 1; i < Seq.Length(); i++)
                {
                    if (Math.Abs(Seq[i] - Seq[i + 1]) <= Precision.Confusion())
                    {
                        Seq.Remove(i + 1);
                        SeqShape.Remove(i + 1);
                        SeqShape2.Remove(i + 1);
                        remove = true;
                    }
                }
            }
            EPmap.ChangeFind(aKey) = Seq;
            EVmap.ChangeFind(aKey) = SeqShape;
            EWmap.ChangeFind(aKey) = SeqShape2;
        }

        // Reconstruction of seam edges
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> VEmap = new(TopTools_ShapeMapHasher.Instance);
        foreach (KeyValuePair<TopoDS_Shape, NCollection_Sequence<double>> anEntry in Emap)
        {
            TopoDS_Edge anEdge = TopoDS.Edge(anEntry.Key);
            bool onepoint = false;
            NCollection_List<TopoDS_Shape> NewEdges = new();
            NCollection_Sequence<double> Seq = new(anEntry.Value);
            NCollection_Sequence<double> Seq2 = new(EPmap.Find(anEdge));
            NCollection_Sequence<TopoDS_Shape> SeqVer = new(EVmap.Find(anEdge));
            NCollection_Sequence<TopoDS_Shape> SeqWire = new(EWmap.Find(anEdge));
            TopExp.Vertices(anEdge, out TopoDS_Vertex Vfirst, out TopoDS_Vertex Vlast);
            double fpar, lpar;
            BRep_Tool.Range(anEdge, out double FirstPar, out double LastPar);
            fpar = FirstPar;
            lpar = Seq[1];
            TopoDS_Edge NewE;
            int firstind = 1;
            if (Math.Abs(fpar - lpar) <= Precision.Confusion())
            {
                firstind = 2;
                fpar = Seq[1];
                lpar = Seq[2];
            }
            else
            {
                if (Seq.Length() % 2 != 0)
                {
                    VEmap.Bind(Vfirst, anEdge);
                    firstind = 2;
                    fpar = Seq[1];
                    if (Seq.Length() > 2)
                    {
                        lpar = Seq[2];
                    }
                    else
                    {
                        onepoint = true;
                    }
                }
            }
            if (!onepoint)
            {
                TopoDS_Shape aLocalShape = anEdge.EmptyCopied();
                NewE = TopoDS.Edge(aLocalShape);
                // NewE = TopoDS::Edge( anEdge.EmptyCopied() );
                BB.Range(NewE, fpar, lpar);
                NewE.Orientation(TopAbs_FORWARD);
                if (firstind == 1)
                {
                    BB.Add(NewE, Vfirst.Oriented(TopAbs_FORWARD));
                    aLocalShape = SeqVer[1].Oriented(TopAbs_REVERSED);
                    BB.Add(NewE, TopoDS.Vertex(aLocalShape));
                    // BB.Add( NewE, TopoDS::Vertex( SeqVer(1).Oriented(TopAbs_REVERSED) ) );
                }
                else
                {
                    aLocalShape = SeqVer[1].Oriented(TopAbs_FORWARD);
                    BB.Add(NewE, TopoDS.Vertex(aLocalShape));
                    aLocalShape = SeqVer[2].Oriented(TopAbs_REVERSED);
                    BB.Add(NewE, TopoDS.Vertex(aLocalShape));
                    // BB.Add( NewE, TopoDS::Vertex( SeqVer(1).Oriented(TopAbs_FORWARD) ) );
                    // BB.Add( NewE, TopoDS::Vertex( SeqVer(2).Oriented(TopAbs_REVERSED) ) );
                }
                NewEdges.Append(NewE);

                firstind++;
                for (i = firstind; i < Seq.Length(); i += 2)
                {
                    aLocalShape = anEdge.EmptyCopied();
                    NewE = TopoDS.Edge(aLocalShape);
                    // NewE = TopoDS::Edge( anEdge.EmptyCopied() );
                    fpar = Seq[i];
                    lpar = Seq[i + 1];
                    BB.Range(NewE, fpar, lpar);
                    // Find vertices
                    for (j = 1; j <= Seq2.Length(); j++)
                    {
                        if (Math.Abs(fpar - Seq2[j]) <= Precision.Confusion())
                        {
                            break;
                        }
                    }
                    NewE.Orientation(TopAbs_FORWARD);
                    TopoDS_Shape aLocalShapeCur = SeqVer[j].Oriented(TopAbs_FORWARD);
                    BB.Add(NewE, TopoDS.Vertex(aLocalShapeCur));
                    aLocalShapeCur = SeqVer[j + 1].Oriented(TopAbs_REVERSED);
                    BB.Add(NewE, TopoDS.Vertex(aLocalShapeCur));
                    // BB.Add( NewE, TopoDS::Vertex( SeqVer(j).Oriented(TopAbs_FORWARD) ) );
                    // BB.Add( NewE, TopoDS::Vertex( SeqVer(j+1).Oriented(TopAbs_REVERSED) ) );
                    NewEdges.Append(NewE);
                }
            }

            i = Seq.Length();
            fpar = Seq[i];
            lpar = LastPar;
            if (Math.Abs(fpar - lpar) <= Precision.Confusion())
            {
                continue;
            }
            {
                TopoDS_Shape aLocalShape = anEdge.EmptyCopied();
                NewE = TopoDS.Edge(aLocalShape);
                // NewE = TopoDS::Edge( anEdge.EmptyCopied() );
                BB.Range(NewE, fpar, lpar);
                NewE.Orientation(TopAbs_FORWARD);
                aLocalShape = SeqVer[SeqVer.Length()].Oriented(TopAbs_FORWARD);
                BB.Add(NewE, TopoDS.Vertex(aLocalShape));
                // BB.Add( NewE, TopoDS::Vertex( SeqVer(SeqVer.Length()).Oriented(TopAbs_FORWARD) ) );
                BB.Add(NewE, Vlast.Oriented(TopAbs_REVERSED));
                NewEdges.Append(NewE);
            }

            // Substitute anEdge by NewEdges
            aSub.Substitute(anEdge, NewEdges);
        }

        // Removing edges connected with missing extremities of seam edges
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> anEntry in VEmap)
        {
            TopoDS_Shape V = anEntry.Key;
            TopoDS_Shape E = anEntry.Value;
            TopoDS_Shape W = new();
            for (i = 1; i <= Eseq.Length(); i++)
            {
                if (E.IsSame(Eseq[i]))
                {
                    W = Wseq[i];
                    break;
                }
            }
            TopoDS_Shape Etoremove = new();
            eit.Initialize(W);
            for (; eit.More(); eit.Next())
            {
                TopoDS_Edge CurE = TopoDS.Edge(eit.Value());
                if (CurE.IsSame(E))
                {
                    continue;
                }
                TopExp.Vertices(CurE, out TopoDS_Vertex Vfirst, out TopoDS_Vertex Vlast);
                if (Vfirst.IsSame(V) || Vlast.IsSame(V))
                {
                    Etoremove = CurE;
                    break;
                }
            }
            if (!Etoremove.IsNull())
            {
                W.Free(true);
                BB.Remove(W, Etoremove);
            }
        }

        aSub.Build(myShape);
        if (aSub.IsCopied(myShape))
        {
            NCollection_List<TopoDS_Shape> listSh = aSub.InternalCopy(myShape);
            if (!listSh.IsEmpty())
            {
                myShape = listSh.First();
            }
        }

        // Reconstruction of wires
        NCollection_List<TopoDS_Shape> theCopy;
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anEntry in WWmap)
        {
            CurWire = anEntry.Key;
            theCopy = aSub.InternalCopy(CurWire);
            CurWire = theCopy.First();
            CurWire.Free(true);
            NCollection_List<TopoDS_Shape>.Iterator itl = new(anEntry.Value);
            for (; itl.More(); itl.Next())
            {
                TopoDS_Shape aWire = itl.Value();
                CurFace = WFmap.Find(aWire);
                theCopy = aSub.InternalCopy(aWire);
                aWire = theCopy.First();
                // Adjusting period
                Geom_Surface? theSurf = BRep_Tool.Surface(TopoDS.Face(CurFace), out TopLoc_Location L);
                eit.Initialize(aWire);
                for (; eit.More(); eit.Next())
                {
                    TopoDS_Edge anEdge = TopoDS.Edge(eit.Value());
                    gp_Pnt2d Pmid;
                    BRep_Tool.UVPoints(anEdge, TopoDS.Face(CurFace), out gp_Pnt2d Pfirst, out gp_Pnt2d Plast);
                    BRepAdaptor_Curve2d bc2d = new(anEdge, TopoDS.Face(CurFace));
                    Pmid = bc2d.Value((bc2d.FirstParameter() + bc2d.LastParameter()) / 2.0);
                    gp_Vec2d offset = new();
                    bool translate = false;
                    if (Pfirst.X() - 2.0 * M_PI > Precision.Confusion() || Plast.X() - 2.0 * M_PI > Precision.Confusion()
                        || Pmid.X() - 2.0 * M_PI > Precision.Confusion())
                    {
                        offset.SetCoord(-2.0 * M_PI, 0);
                        translate = true;
                    }
                    if (Pfirst.X() < -Precision.Confusion() || Plast.X() < -Precision.Confusion()
                        || Pmid.X() < -Precision.Confusion())
                    {
                        offset.SetCoord(2.0 * M_PI, 0);
                        translate = true;
                    }
                    if (translate)
                    {
                        BRep_TEdge TE = (BRep_TEdge)anEdge.TShape()!;
                        NCollection_List<BRep_CurveRepresentation> aCurves = TE.ChangeCurves();
                        for (NCollection_List<BRep_CurveRepresentation>.Iterator itcr = new(aCurves); itcr.More(); itcr.Next())
                        {
                            if (itcr.Value() is BRep_GCurve GC && GC.IsCurveOnSurface(theSurf, L))
                            {
                                Geom2d_Curve PC = GC.PCurve()!;
                                PC = (Geom2d_Curve)PC.Translated(offset);
                                GC.PCurve(PC);
                                aCurves.Remove(itcr);
                                aCurves.Append(GC);
                                break;
                            }
                        }
                    }
                }
                ///////////////////
                eit.Initialize(aWire, false);
                for (; eit.More(); eit.Next())
                {
                    TopoDS_Shape anEdge = eit.Value();
                    BB.Add(CurWire, anEdge);
                }
                if (aSub.IsCopied(CurFace))
                {
                    theCopy = aSub.InternalCopy(CurFace);
                    CurFace = theCopy.First();
                }
                CurFace.Free(true);
                BB.Remove(CurFace, aWire);
            }
        }
    }

    //=================================================================================================

    private void CorrectVertexTol()
    {
        NCollection_Map<TopoDS_Shape> anInitVertices = new(TopTools_ShapeMapHasher.Instance);
        NCollection_Map<TopoDS_Shape> anInitEdges = new(TopTools_ShapeMapHasher.Instance);
        NCollection_Map<TopoDS_Shape> aNewEdges = new(TopTools_ShapeMapHasher.Instance);
        TopExp_Explorer anExp = new(myInitialShape, TopAbs_EDGE);
        for (; anExp.More(); anExp.Next())
        {
            anInitEdges.Add(anExp.Current());
            for (TopoDS_Iterator anIter = new(anExp.Current()); anIter.More(); anIter.Next())
            {
                anInitVertices.Add(anIter.Value());
            }
        }
        //

        BRep_Builder aBB = new();
        myVtxToReplace.Clear();
        anExp.Init(myShape, TopAbs_EDGE);
        for (; anExp.More(); anExp.Next())
        {
            TopoDS_Shape anE = anExp.Current();
            // Skip old (not modified) edges
            if (anInitEdges.Contains(anE))
            {
                continue;
            }
            //
            // Skip processed edges
            if (aNewEdges.Contains(anE))
            {
                continue;
            }
            //
            aNewEdges.Add(anE);
            //
            double anETol = BRep_Tool.Tolerance(TopoDS.Edge(anE));
            for (TopoDS_Iterator anIter = new(anE); anIter.More(); anIter.Next())
            {
                TopoDS_Vertex aVtx = TopoDS.Vertex(anIter.Value());
                if (anInitVertices.Contains(aVtx))
                {
                    if (myVtxToReplace.IsBound(aVtx))
                    {
                        aBB.UpdateVertex(TopoDS.Vertex(myVtxToReplace.Find(aVtx)), anETol + Epsilon(anETol));
                    }
                    else
                    {
                        double aVTol = BRep_Tool.Tolerance(aVtx);
                        if (aVTol < anETol)
                        {
                            TopoDS_Vertex aNewVtx = new();
                            gp_Pnt aVPnt = BRep_Tool.Pnt(aVtx);
                            aBB.MakeVertex(ref aNewVtx, aVPnt, anETol + Epsilon(anETol));
                            aNewVtx.Orientation(aVtx.Orientation());
                            myVtxToReplace.Bind(aVtx, aNewVtx);
                        }
                    }
                }
                else
                {
                    aBB.UpdateVertex(aVtx, anETol + Epsilon(anETol));
                }
            }
        }
        //
        if (myVtxToReplace.IsEmpty())
        {
            return;
        }
        //
        mySubs.Clear();
        // OCCT iterates the map by the vertices' addresses; the port in insertion order (the requests are independent).
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> anEntry in myVtxToReplace)
        {
            mySubs.Replace(anEntry.Key, anEntry.Value);
        }
        mySubs.InternalApply(myShape);
        myShape = mySubs.InternalValue(myShape);
        //
    }
}
