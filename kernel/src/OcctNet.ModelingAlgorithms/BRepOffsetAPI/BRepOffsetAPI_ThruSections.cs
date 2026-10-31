// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_ThruSections.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_ThruSections.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;
using static Occt.Approx_ParametrizationType;
using static Occt.BRepFill_ThruSectionErrorStatus;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Builds a loft: a shell or a solid passing through a set of sections (wires; a first or last section may be a
/// point). The loft is ruled (planar, cylindrical, conical or ruled faces between consecutive sections) or smoothed
/// (B-spline faces approximated through all the sections).
/// </summary>
/// <remarks>
/// <para>
/// OCCT's loft writes into the sections it is given: pcurves, ranges and flags into their edges, tolerances into their
/// vertices (by default, <see cref="IsMutableInput"/> true). The sections a caller gives (whose TShapes are frozen)
/// are copied at <see cref="Build"/>, all at once so that the sub-shapes they share stay shared; the result holds the
/// caller's sub-shapes wherever the loft left them unchanged, and the history (<see cref="BRepBuilderAPI_MakeShape.Generated"/>,
/// <see cref="GeneratedFace"/>) answers for the caller's edges and vertices (decision 937). The caller's shapes are
/// never written.
/// </para>
/// <para>
/// The result, <see cref="FirstShape"/>, <see cref="LastShape"/>, <see cref="GeneratedFace"/> and the history are
/// frozen when returned.
/// </para>
/// </remarks>
public class BRepOffsetAPI_ThruSections : BRepBuilderAPI_MakeShape
{
    private readonly NCollection_List<TopoDS_Shape> myInputWires = new(); //!< List of input wires
    private NCollection_Sequence<TopoDS_Shape> myWires = new();           //!< Working wires
    private readonly NCollection_Sequence<TopoDS_Shape> mySections = new(); // OCCT.NET: the sections as added
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<int>> myEdgeNewIndices = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, int> myVertexIndex = new(TopTools_ShapeMapHasher.Instance);
    private int myNbEdgesInSection;
    private bool myIsSolid;
    private bool myIsRuled;
    private bool myWCheck;
    private double myPres3d;
    private TopoDS_Face myFirst = new();
    private TopoDS_Face myLast = new();
    private bool myDegen1;
    private bool myDegen2;
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myEdgeFace = new(TopTools_ShapeMapHasher.Instance);
    private GeomAbs_Shape myContinuity;
    private Approx_ParametrizationType myParamType;
    private int myDegMax;
    private readonly double[] myCritWeights = new double[3];
    private bool myUseSmoothing;
    private bool myMutableInput;
    private BRepFill_Generator? myBFGenerator;
    private BRepFill_ThruSectionErrorStatus myStatus;
    private BRepTools_CopyAtEntry? myCopy; // OCCT.NET: the copy of frozen sections (decision 937)

    //=======================================================================
    // function : PreciseUpar
    // purpose  : pins the u-parameter of surface close to U-knot
    //           to this U-knot
    //=======================================================================
    private static double PreciseUpar(double anUpar, Geom_BSplineSurface aSurface)
    {
        double Tol = Precision.PConfusion();

        aSurface.LocateU(anUpar, Tol, out int i1, out int i2);
        double U1 = aSurface.UKnot(i1);
        double U2 = aSurface.UKnot(i2);

        double NewU = anUpar;

        NewU = (anUpar - U1 < U2 - anUpar) ? U1 : U2;
        return NewU;
    }

    //=======================================================================
    // function :  PerformPlan
    // purpose  : Construct a plane of filling if exists
    //=======================================================================

    private static bool PerformPlan(in TopoDS_Wire W, double presPln, ref TopoDS_Face theFace)
    {
        bool isDegen = true;
        TopoDS_Iterator iter = new(W);
        for (; iter.More(); iter.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(iter.Value());
            if (!BRep_Tool.Degenerated(anEdge))
            {
                isDegen = false;
            }
        }
        if (isDegen)
        {
            return true;
        }

        bool Ok = false;
        if (!W.IsNull())
        {
            BRepBuilderAPI_FindPlane Searcher = new(W, presPln);
            if (Searcher.Found())
            {
                theFace = new BRepBuilderAPI_MakeFace(Searcher.Plane()!, W).InternalFace();
                Ok = true;
            }
            else // try to find another surface
            {
                BRepBuilderAPI_MakeFace MF = new(W);
                if (MF.IsDone())
                {
                    theFace = MF.InternalFace();
                    Ok = true;
                }
            }
        }

        return Ok;
    }

    //=============================================================================
    // function :  IsSameOriented
    // purpose  : Checks whether aFace is oriented to the same side as aShell or not
    //=============================================================================

    private static bool IsSameOriented(in TopoDS_Shape aFace, in TopoDS_Shape aShell)
    {
        TopExp_Explorer Explo = new(aFace, TopAbs_EDGE);
        TopoDS_Shape anEdge = Explo.Current();
        TopAbs_Orientation Or1 = anEdge.Orientation();

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> EFmap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aShell, TopAbs_EDGE, TopAbs_FACE, EFmap);

        TopoDS_Shape AdjacentFace = EFmap.FindFromKey(anEdge).First();
        TopoDS_Shape theEdge = new();
        for (Explo.Init(AdjacentFace, TopAbs_EDGE); Explo.More(); Explo.Next())
        {
            theEdge = Explo.Current();
            if (theEdge.IsSame(anEdge))
            {
                break;
            }
        }

        TopAbs_Orientation Or2 = theEdge.Orientation();
        return Or1 != Or2;
    }

    //=================================================================================================

    private static TopoDS_Solid MakeSolid(ref TopoDS_Shell shell,
                                          in TopoDS_Wire wire1,
                                          in TopoDS_Wire wire2,
                                          double presPln,
                                          ref TopoDS_Face face1,
                                          ref TopoDS_Face face2)
    {
        if (shell.IsNull())
        {
            throw new StdFail_NotDone("Thrusections is not build");
        }
        bool B = shell.Closed();
        BRep_Builder BB = new();

        if (!B)
        {
            // It is necessary to close the extremities
            B = PerformPlan(wire1, presPln, ref face1);
            if (B)
            {
                B = PerformPlan(wire2, presPln, ref face2);
                if (B)
                {
                    if (!face1.IsNull() && !IsSameOriented(face1, shell))
                    {
                        face1.Reverse();
                    }
                    if (!face2.IsNull() && !IsSameOriented(face2, shell))
                    {
                        face2.Reverse();
                    }

                    if (!face1.IsNull())
                    {
                        BB.Add(shell, face1);
                    }
                    if (!face2.IsNull())
                    {
                        BB.Add(shell, face2);
                    }

                    shell.Closed(true);
                }
            }
        }

        TopoDS_Solid solid = new();
        BB.MakeSolid(ref solid);
        BB.Add(solid, shell);

        // verify the orientation the solid
        BRepClass3d_SolidClassifier clas3d = new(solid);
        clas3d.PerformInfinitePoint(Precision.Confusion());
        if (clas3d.State() == TopAbs_IN)
        {
            BB.MakeSolid(ref solid);
            TopoDS_Shape aLocalShape = shell.Reversed();
            BB.Add(solid, TopoDS.Shell(aLocalShape));
        }

        solid.Closed(true);
        return solid;
    }

    //=================================================================================================

    /// <summary>
    /// A loft: a solid (<paramref name="isSolid"/>) or a shell, ruled (<paramref name="ruled"/>) or smoothed, with
    /// the tolerance <paramref name="pres3d"/> of the approximation and of the end planes.
    /// </summary>
    public BRepOffsetAPI_ThruSections(bool isSolid = false, bool ruled = false, double pres3d = 1.0e-06)
    {
        myNbEdgesInSection = 0;
        myIsSolid = isSolid;
        myIsRuled = ruled;
        myPres3d = pres3d;
        myDegen1 = false;
        myDegen2 = false;
        myWCheck = true;
        myMutableInput = true;
        //----------------------------
        myParamType = Approx_ChordLength;
        myDegMax = 8;
        myContinuity = GeomAbs_C2;
        myCritWeights[0] = .4;
        myCritWeights[1] = .2;
        myCritWeights[2] = .4;
        myUseSmoothing = false;
        myStatus = BRepFill_ThruSectionErrorStatus_NotDone;
    }

    //=================================================================================================

    /// <summary>
    /// Resets the options (the sections stay). As in OCCT, the maximal degree becomes 6, where the constructor sets 8.
    /// </summary>
    public void Init(bool isSolid = false, bool ruled = false, double pres3d = 1.0e-06)
    {
        myIsSolid = isSolid;
        myIsRuled = ruled;
        myPres3d = pres3d;
        myWCheck = true;
        myMutableInput = true;
        //----------------------------
        myParamType = Approx_ChordLength;
        myDegMax = 6; // OCCT: 8 in the constructor (occt-issues.md)
        myContinuity = GeomAbs_C2;
        myCritWeights[0] = .4;
        myCritWeights[1] = .2;
        myCritWeights[2] = .4;
        myUseSmoothing = false;
        myStatus = BRepFill_ThruSectionErrorStatus_NotDone;
    }

    //=================================================================================================

    /// <summary>Adds a section.</summary>
    public void AddWire(in TopoDS_Wire wire)
    {
        mySections.Append(wire);
        myInputWires.Append(wire);
    }

    //=================================================================================================

    /// <summary>Adds a point section (first or last only), as a closed wire of one degenerated edge.</summary>
    public void AddVertex(in TopoDS_Vertex aVertex)
    {
        BRep_Builder BB = new();

        TopoDS_Edge DegEdge = new();
        BB.MakeEdge(ref DegEdge);
        BB.Add(DegEdge, aVertex.Oriented(TopAbs_FORWARD));
        BB.Add(DegEdge, aVertex.Oriented(TopAbs_REVERSED));
        BB.Degenerated(DegEdge, true);

        TopoDS_Wire DegWire = new();
        BB.MakeWire(ref DegWire);
        BB.Add(DegWire, DegEdge);
        DegWire.Closed(true);

        mySections.Append(DegWire);
        myInputWires.Append(DegWire);
    }

    //=================================================================================================

    /// <summary>
    /// Sets whether the sections are made compatible (same number of edges, origins and orientations chosen to avoid
    /// twisting) by BRepFill_CompatibleWires; true by default.
    /// </summary>
    public void CheckCompatibility(bool check = true)
    {
        myWCheck = check;
    }

    //=================================================================================================

    /// <summary>Builds the loft.</summary>
    public override void Build(Message_ProgressRange theRange = default)
    {
        // OCCT.NET: the working sections are the caller's, copied at once when they hold frozen shapes (decision 937)
        myCopy = null;
        myWires = new NCollection_Sequence<TopoDS_Shape>(mySections);
        bool isFrozen = false;
        for (int k = 1; k <= myWires.Length() && !isFrozen; k++)
        {
            isFrozen = BRepTools.HoldsFrozen(myWires.Value(k));
        }
        if (isFrozen)
        {
            BRep_Builder aB = new();
            TopoDS_Compound aHolder = new();
            aB.MakeCompound(ref aHolder);
            for (int k = 1; k <= myWires.Length(); k++)
            {
                aB.Add(aHolder, myWires.Value(k));
            }
            myCopy = new BRepTools_CopyAtEntry(aHolder);
            int j = 1;
            for (TopoDS_Iterator anIt = new(myCopy.Shape(), false, false); anIt.More(); anIt.Next(), j++)
            {
                myWires.ChangeValue(j) = anIt.Value();
            }
        }
        myFirst = new TopoDS_Face();
        myLast = new TopoDS_Face();
        myShape = new TopoDS_Shape();

        try
        {
            build();
        }
        finally
        {
            if (myCopy is not null)
            {
                // the result holds the caller's sub-shapes the loft left unchanged; so do the working sections, whose
                // edges keep sharing their vertices with the result's (Generated reads them)
                List<TopoDS_Shape> anOthers = [myFirst, myLast];
                for (int k = 1; k <= myWires.Length(); k++)
                {
                    anOthers.Add(myWires.Value(k));
                }
                myShape = myCopy.Finish(myShape, anOthers);
                myFirst = TopoDS.Face(myCopy.Final(myFirst));
                myLast = TopoDS.Face(myCopy.Final(myLast));
            }
        }
    }

    private void build()
    {
        myStatus = BRepFill_ThruSectionErrorStatus_Done;
        myBFGenerator = null;
        // Check set of section for right configuration of punctual sections
        int i;
        TopExp_Explorer explo = new();
        for (i = 2; i <= myWires.Length() - 1; i++)
        {
            bool wdeg = true;
            for (explo.Init(myWires.Value(i), TopAbs_EDGE); explo.More(); explo.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(explo.Current());
                wdeg = wdeg && (BRep_Tool.Degenerated(anEdge));
            }
            if (wdeg)
            {
                myStatus = BRepFill_ThruSectionErrorStatus_WrongUsage;
                return;
            }
        }
        if (myWires.Length() <= 2)
        {
            bool wdeg = true;
            for (i = 1; i <= myWires.Length(); i++)
            {
                for (explo.Init(myWires.Value(i), TopAbs_EDGE); explo.More(); explo.Next())
                {
                    TopoDS_Edge anEdge = TopoDS.Edge(explo.Current());
                    wdeg = wdeg && (BRep_Tool.Degenerated(anEdge));
                }
            }
            if (wdeg)
            {
                myStatus = BRepFill_ThruSectionErrorStatus_WrongUsage;
                return;
            }
        }

        myNbEdgesInSection = 0;

        if (myWCheck)
        {
            // compute origin and orientation on wires to avoid twisted results
            // and update wires to have same number of edges

            // use BRepFill_CompatibleWires
            NCollection_Sequence<TopoDS_Shape> WorkingSections = new();
            WorkingSections.Clear();
            // (OCCT's WorkingMap, a copy of Georges' generated map never read, is left out)

            // Calculate the working sections
            BRepFill_CompatibleWires Georges = new(myWires);
            Georges.Perform();
            if (Georges.IsDone())
            {
                WorkingSections = new NCollection_Sequence<TopoDS_Shape>(Georges.InternalShape());
                myDegen1 = Georges.IsDegeneratedFirstSection();
                myDegen2 = Georges.IsDegeneratedLastSection();
                // For each sub-edge of each section
                // we save its splits
                int IndFirstSec = 1;
                if (Georges.IsDegeneratedFirstSection())
                {
                    IndFirstSec = 2;
                }
                TopoDS_Wire aWorkingSection = TopoDS.Wire(WorkingSections.Value(IndFirstSec));
                myNbEdgesInSection += aWorkingSection.NbChildren();
                for (int ii = 1; ii <= myWires.Length(); ii++)
                {
                    TopoDS_Iterator itw = new(myWires.Value(ii));
                    for (; itw.More(); itw.Next())
                    {
                        TopoDS_Edge anEdge = TopoDS.Edge(itw.Value());
                        int aSign = 1;
                        TopExp.Vertices(anEdge, out TopoDS_Vertex Vfirst, out TopoDS_Vertex Vlast);
                        NCollection_List<TopoDS_Shape> aNewEdges = Georges.InternalGeneratedShapes(anEdge);
                        NCollection_List<int> IList = new();
                        aWorkingSection = TopoDS.Wire(WorkingSections.Value(ii));
                        int NbNewEdges = aNewEdges.Extent();
                        NCollection_List<TopoDS_Shape>.Iterator itl = new(aNewEdges);
                        for (int kk = 1; itl.More(); itl.Next(), kk++)
                        {
                            TopoDS_Edge aNewEdge = TopoDS.Edge(itl.Value());
                            int inde = 1;
                            BRepTools_WireExplorer wexp = new(aWorkingSection);
                            for (; wexp.More(); wexp.Next(), inde++)
                            {
                                TopoDS_Shape aWorkingEdge = wexp.Current();
                                if (aWorkingEdge.IsSame(aNewEdge))
                                {
                                    aSign = (aWorkingEdge.Orientation() == TopAbs_FORWARD) ? 1 : -1;
                                    break;
                                }
                            }
                            IList.Append(inde);
                            if (kk == 1 || kk == NbNewEdges)
                            {
                                // For each sub-vertex of each section
                                // we save its index of new edge
                                TopExp.Vertices(aNewEdge, out TopoDS_Vertex NewVfirst, out TopoDS_Vertex NewVlast);
                                if (NewVfirst.IsSame(Vfirst) && !myVertexIndex.IsBound(Vfirst))
                                {
                                    myVertexIndex.Bind(Vfirst, aSign * inde);
                                }
                                if (NewVlast.IsSame(Vlast) && !myVertexIndex.IsBound(Vlast))
                                {
                                    myVertexIndex.Bind(Vlast, aSign * (-inde));
                                }
                            }
                        }
                        myEdgeNewIndices.Bind(anEdge, IList);
                    }
                }
            }
            else
            {
                myStatus = Georges.GetStatus();
                NotDone();
                return;
            }

            myWires = WorkingSections;
        } // if (myWCheck)
        else // no check
        {
            TopoDS_Edge anEdge = new();
            for (int ii = 1; ii <= myWires.Length(); ii++)
            {
                TopExp_Explorer Explo = new(myWires.Value(ii), TopAbs_EDGE);
                int inde = 1;
                for (; Explo.More(); Explo.Next(), inde++)
                {
                    anEdge = TopoDS.Edge(Explo.Current());
                    NCollection_List<int> IList = new();
                    IList.Append(inde);
                    myEdgeNewIndices.Bind(anEdge, IList);
                    TopExp.Vertices(anEdge, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                    if (!myVertexIndex.IsBound(V1))
                    {
                        myVertexIndex.Bind(V1, inde);
                    }
                    if (!myVertexIndex.IsBound(V2))
                    {
                        myVertexIndex.Bind(V2, -inde);
                    }
                }
                inde--;
                if (inde > myNbEdgesInSection)
                {
                    myNbEdgesInSection = inde;
                }
                if (inde == 1 && BRep_Tool.Degenerated(anEdge))
                {
                    if (ii == 1)
                    {
                        myDegen1 = true;
                    }
                    else
                    {
                        myDegen2 = true;
                    }
                }
            }
        }

        try
        {
            // Calculate the resulting shape
            if (myWires.Length() == 2 || myIsRuled)
            {
                // create a ruled shell
                CreateRuled();
            }
            else
            {
                // create a smoothed shell
                CreateSmoothed();
            }
        }
        catch (Standard_Failure)
        {
            NotDone();
            return;
        }

        if (myStatus != BRepFill_ThruSectionErrorStatus_Done)
        {
            NotDone();
            return;
        }
        // Encode the Regularities
        BRepLib.EncodeRegularity(myShape);
    }

    //=================================================================================================

    private void CreateRuled()
    {
        int nbSects = myWires.Length();
        myBFGenerator = new BRepFill_Generator();
        myBFGenerator.SetMutableInput(IsMutableInput());
        //  for (int i=1; i<=nbSects; i++) {
        int i;
        for (i = 1; i <= nbSects; i++)
        {
            myBFGenerator.AddWire(TopoDS.Wire(myWires.Value(i)));
        }
        myBFGenerator.Perform();
        BRepFill_ThruSectionErrorStatus aStatus = myBFGenerator.GetStatus();
        if (aStatus != BRepFill_ThruSectionErrorStatus_Done)
        {
            myStatus = aStatus;
            return;
        }
        TopoDS_Shell shell = myBFGenerator.InternalShell();

        if (myIsSolid)
        {

            // check if the first wire is the same as the last
            bool vClosed = (myWires.Value(1).IsSame(myWires.Value(nbSects)));

            if (vClosed)
            {

                TopoDS_Solid solid = new();
                BRep_Builder B = new();
                B.MakeSolid(ref solid);
                B.Add(solid, shell);

                // verify the orientation of the solid
                BRepClass3d_SolidClassifier clas3d = new(solid);
                clas3d.PerformInfinitePoint(Precision.Confusion());
                if (clas3d.State() == TopAbs_IN)
                {
                    B.MakeSolid(ref solid);
                    TopoDS_Shape aLocalShape = shell.Reversed();
                    B.Add(solid, TopoDS.Shell(aLocalShape));
                }
                myShape = solid;
            }

            else
            {
                // myBFGenerator stores the same 'myWires'
                TopoDS_Wire wire1 = TopoDS.Wire(myBFGenerator.InternalResultShape(myWires.First()));
                TopoDS_Wire wire2 = TopoDS.Wire(myBFGenerator.InternalResultShape(myWires.Last()));

                myShape = MakeSolid(ref shell, wire1, wire2, myPres3d, ref myFirst, ref myLast);
            }

            Done();
        }

        else
        {
            myShape = shell;
            Done();
        }

        // history
        BRepTools_WireExplorer anExp1 = new(), anExp2 = new();
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> M = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(shell, TopAbs_EDGE, TopAbs_FACE, M);
        NCollection_List<TopoDS_Shape>.Iterator it = new();

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> MV = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(shell, TopAbs_VERTEX, TopAbs_FACE, MV);

        for (i = 1; i <= nbSects - 1; i++)
        {

            TopoDS_Wire wire1 = TopoDS.Wire(myWires.Value(i));
            TopoDS_Wire wire2 = TopoDS.Wire(myWires.Value(i + 1));

            anExp1.Init(wire1);
            anExp2.Init(wire2);

            bool tantque = anExp1.More() && anExp2.More();

            while (tantque)
            {

                TopoDS_Shape edge1 = anExp1.Current();
                TopoDS_Shape edge2 = anExp2.Current();
                bool degen1 = BRep_Tool.Degenerated(anExp1.Current());
                bool degen2 = BRep_Tool.Degenerated(anExp2.Current());

                NCollection_Map<TopoDS_Shape> MapFaces = new(TopTools_ShapeMapHasher.Instance);
                if (degen2)
                {
                    TopoDS_Vertex Vdegen =
                      TopoDS.Vertex(myBFGenerator.InternalResultShape(TopExp.FirstVertex(TopoDS.Edge(edge2))));
                    for (it.Initialize(MV.FindFromKey(Vdegen)); it.More(); it.Next())
                    {
                        MapFaces.Add(it.Value());
                    }
                }
                else
                {
                    for (it.Initialize(M.FindFromKey(myBFGenerator.InternalResultShape(edge2))); it.More(); it.Next())
                    {
                        MapFaces.Add(it.Value());
                    }
                }

                if (degen1)
                {
                    TopoDS_Vertex Vdegen =
                      TopoDS.Vertex(myBFGenerator.InternalResultShape(TopExp.FirstVertex(TopoDS.Edge(edge1))));
                    for (it.Initialize(MV.FindFromKey(Vdegen)); it.More(); it.Next())
                    {
                        TopoDS_Shape Face = it.Value();
                        if (MapFaces.Contains(Face))
                        {
                            myEdgeFace.Bind(edge1, Face);
                            break;
                        }
                    }
                }
                else
                {
                    for (it.Initialize(M.FindFromKey(myBFGenerator.InternalResultShape(edge1))); it.More(); it.Next())
                    {
                        TopoDS_Shape Face = it.Value();
                        if (MapFaces.Contains(Face))
                        {
                            myEdgeFace.Bind(edge1, Face);
                            break;
                        }
                    }
                }

                if (!degen1)
                {
                    anExp1.Next();
                }
                if (!degen2)
                {
                    anExp2.Next();
                }

                tantque = anExp1.More() && anExp2.More();
                if (degen1)
                {
                    tantque = anExp2.More();
                }
                if (degen2)
                {
                    tantque = anExp1.More();
                }
            }
        }
    }

    //=================================================================================================


    private void CreateSmoothed()
    {
        // initialisation
        int nbSects = myWires.Length();
        BRepTools_WireExplorer anExp = new();

        bool w1Point = true;
        // check if the first wire is punctual
        for (anExp.Init(TopoDS.Wire(myWires.Value(1))); anExp.More(); anExp.Next())
        {
            w1Point = w1Point && (BRep_Tool.Degenerated(anExp.Current()));
        }

        bool w2Point = true;
        // check if the last wire is punctual
        for (anExp.Init(TopoDS.Wire(myWires.Value(nbSects))); anExp.More(); anExp.Next())
        {
            w2Point = w2Point && (BRep_Tool.Degenerated(anExp.Current()));
        }

        bool vClosed = false;
        // check if the first wire is the same as last
        if (myWires.Value(1).IsSame(myWires.Value(myWires.Length())))
        {
            vClosed = true;
        }

        // find the dimension
        int nbEdges = 0;
        if (!w1Point)
        {
            for (anExp.Init(TopoDS.Wire(myWires.Value(1))); anExp.More(); anExp.Next())
            {
                nbEdges++;
            }
        }
        else
        {
            for (anExp.Init(TopoDS.Wire(myWires.Value(2))); anExp.More(); anExp.Next())
            {
                nbEdges++;
            }
        }

        // recover the shapes
        bool uClosed = true;
        NCollection_Array1<TopoDS_Shape> shapes = new(1, nbSects * nbEdges);
        int nb = 0, i, j;

        for (i = 1; i <= nbSects; i++)
        {
            TopoDS_Wire wire = TopoDS.Wire(myWires.Value(i));
            if (!wire.Closed())
            {
                // check if the vertices are the same
                TopExp.Vertices(wire, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                if (!V1.IsSame(V2))
                {
                    uClosed = false;
                }
            }
            if ((i == 1 && w1Point) || (i == nbSects && w2Point))
            {
                // if the wire is punctual
                anExp.Init(TopoDS.Wire(wire));
                for (j = 1; j <= nbEdges; j++)
                {
                    nb++;
                    shapes[nb] = anExp.Current();
                }
            }
            else
            {
                // otherwise
                for (anExp.Init(TopoDS.Wire(wire)); anExp.More(); anExp.Next())
                {
                    nb++;
                    shapes[nb] = anExp.Current();
                }
            }
        }

        // create the new surface
        TopoDS_Shell shell = new();
        TopoDS_Face face = new();
        TopoDS_Wire W = new();
        TopoDS_Edge edge, edge1 = new(), edge2 = new(), edge3 = new(), edge4 = new(), couture = new();
        NCollection_Array1<TopoDS_Shape> vcouture = new(1, nbEdges);

        BRep_Builder B = new();
        B.MakeShell(ref shell);

        TopoDS_Wire newW1 = new(), newW2 = new();
        BRep_Builder BW1 = new(), BW2 = new();
        BW1.MakeWire(ref newW1);
        BW2.MakeWire(ref newW2);

        TopoDS_Vertex v1f, v1l, v2f, v2l;

        // (OCCT's unused 21 x nbSects array of points is left out)

        // concatenate each section to get a total surface that will be segmented
        Geom_BSplineSurface? TS;
        TS = TotalSurf(shapes, nbSects, nbEdges, w1Point, w2Point, vClosed);

        if (TS is null)
        {
            myStatus = BRepFill_ThruSectionErrorStatus_Failed;
            return;
        }

        TopoDS_Shape firstEdge;
        for (i = 1; i <= nbEdges; i++)
        {

            // segmentation of TS
            Geom_BSplineSurface surface;
            surface = (Geom_BSplineSurface)TS.Copy();
            double Ui1, Ui2, V0, V1;
            Ui1 = i - 1;
            Ui2 = i;
            Ui1 = PreciseUpar(Ui1, surface);
            Ui2 = PreciseUpar(Ui2, surface);
            V0 = surface.VKnot(surface.FirstVKnotIndex());
            V1 = surface.VKnot(surface.LastVKnotIndex());
            surface.Segment(Ui1, Ui2, V0, V1);

            // return vertices
            edge = TopoDS.Edge(shapes[i]);
            TopExp.Vertices(edge, out v1f, out v1l);
            if (edge.Orientation() == TopAbs_REVERSED)
            {
                TopExp.Vertices(edge, out v1l, out v1f);
            }
            firstEdge = edge;

            edge = TopoDS.Edge(shapes[(nbSects - 1) * nbEdges + i]);
            TopExp.Vertices(edge, out v2f, out v2l);
            if (edge.Orientation() == TopAbs_REVERSED)
            {
                TopExp.Vertices(edge, out v2l, out v2f);
            }

            // make the face
            B.MakeFace(ref face, surface, Precision.Confusion());

            // make the wire
            B.MakeWire(ref W);

            // make the missing edges
            surface.Bounds(out double f1, out double l1, out double f2, out double l2);

            // --- edge 1
            if (w1Point)
            {
                // copy the degenerated edge
                TopoDS_Shape aLocalShape = shapes[1].EmptyCopied();
                edge1 = TopoDS.Edge(aLocalShape);
                edge1.Orientation(TopAbs_FORWARD);
            }
            else
            {
                B.MakeEdge(ref edge1, surface.VIso(f2), Precision.Confusion());
            }
            v1f.Orientation(TopAbs_FORWARD);
            B.Add(edge1, v1f);
            v1l.Orientation(TopAbs_REVERSED);
            B.Add(edge1, v1l);
            B.Range(edge1, f1, l1);
            // processing of looping sections
            // store edges of the 1st section
            if (vClosed)
            {
                vcouture[i] = edge1;
            }

            // --- edge 2
            if (vClosed)
            {
                edge2 = TopoDS.Edge(vcouture[i]);
            }
            else
            {
                if (w2Point)
                {
                    // copy of the degenerated edge
                    TopoDS_Shape aLocalShape = shapes[nbSects * nbEdges].EmptyCopied();
                    edge2 = TopoDS.Edge(aLocalShape);
                    edge2.Orientation(TopAbs_FORWARD);
                }
                else
                {
                    B.MakeEdge(ref edge2, surface.VIso(l2), Precision.Confusion());
                }
                v2f.Orientation(TopAbs_FORWARD);
                B.Add(edge2, v2f);
                v2l.Orientation(TopAbs_REVERSED);
                B.Add(edge2, v2l);
                B.Range(edge2, f1, l1);
            }
            edge2.Reverse();

            // --- edge 3
            if (i == 1)
            {
                B.MakeEdge(ref edge3, surface.UIso(f1), Precision.Confusion());
                v1f.Orientation(TopAbs_FORWARD);
                B.Add(edge3, v1f);
                v2f.Orientation(TopAbs_REVERSED);
                B.Add(edge3, v2f);
                B.Range(edge3, f2, l2);
                if (uClosed)
                {
                    couture = edge3;
                }
            }
            else
            {
                edge3 = edge4;
            }
            edge3.Reverse();

            // --- edge 4
            if (uClosed && i == nbEdges)
            {
                edge4 = couture;
            }
            else
            {
                B.MakeEdge(ref edge4, surface.UIso(l1), Precision.Confusion());
                v1l.Orientation(TopAbs_FORWARD);
                B.Add(edge4, v1l);
                v2l.Orientation(TopAbs_REVERSED);
                B.Add(edge4, v2l);
                B.Range(edge4, f2, l2);
            }

            B.Add(W, edge1);
            B.Add(W, edge4);
            B.Add(W, edge2);
            B.Add(W, edge3);

            // set PCurve
            if (vClosed)
            {
                B.UpdateEdge(edge1,
                             new Geom2d_Line(new gp_Pnt2d(0, f2), new gp_Dir2d(gp_Dir2d.D.X)),
                             new Geom2d_Line(new gp_Pnt2d(0, l2), new gp_Dir2d(gp_Dir2d.D.X)),
                             face,
                             Precision.Confusion());
                B.Range(edge1, face, f1, l1);
            }
            else
            {
                B.UpdateEdge(edge1,
                             new Geom2d_Line(new gp_Pnt2d(0, f2), new gp_Dir2d(gp_Dir2d.D.X)),
                             face,
                             Precision.Confusion());
                B.Range(edge1, face, f1, l1);
                B.UpdateEdge(edge2,
                             new Geom2d_Line(new gp_Pnt2d(0, l2), new gp_Dir2d(gp_Dir2d.D.X)),
                             face,
                             Precision.Confusion());
                B.Range(edge2, face, f1, l1);
            }

            if (uClosed && nbEdges == 1)
            {
                B.UpdateEdge(edge3,
                             new Geom2d_Line(new gp_Pnt2d(l1, 0), new gp_Dir2d(gp_Dir2d.D.Y)),
                             new Geom2d_Line(new gp_Pnt2d(f1, 0), new gp_Dir2d(gp_Dir2d.D.Y)),
                             face,
                             Precision.Confusion());
                B.Range(edge3, face, f2, l2);
            }
            else
            {
                B.UpdateEdge(edge3,
                             new Geom2d_Line(new gp_Pnt2d(f1, 0), new gp_Dir2d(gp_Dir2d.D.Y)),
                             face,
                             Precision.Confusion());
                B.Range(edge3, face, f2, l2);
                B.UpdateEdge(edge4,
                             new Geom2d_Line(new gp_Pnt2d(l1, 0), new gp_Dir2d(gp_Dir2d.D.Y)),
                             face,
                             Precision.Confusion());
                B.Range(edge4, face, f2, l2);
            }
            B.Add(face, W);
            B.Add(shell, face);

            // complete newW1 newW2
            TopoDS_Edge edge12 = edge1;
            TopoDS_Edge edge22 = edge2;
            edge12.Reverse();
            edge22.Reverse();
            BW1.Add(newW1, edge12);
            BW2.Add(newW2, edge22);

            // history
            myEdgeFace.Bind(firstEdge, face);
        }

        if (uClosed && w1Point && w2Point)
        {
            shell.Closed(true);
        }

        if (myIsSolid)
        {

            if (vClosed)
            {

                TopoDS_Solid solid = new();
                B.MakeSolid(ref solid);
                B.Add(solid, shell);

                // verify the orientation the solid
                BRepClass3d_SolidClassifier clas3d = new(solid);
                clas3d.PerformInfinitePoint(Precision.Confusion());
                if (clas3d.State() == TopAbs_IN)
                {
                    B.MakeSolid(ref solid);
                    TopoDS_Shape aLocalShape = shell.Reversed();
                    B.Add(solid, TopoDS.Shell(aLocalShape));
                }
                myShape = solid;
            }

            else
            {
                myShape = MakeSolid(ref shell, newW1, newW2, myPres3d, ref myFirst, ref myLast);
            }
        }

        else
        {
            myShape = shell;
        }
        Done();

        NCollection_DataMap<TopoDS_Shape, double> aVertexToleranceMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp_Explorer aTopExplorer = new(myShape, TopAbs_EDGE);
        while (aTopExplorer.More())
        {
            TopoDS_Edge aCurEdge = TopoDS.Edge(aTopExplorer.Current());
            B.SameRange(aCurEdge, false);
            B.SameParameter(aCurEdge, false);
            double aTolerance = BRep_Tool.Tolerance(aCurEdge);
            if (myMutableInput)
            {
                BRepLib.SameParameter(aCurEdge, aTolerance);
            }
            else
            {
                // all edges from myShape can be safely updated/changed
                // all vertices from myShape are the part of the original wires
                double aNewTolerance = -1;
                BRepLib.SameParameter(aCurEdge, aTolerance, ref aNewTolerance, true);
                if (aNewTolerance > 0)
                {
                    TopExp.Vertices(aCurEdge, out TopoDS_Vertex aVertex1, out TopoDS_Vertex aVertex2);
                    if (!aVertex1.IsNull())
                    {
                        if (!aVertexToleranceMap.Find(aVertex1, out double anOldTolerance) || anOldTolerance < aNewTolerance)
                        {
                            aVertexToleranceMap.Bind(aVertex1, aNewTolerance);
                        }
                    }
                    if (!aVertex2.IsNull())
                    {
                        if (!aVertexToleranceMap.Find(aVertex2, out double anOldTolerance) || anOldTolerance < aNewTolerance)
                        {
                            aVertexToleranceMap.Bind(aVertex2, aNewTolerance);
                        }
                    }
                }
            }
            aTopExplorer.Next();
        }

        if (!myMutableInput)
        {
            BRepTools_ReShape aReshaper = new();
            foreach (KeyValuePair<TopoDS_Shape, double> aMapIterator in aVertexToleranceMap)
            {
                TopoDS_Vertex aVertex = TopoDS.Vertex(aMapIterator.Key);
                double aNewTolerance = aMapIterator.Value;
                if (BRep_Tool.Tolerance(aVertex) < aNewTolerance)
                {
                    TopoDS_Vertex aNnewVertex = TopoDS.Vertex(aVertex.EmptyCopied());
                    B.UpdateVertex(aNnewVertex, aNewTolerance);
                    aReshaper.Replace(aVertex, aNnewVertex);
                }
            }
            myShape = aReshaper.InternalApply(myShape);
        }
    }

    //=======================================================================
    // function : EdgeToBSpline
    // purpose  : auxiliary -- get curve from edge and convert it to bspline
    //           parameterized from 0 to 1
    //=======================================================================

    // NOTE: this code duplicates the same function in BRepFill_NSections.cxx
    private static Geom_BSplineCurve? EdgeToBSpline(in TopoDS_Edge theEdge)
    {
        Geom_BSplineCurve? aBSCurve = null;
        if (BRep_Tool.Degenerated(theEdge))
        {
            // degenerated edge : construction of a point curve
            NCollection_Array1<double> aKnots = new(1, 2);
            aKnots[1] = 0.0;
            aKnots[2] = 1.0;

            NCollection_Array1<int> aMults = new(1, 2);
            aMults[1] = 2;
            aMults[2] = 2;

            NCollection_Array1<gp_Pnt> aPoles = new(1, 2);
            TopExp.Vertices(theEdge, out TopoDS_Vertex vl, out TopoDS_Vertex vf);
            aPoles[1] = BRep_Tool.Pnt(vf);
            aPoles[2] = BRep_Tool.Pnt(vl);

            aBSCurve = new Geom_BSplineCurve(aPoles, aKnots, aMults, 1);
        }
        else
        {
            // get the curve of the edge
            Geom_Curve? aCurve = BRep_Tool.Curve(theEdge, out TopLoc_Location aLoc, out double aFirst, out double aLast);
            if (aCurve is null)
            {
                return null;
            }

            // convert its part used by edge to bspline; note that if edge curve is bspline,
            // conversion made via trimmed curve is still needed -- it will copy it, segment
            // as appropriate, and remove periodicity if it is periodic (deadly for approximator)
            Geom_TrimmedCurve aTrimCurve = new(aCurve, aFirst, aLast);

            // special treatment of conic curve
            if (aTrimCurve.BasisCurve() is Geom_Conic)
            {
                Geom_Curve aCurveTrimmed = aTrimCurve; // to avoid ambiguity
                GeomConvert_ApproxCurve anAppr = new(aCurveTrimmed, Precision.Confusion(), GeomAbs_C1, 16, 14);
                if (anAppr.HasResult())
                {
                    aBSCurve = anAppr.InternalCurve();
                }
            }

            // general case
            if (aBSCurve is null)
            {
                aBSCurve = GeomConvert.CurveToBSplineCurve(aTrimCurve);
            }

            // apply transformation if needed
            if (!aLoc.IsIdentity())
            {
                aBSCurve.Transform(aLoc.Transformation());
            }

            // reparameterize to [0,1]
            NCollection_Array1<double> aKnots = new(aBSCurve.InternalKnots());
            BSplCLib.Reparametrize(0.0, 1.0, aKnots);
            aBSCurve.SetKnots(aKnots);
        }

        // reverse curve if edge is reversed
        if (theEdge.Orientation() == TopAbs_REVERSED)
        {
            aBSCurve.Reverse();
        }

        return aBSCurve;
    }

    //=================================================================================================

    private Geom_BSplineSurface? TotalSurf(NCollection_Array1<TopoDS_Shape> shapes,
                                           int NbSects,
                                           int NbEdges,
                                           bool w1Point,
                                           bool w2Point,
                                           bool vClosed)
    {
        int i, j, jdeb = 1, jfin = NbSects;
        TopoDS_Vertex vf, vl;

        GeomFill_SectionGenerator section = new();
        Geom_BSplineSurface? surface = null;
        Geom_BSplineCurve? BS, BS1 = null;

        if (w1Point)
        {
            jdeb++;
            TopoDS_Edge edge = TopoDS.Edge(shapes[1]);
            TopExp.Vertices(edge, out vl, out vf);
            NCollection_Array1<gp_Pnt> Extremities = new(1, 2);
            Extremities[1] = BRep_Tool.Pnt(vf);
            Extremities[2] = BRep_Tool.Pnt(vl);
            NCollection_Array1<double> Bounds = new(1, 2);
            Bounds[1] = 0.0;
            Bounds[2] = 1.0;
            NCollection_Array1<int> Mult = new(1, 2);
            Mult[1] = 2;
            Mult[2] = 2;
            Geom_BSplineCurve BSPoint = new(Extremities, Bounds, Mult, 1);
            section.AddCurve(BSPoint);
        }

        if (w2Point)
        {
            jfin--;
        }

        for (j = jdeb; j <= jfin; j++)
        {

            // case of looping sections
            if (j == jfin && vClosed)
            {
                section.AddCurve(BS1!);
            }

            else
            {
                // read the first edge to initialise CompBS;
                TopoDS_Edge aPrevEdge = TopoDS.Edge(shapes[(j - 1) * NbEdges + 1]);
                Geom_BSplineCurve? curvBS = EdgeToBSpline(aPrevEdge);
                if (curvBS is null)
                {
                    return null;
                }

                // initialization
                GeomConvert_CompCurveToBSplineCurve CompBS = new(curvBS);

                for (i = 2; i <= NbEdges; i++)
                {
                    // read the edge
                    TopoDS_Edge aNextEdge = TopoDS.Edge(shapes[(j - 1) * NbEdges + i]);
                    double aTolV = Precision.Confusion();
                    TopExp.Vertices(aNextEdge, out vf, out vl);
                    aTolV = Max(aTolV, BRep_Tool.Tolerance(vf));
                    aTolV = Max(aTolV, BRep_Tool.Tolerance(vl));
                    aTolV = Min(aTolV, 1.0e-3);
                    curvBS = EdgeToBSpline(aNextEdge);
                    if (curvBS is null)
                    {
                        return null;
                    }

                    // concatenation
                    CompBS.Add(curvBS, aTolV, true, false, 1);
                }

                // return the final section
                BS = CompBS.InternalBSplineCurve();
                section.AddCurve(BS!);

                // case of looping sections
                if (j == jdeb && vClosed)
                {
                    BS1 = BS;
                }
            }
        }

        if (w2Point)
        {
            TopoDS_Edge edge = TopoDS.Edge(shapes[NbSects * NbEdges]);
            TopExp.Vertices(edge, out vl, out vf);
            NCollection_Array1<gp_Pnt> Extremities = new(1, 2);
            Extremities[1] = BRep_Tool.Pnt(vf);
            Extremities[2] = BRep_Tool.Pnt(vl);
            NCollection_Array1<double> Bounds = new(1, 2);
            Bounds[1] = 0.0;
            Bounds[2] = 1.0;
            NCollection_Array1<int> Mult = new(1, 2);
            Mult[1] = 2;
            Mult[2] = 2;
            Geom_BSplineCurve BSPoint = new(Extremities, Bounds, Mult, 1);
            section.AddCurve(BSPoint);
        }

        section.Perform(Precision.PConfusion());
        GeomFill_Line line = new(NbSects);

        int nbIt = 3;
        if (myPres3d <= 1.0e-3)
        {
            nbIt = 0;
        }

        int degmin = 2, degmax = Max(myDegMax, degmin);
        bool SpApprox = true;

        GeomFill_AppSurf anApprox = new(degmin, degmax, myPres3d, myPres3d, nbIt);
        anApprox.SetContinuity(myContinuity);

        if (myUseSmoothing)
        {
            anApprox.SetCriteriumWeight(myCritWeights[0], myCritWeights[1], myCritWeights[2]);
            anApprox.PerformSmoothing(line, section);
        }
        else
        {
            anApprox.SetParType(myParamType);
            anApprox.Perform(line, section, SpApprox);
        }

        if (anApprox.IsDone())
        {
            surface = new Geom_BSplineSurface(anApprox.SurfPoles(),
                                              anApprox.SurfWeights(),
                                              anApprox.SurfUKnots(),
                                              anApprox.SurfVKnots(),
                                              anApprox.SurfUMults(),
                                              anApprox.SurfVMults(),
                                              anApprox.UDegree(),
                                              anApprox.VDegree());
        }

        return surface;
    }

    //=================================================================================================

    /// <summary>The face closing the solid at the first section (frozen; null for a shell or a point section).</summary>
    public TopoDS_Shape FirstShape() => published(myFirst);

    /// <summary>The face closing the solid at the last section (frozen; null for a shell or a point section).</summary>
    public TopoDS_Shape LastShape() => published(myLast);

    private static TopoDS_Shape published(TopoDS_Shape theShape)
    {
        theShape.TShape()?.FreezeGraph();
        return theShape;
    }

    // OCCT.NET: a shape of the caller as the loft worked on it (its copy), and a shape the loft worked on as it stands
    // in the result (the caller's own where the loft left it unchanged), decision 937
    private TopoDS_Shape working(in TopoDS_Shape theShape) => myCopy is null ? theShape : myCopy.Copied(theShape);

    private TopoDS_Shape final(in TopoDS_Shape theShape) => myCopy is null ? theShape : myCopy.Final(theShape);

    private TopoDS_Shape resultShape(in TopoDS_Shape theShape) => final(myBFGenerator!.InternalResultShape(working(theShape)));

    private protected override void BeforePublishingLists() => myShape.TShape()?.FreezeGraph();

    //=================================================================================================

    /// <summary>
    /// The shapes generated from <paramref name="S"/>: for an edge of a section, the lateral faces built from it (from
    /// every section for a ruled loft); for a vertex, the lateral edges built from it.
    /// </summary>
    internal override NCollection_List<TopoDS_Shape> InternalGenerated(in TopoDS_Shape S)
    {
        myGenerated.Clear();

        NCollection_Sequence<TopoDS_Shape> AllFaces = new();
        TopExp_Explorer Explo = new(myShape, TopAbs_FACE);
        for (; Explo.More(); Explo.Next())
        {
            AllFaces.Append(Explo.Current());
        }

        TopoDS_Shape SW = working(S); // OCCT.NET: the key of the maps
        TopoDS_Shape SF = final(SW);  // OCCT.NET: as it stands in the result

        if (S.ShapeType() == TopAbs_EDGE)
        {
            if (!myEdgeNewIndices.IsBound(SW))
            {
                return myGenerated;
            }

            NCollection_List<int> Indices = myEdgeNewIndices.Find(SW);
            // Append the faces corresponding to <Indices>
            // These faces "grow" from the first section
            NCollection_List<int>.Iterator itl = new(Indices);
            for (; itl.More(); itl.Next())
            {
                int IndOfFace = itl.Value();
                if (AllFaces.Length() < IndOfFace)
                {
                    continue;
                }
                myGenerated.Append(AllFaces.Value(IndOfFace));
            }

            if (myIsRuled)
            {
                // Append the next faces corresponding to <Indices>
                for (int i = 2; i < myWires.Length(); i++)
                {
                    for (itl.Initialize(Indices); itl.More(); itl.Next())
                    {
                        int IndOfFace = itl.Value();
                        IndOfFace += (i - 1) * myNbEdgesInSection;
                        if (AllFaces.Length() < IndOfFace)
                        {
                            continue;
                        }
                        myGenerated.Append(AllFaces.Value(IndOfFace));
                    }
                }
            }
        }
        else if (S.ShapeType() == TopAbs_VERTEX)
        {
            if (!myVertexIndex.IsBound(SW))
            {
                return myGenerated;
            }

            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> VEmap = new(TopTools_ShapeMapHasher.Instance);

            bool[] IsDegen = [false, false];
            if (myDegen1 || myDegen2)
            {
                TopoDS_Shape[] EndSections = new TopoDS_Shape[2];
                EndSections[0] = myWires.Value(1);
                EndSections[1] = myWires.Value(myWires.Length());
                for (int i = 0; i < 2; i++)
                {
                    if (i == 0 && !myDegen1)
                    {
                        continue;
                    }
                    if (i == 1 && !myDegen2)
                    {
                        continue;
                    }

                    Explo.Init(EndSections[i], TopAbs_VERTEX);
                    TopoDS_Shape aVertex = final(Explo.Current());
                    if (SF.IsSame(aVertex))
                    {
                        IsDegen[i] = true;
                        break;
                    }
                }
            }
            // Only one of <IsDegen> can be True:
            // in case of one vertex for start and end degenerated sections
            // IsDegen[0] is True;
            if (IsDegen[0] || IsDegen[1])
            {
                // For start or end degenerated section
                // we return the whole bunch of longitudinal edges
                TopExp.MapShapesAndAncestors(myShape, TopAbs_VERTEX, TopAbs_EDGE, VEmap);
                NCollection_IndexedMap<TopoDS_Shape> Emap = new(TopTools_ShapeMapHasher.Instance);
                TopoDS_Shape aNewShape = SF;
                if ((myIsRuled || !myMutableInput) && myBFGenerator is not null)
                {
                    aNewShape = resultShape(S);
                }

                NCollection_List<TopoDS_Shape> anEdgeList = VEmap.FindFromKey(aNewShape);
                NCollection_List<TopoDS_Shape>.Iterator aListIterator = new(anEdgeList);
                for (; aListIterator.More(); aListIterator.Next())
                {
                    TopoDS_Edge anEdge = TopoDS.Edge(aListIterator.Value());
                    if (!BRep_Tool.Degenerated(anEdge))
                    {
                        TopoDS_Vertex[] VV = new TopoDS_Vertex[2];
                        TopExp.Vertices(anEdge, out VV[0], out VV[1]);
                        // Comprehensive check for possible case of
                        // one vertex for start and end degenerated sections:
                        // we must take only outgoing or only ingoing edges
                        if ((IsDegen[0] && aNewShape.IsSame(VV[0])) || (IsDegen[1] && aNewShape.IsSame(VV[1])))
                        {
                            Emap.Add(anEdge);
                        }
                    }
                }
                for (int j = 1; j <= Emap.Extent(); j++)
                {
                    TopoDS_Edge anEdge = TopoDS.Edge(Emap.FindKey(j));
                    myGenerated.Append(anEdge);
                    if (myIsRuled)
                    {
                        int i, k;
                        for (i = 2, k = myWires.Length() - 1; i < myWires.Length(); i++, k--)
                        {
                            int IndOfSec = (IsDegen[0]) ? i : k;
                            TopoDS_Vertex aVertex =
                              (IsDegen[0]) ? TopExp.LastVertex(anEdge) : TopExp.FirstVertex(anEdge);
                            NCollection_List<TopoDS_Shape> EElist = VEmap.FindFromKey(aVertex);
                            NCollection_IndexedMap<TopoDS_Shape> EmapOfSection = new(TopTools_ShapeMapHasher.Instance);
                            TopoDS_Shape aWireSection = final(myWires.Value(IndOfSec));
                            if ((myIsRuled || !myMutableInput) && myBFGenerator is not null)
                            {
                                aWireSection = resultShape(myWires.Value(IndOfSec));
                            }
                            TopExp.MapShapes(aWireSection, TopAbs_EDGE, EmapOfSection);
                            if (myCopy is not null)
                            {
                                // OCCT.NET: the section's edges as they stand in the result
                                NCollection_IndexedMap<TopoDS_Shape> aFinalEdges = new(TopTools_ShapeMapHasher.Instance);
                                for (int e = 1; e <= EmapOfSection.Extent(); e++)
                                {
                                    aFinalEdges.Add(final(EmapOfSection.FindKey(e)));
                                }
                                EmapOfSection = aFinalEdges;
                            }
                            TopoDS_Edge NextEdge = new();
                            for (aListIterator.Initialize(EElist); aListIterator.More(); aListIterator.Next())
                            {
                                NextEdge = TopoDS.Edge(aListIterator.Value());
                                if (!NextEdge.IsSame(anEdge) && !EmapOfSection.Contains(NextEdge))
                                {
                                    break;
                                }
                            }
                            myGenerated.Append(NextEdge);
                            anEdge = NextEdge;
                        }
                    }
                }
                return myGenerated;
            } // end of if (IsDegen[0] || IsDegen[1])

            int Eindex = myVertexIndex.Find(SW);
            int Vindex = (Eindex > 0) ? 0 : 1;
            Eindex = Math.Abs(Eindex);

            // Find the first longitudinal edge
            TopoDS_Face FirstFace = TopoDS.Face(AllFaces.Value(Eindex));
            FirstFace.Orientation(TopAbs_FORWARD);
            Explo.Init(FirstFace, TopAbs_EDGE);
            TopoDS_Edge anEdge2 = new();
            BRepAdaptor_Surface BAsurf = new(FirstFace, false);
            TopoDS_Vertex FirstVertex = new();
            TopExp.MapShapesAndAncestors(FirstFace, TopAbs_VERTEX, TopAbs_EDGE, VEmap);
            if (myDegen1 && BAsurf.GetType() == GeomAbs_Plane)
            {
                // There are only 3 edges in the face in this case:
                // we take 1-st or 3-rd edge
                if (Vindex == 0)
                {
                    Explo.Next();
                    Explo.Next();
                }
                anEdge2 = TopoDS.Edge(Explo.Current());
            }
            else
            {
                TopoDS_Edge FirstEdge = new();
                TopoDS_Vertex FirstVertexOfFirstEdge = new();
                TopoDS_Wire FirstSection = TopoDS.Wire(myWires.Value(1));
                BRepTools_WireExplorer aWireExplorer = new(FirstSection);
                for (int i = 1; aWireExplorer.More(); aWireExplorer.Next(), i++)
                {
                    FirstEdge = aWireExplorer.Current();
                    if (i == Eindex)
                    {
                        if ((myIsRuled || !myMutableInput) && myBFGenerator is not null)
                        {
                            FirstEdge = TopoDS.Edge(resultShape(FirstEdge));
                        }
                        else
                        {
                            FirstEdge = TopoDS.Edge(final(FirstEdge));
                        }
                        FirstVertexOfFirstEdge = TopoDS.Vertex(final(aWireExplorer.CurrentVertex()));
                        break;
                    }
                }

                TopoDS_Shape FirstEdgeInFace;
                FirstEdgeInFace = Explo.Current();
                TopoDS_Vertex[] VV = new TopoDS_Vertex[2];
                TopExp.Vertices(FirstEdge, out VV[0], out VV[1]);
                VV[0] = TopoDS.Vertex(final(VV[0])); // OCCT.NET: a section's edge may be outside the result
                VV[1] = TopoDS.Vertex(final(VV[1]));
                if (Vindex == 0)
                {
                    if (VV[0].IsSame(FirstVertexOfFirstEdge))
                    {
                        FirstVertex = VV[0];
                    }
                    else
                    {
                        FirstVertex = VV[1];
                    }
                }
                else // Vindex == 1
                {
                    if (VV[0].IsSame(FirstVertexOfFirstEdge))
                    {
                        FirstVertex = VV[1];
                    }
                    else
                    {
                        FirstVertex = VV[0];
                    }
                }
                NCollection_List<TopoDS_Shape> Elist = VEmap.FindFromKey(FirstVertex);
                NCollection_List<TopoDS_Shape>.Iterator itl = new(Elist);
                TopAbs_Orientation anEdgeOr = (Vindex == 0) ? TopAbs_REVERSED : TopAbs_FORWARD;
                for (; itl.More(); itl.Next())
                {
                    anEdge2 = TopoDS.Edge(itl.Value());
                    if (!anEdge2.IsSame(FirstEdgeInFace) && !BRep_Tool.Degenerated(anEdge2)
                        && anEdge2.Orientation() == anEdgeOr)
                    {
                        break;
                    }
                }
            }
            myGenerated.Append(anEdge2);
            if (myIsRuled)
            {
                // Find the chain of longitudinal edges from first to last
                for (int i = 2; i < myWires.Length(); i++)
                {
                    FirstVertex = TopExp.LastVertex(anEdge2);
                    NCollection_List<TopoDS_Shape> Elist1 = VEmap.FindFromKey(FirstVertex);
                    TopoDS_Edge FirstEdge = (anEdge2.IsSame(Elist1.First())) ? TopoDS.Edge(Elist1.Last())
                                                                             : TopoDS.Edge(Elist1.First());
                    Eindex += myNbEdgesInSection;
                    FirstFace = TopoDS.Face(AllFaces.Value(Eindex));
                    FirstFace.Orientation(TopAbs_FORWARD);
                    VEmap.Clear();
                    TopExp.MapShapesAndAncestors(FirstFace, TopAbs_VERTEX, TopAbs_EDGE, VEmap);
                    NCollection_List<TopoDS_Shape> Elist2 = VEmap.FindFromKey(FirstVertex);
                    anEdge2 = (FirstEdge.IsSame(Elist2.First())) ? TopoDS.Edge(Elist2.Last())
                                                                 : TopoDS.Edge(Elist2.First());
                    myGenerated.Append(anEdge2);
                }
            }
        }

        return myGenerated;
    }

    //=================================================================================================

    /// <summary>
    /// The face of the loft generated by the edge <paramref name="Edge"/> of the first section (or of a section, for a
    /// ruled loft), frozen; a null shape for another edge.
    /// </summary>
    public TopoDS_Shape GeneratedFace(in TopoDS_Shape Edge)
    {
        BeforePublishingLists();
        return published(InternalGeneratedFace(Edge));
    }

    /// <summary><see cref="GeneratedFace"/> without freezing.</summary>
    internal TopoDS_Shape InternalGeneratedFace(in TopoDS_Shape edge)
    {
        TopoDS_Shape bid = new();
        TopoDS_Shape aKey = working(edge);
        if (myEdgeFace.IsBound(aKey))
        {
            return myEdgeFace.Find(aKey);
        }
        else
        {
            return bid;
        }
    }

    //=======================================================================
    // function : CriteriumWeight
    // purpose  : returns the Weights associated  to the criterium used in
    //           the  optimization.
    //=======================================================================
    //
    /// <summary>The weights of the smoothing criterion.</summary>
    public void CriteriumWeight(out double W1, out double W2, out double W3)
    {
        W1 = myCritWeights[0];
        W2 = myCritWeights[1];
        W3 = myCritWeights[2];
    }

    //=================================================================================================

    /// <summary>
    /// Sets the weights of the smoothing criterion (length, curvature, torsion); a negative weight sets the status
    /// to Failed and changes nothing (as in OCCT, the next <see cref="Build"/> resets the status).
    /// </summary>
    public void SetCriteriumWeight(double W1, double W2, double W3)
    {
        if (W1 < 0 || W2 < 0 || W3 < 0)
        {
            myStatus = BRepFill_ThruSectionErrorStatus_Failed;
            return;
        }
        myCritWeights[0] = W1;
        myCritWeights[1] = W2;
        myCritWeights[2] = W3;
    }

    //=================================================================================================

    /// <summary>Sets the continuity of the smoothed surfaces (C2 by default).</summary>
    public void SetContinuity(GeomAbs_Shape TheCont)
    {
        myContinuity = TheCont;
    }

    /// <summary>The continuity of the smoothed surfaces.</summary>
    public GeomAbs_Shape Continuity()
    {
        return myContinuity;
    }

    //=================================================================================================

    /// <summary>Sets the parametrisation of the approximation (chord length by default).</summary>
    public void SetParType(Approx_ParametrizationType ParType)
    {
        myParamType = ParType;
    }

    /// <summary>The parametrisation of the approximation.</summary>
    public Approx_ParametrizationType ParType()
    {
        return myParamType;
    }

    //=================================================================================================

    /// <summary>Sets the maximal U degree of the smoothed surfaces (8 by default).</summary>
    public void SetMaxDegree(int MaxDeg)
    {
        myDegMax = MaxDeg;
    }

    /// <summary>The maximal U degree of the smoothed surfaces.</summary>
    public int MaxDegree()
    {
        return myDegMax;
    }

    //=================================================================================================

    /// <summary>Sets whether the approximation uses the smoothing criterion (false by default).</summary>
    public void SetSmoothing(bool UseVar)
    {
        myUseSmoothing = UseVar;
    }

    /// <summary>True if the approximation uses the smoothing criterion.</summary>
    public bool UseSmoothing()
    {
        return myUseSmoothing;
    }

    //=================================================================================================

    /// <summary>
    /// Sets the mutable input state: when true (the default), the loft writes into its (working) sections' edges and
    /// vertices; when false, into copies of them. The caller's shapes are never written either way.
    /// </summary>
    public void SetMutableInput(bool theIsMutableInput)
    {
        myMutableInput = theIsMutableInput;
    }

    /// <summary>The mutable input state.</summary>
    public bool IsMutableInput()
    {
        return myMutableInput;
    }

    /// <summary>The sections as given (a point section as its wire of one degenerated edge), frozen.</summary>
    public IBentoList<TopoDS_Shape> Wires() => PublishedList(myInputWires);

    /// <summary>The status of the last <see cref="Build"/>.</summary>
    public BRepFill_ThruSectionErrorStatus GetStatus() => myStatus;
}
