// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_ThruSections.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_ThruSections.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using static Kerf.Engine.Approximation.ParametrizationType;
using static Occt.BRepFill_ThruSectionErrorStatus;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

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
    private readonly NCollection_List<Shape> myInputWires = new(); //!< List of input wires
    private NCollection_Sequence<Shape> myWires = new();           //!< Working wires
    private readonly NCollection_Sequence<Shape> mySections = new(); // OCCT.NET: the sections as added
    private readonly NCollection_DataMap<Shape, NCollection_List<int>> myEdgeNewIndices = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, int> myVertexIndex = new(ShapeHasher.Instance);
    private int myNbEdgesInSection;
    private bool myIsSolid;
    private bool myIsRuled;
    private bool myWCheck;
    private double myPres3d;
    private Face myFirst = new();
    private Face myLast = new();
    private bool myDegen1;
    private bool myDegen2;
    private readonly NCollection_DataMap<Shape, Shape> myEdgeFace = new(ShapeHasher.Instance);
    private Continuity myContinuity;
    private ParametrizationType myParamType;
    private int myDegMax;
    private readonly double[] myCritWeights = new double[3];
    private bool myUseSmoothing;
    private bool myMutableInput;
    private BRepFill_Generator? myBFGenerator;
    private BRepFill_ThruSectionErrorStatus myStatus;
    private CopyAtEntry? myCopy; // OCCT.NET: the copy of frozen sections (decision 937)

    //=======================================================================
    // function : PreciseUpar
    // purpose  : pins the u-parameter of surface close to U-knot
    //           to this U-knot
    //=======================================================================
    private static double PreciseUpar(double anUpar, global::Kerf.Geometry.BSplineSurface aSurface)
    {
        double Tol = global::Kerf.Engine.Foundation.Precision.PConfusion();

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

    private static bool PerformPlan(in Wire W, double presPln, ref Face theFace)
    {
        bool isDegen = true;
        Iterator iter = new(W);
        for (; iter.More(); iter.Next())
        {
            Edge anEdge = ShapeCasts.Edge(iter.Value());
            if (!Tool.Degenerated(anEdge))
            {
                isDegen = false;
            }
        }
        if (isDegen)
        {
            return true;
        }

        bool Ok = false;
        if (!W.IsNull)
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

    private static bool IsSameOriented(in Shape aFace, in Shape aShell)
    {
        Explorer Explo = new(aFace, ShapeType.Edge);
        Shape anEdge = Explo.Current();
        Orientation Or1 = anEdge.Orientation();

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> EFmap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aShell, ShapeType.Edge, ShapeType.Face, EFmap);

        Shape AdjacentFace = EFmap.FindFromKey(anEdge).First();
        Shape theEdge = new();
        for (Explo.Init(AdjacentFace, ShapeType.Edge); Explo.More(); Explo.Next())
        {
            theEdge = Explo.Current();
            if (theEdge.IsSame(anEdge))
            {
                break;
            }
        }

        Orientation Or2 = theEdge.Orientation();
        return Or1 != Or2;
    }

    //=================================================================================================

    private static Solid MakeSolid(ref Shell shell,
                                          in Wire wire1,
                                          in Wire wire2,
                                          double presPln,
                                          ref Face face1,
                                          ref Face face2)
    {
        if (shell.IsNull)
        {
            throw new NotDoneException("Thrusections is not build");
        }
        bool B = shell.Closed();
        BrepBuilder BB = new();

        if (!B)
        {
            // It is necessary to close the extremities
            B = PerformPlan(wire1, presPln, ref face1);
            if (B)
            {
                B = PerformPlan(wire2, presPln, ref face2);
                if (B)
                {
                    if (!face1.IsNull && !IsSameOriented(face1, shell))
                    {
                        face1.Reverse();
                    }
                    if (!face2.IsNull && !IsSameOriented(face2, shell))
                    {
                        face2.Reverse();
                    }

                    if (!face1.IsNull)
                    {
                        BB.Add(shell, face1);
                    }
                    if (!face2.IsNull)
                    {
                        BB.Add(shell, face2);
                    }

                    shell.Closed(true);
                }
            }
        }

        Solid solid = new();
        BB.MakeSolid(ref solid);
        BB.Add(solid, shell);

        // verify the orientation the solid
        BRepClass3d_SolidClassifier clas3d = new(solid);
        clas3d.PerformInfinitePoint(global::Kerf.Engine.Foundation.Precision.Confusion());
        if (clas3d.State() == In)
        {
            BB.MakeSolid(ref solid);
            Shape aLocalShape = shell.Reversed();
            BB.Add(solid, ShapeCasts.Shell(aLocalShape));
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
        myParamType = ChordLength;
        myDegMax = 8;
        myContinuity = C2;
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
        myParamType = ChordLength;
        myDegMax = 6; // OCCT: 8 in the constructor (occt-issues.md)
        myContinuity = C2;
        myCritWeights[0] = .4;
        myCritWeights[1] = .2;
        myCritWeights[2] = .4;
        myUseSmoothing = false;
        myStatus = BRepFill_ThruSectionErrorStatus_NotDone;
    }

    //=================================================================================================

    /// <summary>Adds a section.</summary>
    public void AddWire(in Wire wire)
    {
        mySections.Append(wire);
        myInputWires.Append(wire);
    }

    //=================================================================================================

    /// <summary>Adds a point section (first or last only), as a closed wire of one degenerated edge.</summary>
    public void AddVertex(in Vertex aVertex)
    {
        BrepBuilder BB = new();

        Edge DegEdge = new();
        BB.MakeEdge(ref DegEdge);
        BB.Add(DegEdge, aVertex.Oriented(Forward));
        BB.Add(DegEdge, aVertex.Oriented(Reversed));
        BB.Degenerated(DegEdge, true);

        Wire DegWire = new();
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
    public override void Build(ProgressRange theRange = default)
    {
        // OCCT.NET: the working sections are the caller's, copied at once when they hold frozen shapes (decision 937)
        myCopy = null;
        myWires = new NCollection_Sequence<Shape>(mySections);
        bool isFrozen = false;
        for (int k = 1; k <= myWires.Length() && !isFrozen; k++)
        {
            isFrozen = BrepTools.HoldsFrozen(myWires.Value(k));
        }
        if (isFrozen)
        {
            BrepBuilder aB = new();
            Compound aHolder = new();
            aB.MakeCompound(ref aHolder);
            for (int k = 1; k <= myWires.Length(); k++)
            {
                aB.Add(aHolder, myWires.Value(k));
            }
            myCopy = new CopyAtEntry(aHolder);
            int j = 1;
            for (Iterator anIt = new(myCopy.Shape(), false, false); anIt.More(); anIt.Next(), j++)
            {
                myWires.ChangeValue(j) = anIt.Value();
            }
        }
        myFirst = new Face();
        myLast = new Face();
        myShape = new Shape();

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
                List<Shape> anOthers = [myFirst, myLast];
                for (int k = 1; k <= myWires.Length(); k++)
                {
                    anOthers.Add(myWires.Value(k));
                }
                myShape = myCopy.Finish(myShape, anOthers);
                myFirst = ShapeCasts.Face(myCopy.Final(myFirst));
                myLast = ShapeCasts.Face(myCopy.Final(myLast));
            }
        }
    }

    private void build()
    {
        myStatus = BRepFill_ThruSectionErrorStatus_Done;
        myBFGenerator = null;
        // Check set of section for right configuration of punctual sections
        int i;
        Explorer explo = new();
        for (i = 2; i <= myWires.Length() - 1; i++)
        {
            bool wdeg = true;
            for (explo.Init(myWires.Value(i), ShapeType.Edge); explo.More(); explo.Next())
            {
                Edge anEdge = ShapeCasts.Edge(explo.Current());
                wdeg = wdeg && (Tool.Degenerated(anEdge));
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
                for (explo.Init(myWires.Value(i), ShapeType.Edge); explo.More(); explo.Next())
                {
                    Edge anEdge = ShapeCasts.Edge(explo.Current());
                    wdeg = wdeg && (Tool.Degenerated(anEdge));
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
            NCollection_Sequence<Shape> WorkingSections = new();
            WorkingSections.Clear();
            // (OCCT's WorkingMap, a copy of Georges' generated map never read, is left out)

            // Calculate the working sections
            BRepFill_CompatibleWires Georges = new(myWires);
            Georges.Perform();
            if (Georges.IsDone())
            {
                WorkingSections = new NCollection_Sequence<Shape>(Georges.InternalShape());
                myDegen1 = Georges.IsDegeneratedFirstSection();
                myDegen2 = Georges.IsDegeneratedLastSection();
                // For each sub-edge of each section
                // we save its splits
                int IndFirstSec = 1;
                if (Georges.IsDegeneratedFirstSection())
                {
                    IndFirstSec = 2;
                }
                Wire aWorkingSection = ShapeCasts.Wire(WorkingSections.Value(IndFirstSec));
                myNbEdgesInSection += aWorkingSection.ChildCount;
                for (int ii = 1; ii <= myWires.Length(); ii++)
                {
                    Iterator itw = new(myWires.Value(ii));
                    for (; itw.More(); itw.Next())
                    {
                        Edge anEdge = ShapeCasts.Edge(itw.Value());
                        int aSign = 1;
                        Exploration.Vertices(anEdge, out Vertex Vfirst, out Vertex Vlast);
                        NCollection_List<Shape> aNewEdges = Georges.InternalGeneratedShapes(anEdge);
                        NCollection_List<int> IList = new();
                        aWorkingSection = ShapeCasts.Wire(WorkingSections.Value(ii));
                        int NbNewEdges = aNewEdges.Extent();
                        NCollection_List<Shape>.Iterator itl = new(aNewEdges);
                        for (int kk = 1; itl.More(); itl.Next(), kk++)
                        {
                            Edge aNewEdge = ShapeCasts.Edge(itl.Value());
                            int inde = 1;
                            WireExplorer wexp = new(aWorkingSection);
                            for (; wexp.More(); wexp.Next(), inde++)
                            {
                                Shape aWorkingEdge = wexp.Current();
                                if (aWorkingEdge.IsSame(aNewEdge))
                                {
                                    aSign = (aWorkingEdge.Orientation() == Forward) ? 1 : -1;
                                    break;
                                }
                            }
                            IList.Append(inde);
                            if (kk == 1 || kk == NbNewEdges)
                            {
                                // For each sub-vertex of each section
                                // we save its index of new edge
                                Exploration.Vertices(aNewEdge, out Vertex NewVfirst, out Vertex NewVlast);
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
            Edge anEdge = new();
            for (int ii = 1; ii <= myWires.Length(); ii++)
            {
                Explorer Explo = new(myWires.Value(ii), ShapeType.Edge);
                int inde = 1;
                for (; Explo.More(); Explo.Next(), inde++)
                {
                    anEdge = ShapeCasts.Edge(Explo.Current());
                    NCollection_List<int> IList = new();
                    IList.Append(inde);
                    myEdgeNewIndices.Bind(anEdge, IList);
                    Exploration.Vertices(anEdge, out Vertex V1, out Vertex V2);
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
                if (inde == 1 && Tool.Degenerated(anEdge))
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
        catch (FailureException)
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
            myBFGenerator.AddWire(ShapeCasts.Wire(myWires.Value(i)));
        }
        myBFGenerator.Perform();
        BRepFill_ThruSectionErrorStatus aStatus = myBFGenerator.GetStatus();
        if (aStatus != BRepFill_ThruSectionErrorStatus_Done)
        {
            myStatus = aStatus;
            return;
        }
        Shell shell = myBFGenerator.InternalShell();

        if (myIsSolid)
        {

            // check if the first wire is the same as the last
            bool vClosed = (myWires.Value(1).IsSame(myWires.Value(nbSects)));

            if (vClosed)
            {

                Solid solid = new();
                BrepBuilder B = new();
                B.MakeSolid(ref solid);
                B.Add(solid, shell);

                // verify the orientation of the solid
                BRepClass3d_SolidClassifier clas3d = new(solid);
                clas3d.PerformInfinitePoint(global::Kerf.Engine.Foundation.Precision.Confusion());
                if (clas3d.State() == In)
                {
                    B.MakeSolid(ref solid);
                    Shape aLocalShape = shell.Reversed();
                    B.Add(solid, ShapeCasts.Shell(aLocalShape));
                }
                myShape = solid;
            }

            else
            {
                // myBFGenerator stores the same 'myWires'
                Wire wire1 = ShapeCasts.Wire(myBFGenerator.InternalResultShape(myWires.First()));
                Wire wire2 = ShapeCasts.Wire(myBFGenerator.InternalResultShape(myWires.Last()));

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
        WireExplorer anExp1 = new(), anExp2 = new();
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> M = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(shell, ShapeType.Edge, ShapeType.Face, M);
        NCollection_List<Shape>.Iterator it = new();

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> MV = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(shell, ShapeType.Vertex, ShapeType.Face, MV);

        for (i = 1; i <= nbSects - 1; i++)
        {

            Wire wire1 = ShapeCasts.Wire(myWires.Value(i));
            Wire wire2 = ShapeCasts.Wire(myWires.Value(i + 1));

            anExp1.Init(wire1);
            anExp2.Init(wire2);

            bool tantque = anExp1.More() && anExp2.More();

            while (tantque)
            {

                Shape edge1 = anExp1.Current();
                Shape edge2 = anExp2.Current();
                bool degen1 = Tool.Degenerated(anExp1.Current());
                bool degen2 = Tool.Degenerated(anExp2.Current());

                NCollection_Map<Shape> MapFaces = new(ShapeHasher.Instance);
                if (degen2)
                {
                    Vertex Vdegen =
                      ShapeCasts.Vertex(myBFGenerator.InternalResultShape(Exploration.FirstVertex(ShapeCasts.Edge(edge2))));
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
                    Vertex Vdegen =
                      ShapeCasts.Vertex(myBFGenerator.InternalResultShape(Exploration.FirstVertex(ShapeCasts.Edge(edge1))));
                    for (it.Initialize(MV.FindFromKey(Vdegen)); it.More(); it.Next())
                    {
                        Shape Face = it.Value();
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
                        Shape Face = it.Value();
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
        WireExplorer anExp = new();

        bool w1Point = true;
        // check if the first wire is punctual
        for (anExp.Init(ShapeCasts.Wire(myWires.Value(1))); anExp.More(); anExp.Next())
        {
            w1Point = w1Point && (Tool.Degenerated(anExp.Current()));
        }

        bool w2Point = true;
        // check if the last wire is punctual
        for (anExp.Init(ShapeCasts.Wire(myWires.Value(nbSects))); anExp.More(); anExp.Next())
        {
            w2Point = w2Point && (Tool.Degenerated(anExp.Current()));
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
            for (anExp.Init(ShapeCasts.Wire(myWires.Value(1))); anExp.More(); anExp.Next())
            {
                nbEdges++;
            }
        }
        else
        {
            for (anExp.Init(ShapeCasts.Wire(myWires.Value(2))); anExp.More(); anExp.Next())
            {
                nbEdges++;
            }
        }

        // recover the shapes
        bool uClosed = true;
        NCollection_Array1<Shape> shapes = new(1, nbSects * nbEdges);
        int nb = 0, i, j;

        for (i = 1; i <= nbSects; i++)
        {
            Wire wire = ShapeCasts.Wire(myWires.Value(i));
            if (!wire.Closed())
            {
                // check if the vertices are the same
                Exploration.Vertices(wire, out Vertex V1, out Vertex V2);
                if (!V1.IsSame(V2))
                {
                    uClosed = false;
                }
            }
            if ((i == 1 && w1Point) || (i == nbSects && w2Point))
            {
                // if the wire is punctual
                anExp.Init(ShapeCasts.Wire(wire));
                for (j = 1; j <= nbEdges; j++)
                {
                    nb++;
                    shapes[nb] = anExp.Current();
                }
            }
            else
            {
                // otherwise
                for (anExp.Init(ShapeCasts.Wire(wire)); anExp.More(); anExp.Next())
                {
                    nb++;
                    shapes[nb] = anExp.Current();
                }
            }
        }

        // create the new surface
        Shell shell = new();
        Face face = new();
        Wire W = new();
        Edge edge, edge1 = new(), edge2 = new(), edge3 = new(), edge4 = new(), couture = new();
        NCollection_Array1<Shape> vcouture = new(1, nbEdges);

        BrepBuilder B = new();
        B.MakeShell(ref shell);

        Wire newW1 = new(), newW2 = new();
        BrepBuilder BW1 = new(), BW2 = new();
        BW1.MakeWire(ref newW1);
        BW2.MakeWire(ref newW2);

        Vertex v1f, v1l, v2f, v2l;

        // (OCCT's unused 21 x nbSects array of points is left out)

        // concatenate each section to get a total surface that will be segmented
        global::Kerf.Geometry.BSplineSurface? TS;
        TS = TotalSurf(shapes, nbSects, nbEdges, w1Point, w2Point, vClosed);

        if (TS is null)
        {
            myStatus = BRepFill_ThruSectionErrorStatus_Failed;
            return;
        }

        Shape firstEdge;
        for (i = 1; i <= nbEdges; i++)
        {

            // segmentation of TS
            global::Kerf.Geometry.BSplineSurface surface;
            surface = (global::Kerf.Geometry.BSplineSurface)TS.Copy();
            double Ui1, Ui2, V0, V1;
            Ui1 = i - 1;
            Ui2 = i;
            Ui1 = PreciseUpar(Ui1, surface);
            Ui2 = PreciseUpar(Ui2, surface);
            V0 = surface.VKnot(surface.FirstVKnotIndex());
            V1 = surface.VKnot(surface.LastVKnotIndex());
            surface.Segment(Ui1, Ui2, V0, V1);

            // return vertices
            edge = ShapeCasts.Edge(shapes[i]);
            Exploration.Vertices(edge, out v1f, out v1l);
            if (edge.Orientation() == Reversed)
            {
                Exploration.Vertices(edge, out v1l, out v1f);
            }
            firstEdge = edge;

            edge = ShapeCasts.Edge(shapes[(nbSects - 1) * nbEdges + i]);
            Exploration.Vertices(edge, out v2f, out v2l);
            if (edge.Orientation() == Reversed)
            {
                Exploration.Vertices(edge, out v2l, out v2f);
            }

            // make the face
            B.MakeFace(ref face, surface, global::Kerf.Engine.Foundation.Precision.Confusion());

            // make the wire
            B.MakeWire(ref W);

            // make the missing edges
            surface.Bounds(out double f1, out double l1, out double f2, out double l2);

            // --- edge 1
            if (w1Point)
            {
                // copy the degenerated edge
                Shape aLocalShape = shapes[1].EmptyCopied();
                edge1 = ShapeCasts.Edge(aLocalShape);
                edge1.Orientation(Forward);
            }
            else
            {
                B.MakeEdge(ref edge1, surface.VIso(f2), global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            v1f.Orientation(Forward);
            B.Add(edge1, v1f);
            v1l.Orientation(Reversed);
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
                edge2 = ShapeCasts.Edge(vcouture[i]);
            }
            else
            {
                if (w2Point)
                {
                    // copy of the degenerated edge
                    Shape aLocalShape = shapes[nbSects * nbEdges].EmptyCopied();
                    edge2 = ShapeCasts.Edge(aLocalShape);
                    edge2.Orientation(Forward);
                }
                else
                {
                    B.MakeEdge(ref edge2, surface.VIso(l2), global::Kerf.Engine.Foundation.Precision.Confusion());
                }
                v2f.Orientation(Forward);
                B.Add(edge2, v2f);
                v2l.Orientation(Reversed);
                B.Add(edge2, v2l);
                B.Range(edge2, f1, l1);
            }
            edge2.Reverse();

            // --- edge 3
            if (i == 1)
            {
                B.MakeEdge(ref edge3, surface.UIso(f1), global::Kerf.Engine.Foundation.Precision.Confusion());
                v1f.Orientation(Forward);
                B.Add(edge3, v1f);
                v2f.Orientation(Reversed);
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
                B.MakeEdge(ref edge4, surface.UIso(l1), global::Kerf.Engine.Foundation.Precision.Confusion());
                v1l.Orientation(Forward);
                B.Add(edge4, v1l);
                v2l.Orientation(Reversed);
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
                             new LineCurve2D(new Point2D(0, f2), new Direction2D(Direction2D.D.X)),
                             new LineCurve2D(new Point2D(0, l2), new Direction2D(Direction2D.D.X)),
                             face,
                             global::Kerf.Engine.Foundation.Precision.Confusion());
                B.Range(edge1, face, f1, l1);
            }
            else
            {
                B.UpdateEdge(edge1,
                             new LineCurve2D(new Point2D(0, f2), new Direction2D(Direction2D.D.X)),
                             face,
                             global::Kerf.Engine.Foundation.Precision.Confusion());
                B.Range(edge1, face, f1, l1);
                B.UpdateEdge(edge2,
                             new LineCurve2D(new Point2D(0, l2), new Direction2D(Direction2D.D.X)),
                             face,
                             global::Kerf.Engine.Foundation.Precision.Confusion());
                B.Range(edge2, face, f1, l1);
            }

            if (uClosed && nbEdges == 1)
            {
                B.UpdateEdge(edge3,
                             new LineCurve2D(new Point2D(l1, 0), new Direction2D(Direction2D.D.Y)),
                             new LineCurve2D(new Point2D(f1, 0), new Direction2D(Direction2D.D.Y)),
                             face,
                             global::Kerf.Engine.Foundation.Precision.Confusion());
                B.Range(edge3, face, f2, l2);
            }
            else
            {
                B.UpdateEdge(edge3,
                             new LineCurve2D(new Point2D(f1, 0), new Direction2D(Direction2D.D.Y)),
                             face,
                             global::Kerf.Engine.Foundation.Precision.Confusion());
                B.Range(edge3, face, f2, l2);
                B.UpdateEdge(edge4,
                             new LineCurve2D(new Point2D(l1, 0), new Direction2D(Direction2D.D.Y)),
                             face,
                             global::Kerf.Engine.Foundation.Precision.Confusion());
                B.Range(edge4, face, f2, l2);
            }
            B.Add(face, W);
            B.Add(shell, face);

            // complete newW1 newW2
            Edge edge12 = edge1;
            Edge edge22 = edge2;
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

                Solid solid = new();
                B.MakeSolid(ref solid);
                B.Add(solid, shell);

                // verify the orientation the solid
                BRepClass3d_SolidClassifier clas3d = new(solid);
                clas3d.PerformInfinitePoint(global::Kerf.Engine.Foundation.Precision.Confusion());
                if (clas3d.State() == In)
                {
                    B.MakeSolid(ref solid);
                    Shape aLocalShape = shell.Reversed();
                    B.Add(solid, ShapeCasts.Shell(aLocalShape));
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

        NCollection_DataMap<Shape, double> aVertexToleranceMap = new(ShapeHasher.Instance);
        Explorer aTopExplorer = new(myShape, ShapeType.Edge);
        while (aTopExplorer.More())
        {
            Edge aCurEdge = ShapeCasts.Edge(aTopExplorer.Current());
            B.SameRange(aCurEdge, false);
            B.SameParameter(aCurEdge, false);
            double aTolerance = Tool.Tolerance(aCurEdge);
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
                    Exploration.Vertices(aCurEdge, out Vertex aVertex1, out Vertex aVertex2);
                    if (!aVertex1.IsNull)
                    {
                        if (!aVertexToleranceMap.Find(aVertex1, out double anOldTolerance) || anOldTolerance < aNewTolerance)
                        {
                            aVertexToleranceMap.Bind(aVertex1, aNewTolerance);
                        }
                    }
                    if (!aVertex2.IsNull)
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
            ReShape aReshaper = new();
            foreach (KeyValuePair<Shape, double> aMapIterator in aVertexToleranceMap)
            {
                Vertex aVertex = ShapeCasts.Vertex(aMapIterator.Key);
                double aNewTolerance = aMapIterator.Value;
                if (Tool.Tolerance(aVertex) < aNewTolerance)
                {
                    Vertex aNnewVertex = ShapeCasts.Vertex(aVertex.EmptyCopied());
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
    private static BSplineCurve3D? EdgeToBSpline(in Edge theEdge)
    {
        BSplineCurve3D? aBSCurve = null;
        if (Tool.Degenerated(theEdge))
        {
            // degenerated edge : construction of a point curve
            NCollection_Array1<double> aKnots = new(1, 2);
            aKnots[1] = 0.0;
            aKnots[2] = 1.0;

            NCollection_Array1<int> aMults = new(1, 2);
            aMults[1] = 2;
            aMults[2] = 2;

            NCollection_Array1<Point3D> aPoles = new(1, 2);
            Exploration.Vertices(theEdge, out Vertex vl, out Vertex vf);
            aPoles[1] = Tool.Pnt(vf);
            aPoles[2] = Tool.Pnt(vl);

            aBSCurve = new BSplineCurve3D(aPoles, aKnots, aMults, 1);
        }
        else
        {
            // get the curve of the edge
            Curve3D? aCurve = Tool.Curve(theEdge, out Placement aLoc, out double aFirst, out double aLast);
            if (aCurve is null)
            {
                return null;
            }

            // convert its part used by edge to bspline; note that if edge curve is bspline,
            // conversion made via trimmed curve is still needed -- it will copy it, segment
            // as appropriate, and remove periodicity if it is periodic (deadly for approximator)
            TrimmedCurve3D aTrimCurve = new(aCurve, aFirst, aLast);

            // special treatment of conic curve
            if (aTrimCurve.BasisCurve() is ConicCurve3D)
            {
                Curve3D aCurveTrimmed = aTrimCurve; // to avoid ambiguity
                ApproxCurve anAppr = new(aCurveTrimmed, global::Kerf.Engine.Foundation.Precision.Confusion(), C1, 16, 14);
                if (anAppr.HasResult)
                {
                    aBSCurve = anAppr.InternalCurve();
                }
            }

            // general case
            if (aBSCurve is null)
            {
                aBSCurve = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(aTrimCurve);
            }

            // apply transformation if needed
            if (!aLoc.IsIdentity)
            {
                aBSCurve.Transform(aLoc.Transformation());
            }

            // reparameterize to [0,1]
            NCollection_Array1<double> aKnots = new(aBSCurve.InternalKnots());
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(0.0, 1.0, aKnots);
            aBSCurve.SetKnots(aKnots);
        }

        // reverse curve if edge is reversed
        if (theEdge.Orientation() == Reversed)
        {
            aBSCurve.Reverse();
        }

        return aBSCurve;
    }

    //=================================================================================================

    private global::Kerf.Geometry.BSplineSurface? TotalSurf(NCollection_Array1<Shape> shapes,
                                           int NbSects,
                                           int NbEdges,
                                           bool w1Point,
                                           bool w2Point,
                                           bool vClosed)
    {
        int i, j, jdeb = 1, jfin = NbSects;
        Vertex vf, vl;

        GeomFill_SectionGenerator section = new();
        global::Kerf.Geometry.BSplineSurface? surface = null;
        BSplineCurve3D? BS, BS1 = null;

        if (w1Point)
        {
            jdeb++;
            Edge edge = ShapeCasts.Edge(shapes[1]);
            Exploration.Vertices(edge, out vl, out vf);
            NCollection_Array1<Point3D> Extremities = new(1, 2);
            Extremities[1] = Tool.Pnt(vf);
            Extremities[2] = Tool.Pnt(vl);
            NCollection_Array1<double> Bounds = new(1, 2);
            Bounds[1] = 0.0;
            Bounds[2] = 1.0;
            NCollection_Array1<int> Mult = new(1, 2);
            Mult[1] = 2;
            Mult[2] = 2;
            BSplineCurve3D BSPoint = new(Extremities, Bounds, Mult, 1);
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
                Edge aPrevEdge = ShapeCasts.Edge(shapes[(j - 1) * NbEdges + 1]);
                BSplineCurve3D? curvBS = EdgeToBSpline(aPrevEdge);
                if (curvBS is null)
                {
                    return null;
                }

                // initialization
                CompCurveToBSplineCurve CompBS = new(curvBS);

                for (i = 2; i <= NbEdges; i++)
                {
                    // read the edge
                    Edge aNextEdge = ShapeCasts.Edge(shapes[(j - 1) * NbEdges + i]);
                    double aTolV = global::Kerf.Engine.Foundation.Precision.Confusion();
                    Exploration.Vertices(aNextEdge, out vf, out vl);
                    aTolV = Max(aTolV, Tool.Tolerance(vf));
                    aTolV = Max(aTolV, Tool.Tolerance(vl));
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
            Edge edge = ShapeCasts.Edge(shapes[NbSects * NbEdges]);
            Exploration.Vertices(edge, out vl, out vf);
            NCollection_Array1<Point3D> Extremities = new(1, 2);
            Extremities[1] = Tool.Pnt(vf);
            Extremities[2] = Tool.Pnt(vl);
            NCollection_Array1<double> Bounds = new(1, 2);
            Bounds[1] = 0.0;
            Bounds[2] = 1.0;
            NCollection_Array1<int> Mult = new(1, 2);
            Mult[1] = 2;
            Mult[2] = 2;
            BSplineCurve3D BSPoint = new(Extremities, Bounds, Mult, 1);
            section.AddCurve(BSPoint);
        }

        section.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());
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
            surface = new global::Kerf.Geometry.BSplineSurface(anApprox.SurfPoles(),
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
    public Shape FirstShape() => published(myFirst);

    /// <summary>The face closing the solid at the last section (frozen; null for a shell or a point section).</summary>
    public Shape LastShape() => published(myLast);

    private static Shape published(Shape theShape)
    {
        theShape.TShape()?.FreezeGraph();
        return theShape;
    }

    // OCCT.NET: a shape of the caller as the loft worked on it (its copy), and a shape the loft worked on as it stands
    // in the result (the caller's own where the loft left it unchanged), decision 937
    private Shape working(in Shape theShape) => myCopy is null ? theShape : myCopy.Copied(theShape);

    private Shape final(in Shape theShape) => myCopy is null ? theShape : myCopy.Final(theShape);

    private Shape resultShape(in Shape theShape) => final(myBFGenerator!.InternalResultShape(working(theShape)));

    private protected override void BeforePublishingLists() => myShape.TShape()?.FreezeGraph();

    //=================================================================================================

    /// <summary>
    /// The shapes generated from <paramref name="S"/>: for an edge of a section, the lateral faces built from it (from
    /// every section for a ruled loft); for a vertex, the lateral edges built from it.
    /// </summary>
    internal override NCollection_List<Shape> InternalGenerated(in Shape S)
    {
        myGenerated.Clear();

        NCollection_Sequence<Shape> AllFaces = new();
        Explorer Explo = new(myShape, ShapeType.Face);
        for (; Explo.More(); Explo.Next())
        {
            AllFaces.Append(Explo.Current());
        }

        Shape SW = working(S); // OCCT.NET: the key of the maps
        Shape SF = final(SW);  // OCCT.NET: as it stands in the result

        if (S.ShapeType() == ShapeType.Edge)
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
        else if (S.ShapeType() == ShapeType.Vertex)
        {
            if (!myVertexIndex.IsBound(SW))
            {
                return myGenerated;
            }

            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> VEmap = new(ShapeHasher.Instance);

            bool[] IsDegen = [false, false];
            if (myDegen1 || myDegen2)
            {
                Shape[] EndSections = new Shape[2];
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

                    Explo.Init(EndSections[i], ShapeType.Vertex);
                    Shape aVertex = final(Explo.Current());
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
                Exploration.MapShapesAndAncestors(myShape, ShapeType.Vertex, ShapeType.Edge, VEmap);
                NCollection_IndexedMap<Shape> Emap = new(ShapeHasher.Instance);
                Shape aNewShape = SF;
                if ((myIsRuled || !myMutableInput) && myBFGenerator is not null)
                {
                    aNewShape = resultShape(S);
                }

                NCollection_List<Shape> anEdgeList = VEmap.FindFromKey(aNewShape);
                NCollection_List<Shape>.Iterator aListIterator = new(anEdgeList);
                for (; aListIterator.More(); aListIterator.Next())
                {
                    Edge anEdge = ShapeCasts.Edge(aListIterator.Value());
                    if (!Tool.Degenerated(anEdge))
                    {
                        Vertex[] VV = new Vertex[2];
                        Exploration.Vertices(anEdge, out VV[0], out VV[1]);
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
                    Edge anEdge = ShapeCasts.Edge(Emap.FindKey(j));
                    myGenerated.Append(anEdge);
                    if (myIsRuled)
                    {
                        int i, k;
                        for (i = 2, k = myWires.Length() - 1; i < myWires.Length(); i++, k--)
                        {
                            int IndOfSec = (IsDegen[0]) ? i : k;
                            Vertex aVertex =
                              (IsDegen[0]) ? Exploration.LastVertex(anEdge) : Exploration.FirstVertex(anEdge);
                            NCollection_List<Shape> EElist = VEmap.FindFromKey(aVertex);
                            NCollection_IndexedMap<Shape> EmapOfSection = new(ShapeHasher.Instance);
                            Shape aWireSection = final(myWires.Value(IndOfSec));
                            if ((myIsRuled || !myMutableInput) && myBFGenerator is not null)
                            {
                                aWireSection = resultShape(myWires.Value(IndOfSec));
                            }
                            Exploration.MapShapes(aWireSection, ShapeType.Edge, EmapOfSection);
                            if (myCopy is not null)
                            {
                                // OCCT.NET: the section's edges as they stand in the result
                                NCollection_IndexedMap<Shape> aFinalEdges = new(ShapeHasher.Instance);
                                for (int e = 1; e <= EmapOfSection.Extent(); e++)
                                {
                                    aFinalEdges.Add(final(EmapOfSection.FindKey(e)));
                                }
                                EmapOfSection = aFinalEdges;
                            }
                            Edge NextEdge = new();
                            for (aListIterator.Initialize(EElist); aListIterator.More(); aListIterator.Next())
                            {
                                NextEdge = ShapeCasts.Edge(aListIterator.Value());
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
            Face FirstFace = ShapeCasts.Face(AllFaces.Value(Eindex));
            FirstFace.Orientation(Forward);
            Explo.Init(FirstFace, ShapeType.Edge);
            Edge anEdge2 = new();
            FaceSurfaceAdaptor BAsurf = new(FirstFace, false);
            Vertex FirstVertex = new();
            Exploration.MapShapesAndAncestors(FirstFace, ShapeType.Vertex, ShapeType.Edge, VEmap);
            if (myDegen1 && BAsurf.GetType() == Plane)
            {
                // There are only 3 edges in the face in this case:
                // we take 1-st or 3-rd edge
                if (Vindex == 0)
                {
                    Explo.Next();
                    Explo.Next();
                }
                anEdge2 = ShapeCasts.Edge(Explo.Current());
            }
            else
            {
                Edge FirstEdge = new();
                Vertex FirstVertexOfFirstEdge = new();
                Wire FirstSection = ShapeCasts.Wire(myWires.Value(1));
                WireExplorer aWireExplorer = new(FirstSection);
                for (int i = 1; aWireExplorer.More(); aWireExplorer.Next(), i++)
                {
                    FirstEdge = aWireExplorer.Current();
                    if (i == Eindex)
                    {
                        if ((myIsRuled || !myMutableInput) && myBFGenerator is not null)
                        {
                            FirstEdge = ShapeCasts.Edge(resultShape(FirstEdge));
                        }
                        else
                        {
                            FirstEdge = ShapeCasts.Edge(final(FirstEdge));
                        }
                        FirstVertexOfFirstEdge = ShapeCasts.Vertex(final(aWireExplorer.CurrentVertex()));
                        break;
                    }
                }

                Shape FirstEdgeInFace;
                FirstEdgeInFace = Explo.Current();
                Vertex[] VV = new Vertex[2];
                Exploration.Vertices(FirstEdge, out VV[0], out VV[1]);
                VV[0] = ShapeCasts.Vertex(final(VV[0])); // OCCT.NET: a section's edge may be outside the result
                VV[1] = ShapeCasts.Vertex(final(VV[1]));
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
                NCollection_List<Shape> Elist = VEmap.FindFromKey(FirstVertex);
                NCollection_List<Shape>.Iterator itl = new(Elist);
                Orientation anEdgeOr = (Vindex == 0) ? Reversed : Forward;
                for (; itl.More(); itl.Next())
                {
                    anEdge2 = ShapeCasts.Edge(itl.Value());
                    if (!anEdge2.IsSame(FirstEdgeInFace) && !Tool.Degenerated(anEdge2)
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
                    FirstVertex = Exploration.LastVertex(anEdge2);
                    NCollection_List<Shape> Elist1 = VEmap.FindFromKey(FirstVertex);
                    Edge FirstEdge = (anEdge2.IsSame(Elist1.First())) ? ShapeCasts.Edge(Elist1.Last())
                                                                             : ShapeCasts.Edge(Elist1.First());
                    Eindex += myNbEdgesInSection;
                    FirstFace = ShapeCasts.Face(AllFaces.Value(Eindex));
                    FirstFace.Orientation(Forward);
                    VEmap.Clear();
                    Exploration.MapShapesAndAncestors(FirstFace, ShapeType.Vertex, ShapeType.Edge, VEmap);
                    NCollection_List<Shape> Elist2 = VEmap.FindFromKey(FirstVertex);
                    anEdge2 = (FirstEdge.IsSame(Elist2.First())) ? ShapeCasts.Edge(Elist2.Last())
                                                                 : ShapeCasts.Edge(Elist2.First());
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
    public Shape GeneratedFace(in Shape Edge)
    {
        BeforePublishingLists();
        return published(InternalGeneratedFace(Edge));
    }

    /// <summary><see cref="GeneratedFace"/> without freezing.</summary>
    internal Shape InternalGeneratedFace(in Shape edge)
    {
        Shape bid = new();
        Shape aKey = working(edge);
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
    public void SetContinuity(Continuity TheCont)
    {
        myContinuity = TheCont;
    }

    /// <summary>The continuity of the smoothed surfaces.</summary>
    public Continuity Continuity()
    {
        return myContinuity;
    }

    //=================================================================================================

    /// <summary>Sets the parametrisation of the approximation (chord length by default).</summary>
    public void SetParType(ParametrizationType ParType)
    {
        myParamType = ParType;
    }

    /// <summary>The parametrisation of the approximation.</summary>
    public ParametrizationType ParType()
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
    public ISchematiqList<Shape> Wires() => PublishedList(myInputWires);

    /// <summary>The status of the last <see cref="Build"/>.</summary>
    public BRepFill_ThruSectionErrorStatus GetStatus() => myStatus;
}
