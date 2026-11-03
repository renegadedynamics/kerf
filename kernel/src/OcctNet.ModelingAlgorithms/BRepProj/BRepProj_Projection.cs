// Port of: src/ModelingAlgorithms/TKBool/BRepProj/BRepProj_Projection.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepProj/BRepProj_Projection.lxx
// Port of: src/ModelingAlgorithms/TKBool/BRepProj/BRepProj_Projection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// The projection of a wire or an edge onto a shape, along a direction (cylindrical projection) or from a point
/// (conical projection): the section of the shape's faces by the prism or cone the wire sweeps, as connected wires.
/// </summary>
/// <remarks>
/// <para>
/// As in OCCT, the swept surface is built from a transformed copy of the wire, so neither the wire nor the shape is
/// written. <see cref="Current"/> and <see cref="Shape"/> hand out frozen shapes (decision 938).
/// </para>
/// </remarks>
public sealed class BRepProj_Projection
{
    private bool myIsDone;
    // (OCCT's myLsh, never used, is left out)
    private Compound myShape = new();
    private NCollection_HSequence<Shape>? mySection;
    private int myItr;

    //=======================================================================
    // function : DistanceOut
    // purpose  : Compute the minimum distance between input shapes
    //           (using Bounding Boxes of each Shape)
    //=======================================================================
    private static double DistanceOut(in Shape S1, in Shape S2)
    {
        Box BBox1 = new(), BBox2 = new();
        BRepBndLib.Add(S1, ref BBox1);
        BRepBndLib.Add(S2, ref BBox2);
        return BBox1.Distance(BBox2);
    }

    //=======================================================================
    // function : DistanceIn
    // purpose  : Compute the maximum distance between input Shapes
    //           we compute the maximum dimension of each Bounding Box and then
    //           add each other with the minimum distance of shapes.
    //=======================================================================

    private static double DistanceIn(in Shape S1, in Shape S2)
    {
        Box LBBox = new(), SBBox = new();
        BRepBndLib.Add(S1, ref SBBox);
        BRepBndLib.Add(S2, ref LBBox);

        SBBox.Get(out double SXmin, out double SYmin, out double SZmin, out double SXmax, out double SYmax, out double SZmax);
        LBBox.Get(out double LXmin, out double LYmin, out double LZmin, out double LXmax, out double LYmax, out double LZmax);

        // Compute the max distance between input shapes------------//
        Xyz Lmin = new(LXmin, LYmin, LZmin), Lmax = new(LXmax, LYmax, LZmax);
        Xyz Smin = new(SXmin, SYmin, SZmin), Smax = new(SXmax, SYmax, SZmax);
        Lmax.Subtract(Lmin);
        Smax.Subtract(Smin);
        return Lmax.Modulus() + Smax.Modulus() + DistanceOut(S1, S2);
    }

    //=======================================================================
    // function : BuildSection
    // purpose  : Cuts theShape by theTool using BRepAlgoAPI_Section and
    //           stores result as set of connected wires and compound
    //=======================================================================

    private void BuildSection(in Shape theShape, in Shape theTool)
    {
        myIsDone = false;
        mySection = null;
        myShape = new Compound();
        myItr = 0;

        // if theShape is compound, extract only faces -- section algorithm
        // may refuse to work if e.g. vertex is present
        Shape aShape = new();
        if (theShape.ShapeType() == ShapeType.Face || theShape.ShapeType() == ShapeType.Shell
            || theShape.ShapeType() == ShapeType.Solid || theShape.ShapeType() == ShapeType.CompSolid)
        {
            aShape = theShape;
        }
        else if (theShape.ShapeType() == ShapeType.Compound)
        {
            Compound C = new();
            BrepBuilder B = new();
            Explorer exp = new(theShape, ShapeType.Face);
            for (; exp.More(); exp.Next())
            {
                if (C.IsNull)
                {
                    B.MakeCompound(ref C);
                }
                B.Add(C, exp.Current());
            }
            aShape = C;
        }
        if (aShape.IsNull)
        {
            throw new ConstructionErrorException("BRepProj_Projection.cxx: target shape has no faces");
        }

        // build section computing p-curves on both shapes to get higher precision
        BRepAlgoAPI_Section aSectionTool = new(aShape, theTool, false);
        aSectionTool.Approximation(true);
        aSectionTool.ComputePCurveOn1(true);
        aSectionTool.ComputePCurveOn2(true);
        // Use Oriented Bounding Boxes inside Booleans to speed up calculation of the section
        aSectionTool.SetUseOBB(true);
        // OCCT.NET: the faces of a compound target are gathered in a new compound, which isn't locked, so the section
        // would run destructively and write pcurves into the caller's (frozen) edges, as OCCT writes into its target.
        // A target holding frozen shapes is sectioned non-destructively, as a frozen solid or face is (decision 939).
        if (BrepTools.HoldsFrozen(aShape))
        {
            aSectionTool.SetNonDestructive(true);
        }
        aSectionTool.Build();

        // check for successful work of the section tool
        if (!aSectionTool.IsDone())
        {
            return;
        }

        // get edges of the result
        NCollection_HSequence<Shape> anEdges = new();
        Explorer exp2 = new(aSectionTool.Shape(), ShapeType.Edge);
        for (; exp2.More(); exp2.Next())
        {
            anEdges.Append(exp2.Current());
        }

        // if no edges are found, this means that this section yields no result
        if (anEdges.Length() <= 0)
        {
            return;
        }

        // connect edges to wires using ShapeAnalysis functionality
        mySection = ShapeAnalysis_FreeBounds.ConnectEdgesToWires(anEdges, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        myIsDone = (mySection is not null && mySection.Length() > 0);

        // collect all resulting wires to compound
        if (myIsDone)
        {
            BrepBuilder B = new();
            Compound aShapeC = new();
            B.MakeCompound(ref aShapeC);
            for (int i = 1; i <= mySection!.Length(); i++)
            {
                B.Add(aShapeC, mySection.Value(i));
            }
            aShapeC.TShape()!.FreezeGraph(); // OCCT.NET: handed out as is
            myShape = aShapeC;

            // initialize iteration (for compatibility with previous versions)
            myItr = 1;
        }
    }

    //=================================================================================================

    /// <summary>
    /// The projection of <paramref name="Wire"/> (a wire or an edge) onto <paramref name="Shape"/> along the direction
    /// <paramref name="D"/>.
    /// </summary>
    /// <exception cref="NullObjectException">A null input shape.</exception>
    /// <exception cref="ConstructionErrorException">The projected shape is neither a wire nor an edge, or the target
    /// has no faces.</exception>
    public BRepProj_Projection(in Shape Wire, in Shape Shape, in Direction3D D)
    {
        myIsDone = false;
        myItr = 0;
        // Check the input
        Standard_NullObject_Raise_if((Wire.IsNull || Shape.IsNull), "BRepProj_Projection.cxx: null input shape");
        if (Wire.ShapeType() != ShapeType.Edge && Wire.ShapeType() != global::Kerf.Topology.ShapeType.Wire)
        {
            throw new ConstructionErrorException("BRepProj_Projection.cxx: projected shape is neither wire nor edge");
        }

        // compute the "length" of the cylindrical surface to build
        double mdis = DistanceIn(Wire, Shape);
        Vector3D Vsup = new(D.XYZ() * 2 * mdis);
        Vector3D Vinf = new(D.XYZ() * -mdis);

        // move the base of the cylindrical surface by translating it by -mdis
        Transform3D T = new();
        T.SetTranslation(Vinf);
        // Note: it is necessary to create copy of wire to avoid adding new pcurves into it
        TrsfModification Trsf = new(T);
        Modifier Modif = new(Wire, Trsf);
        Shape WireBase = Modif.InternalModifiedShape(Wire);

        // Creation of a cylindrical surface
        BRepSweep_Prism CylSurf = new(WireBase, Vsup, false);

        // Perform section
        BuildSection(Shape, CylSurf.Shape());
    }

    //=================================================================================================

    /// <summary>
    /// The projection of <paramref name="Wire"/> (a wire or an edge) onto <paramref name="Shape"/> from the point
    /// <paramref name="P"/>.
    /// </summary>
    /// <exception cref="NullObjectException">A null input shape.</exception>
    /// <exception cref="ConstructionErrorException">The projected shape is neither a wire nor an edge, the point is the
    /// wire's first vertex, or the target has no faces.</exception>
    public BRepProj_Projection(in Shape Wire, in Shape Shape, in Point3D P)
    {
        myIsDone = false;
        myItr = 0;
        // Check the input
        Standard_NullObject_Raise_if((Wire.IsNull || Shape.IsNull), "BRepProj_Projection.cxx: null input shape");
        if (Wire.ShapeType() != ShapeType.Edge && Wire.ShapeType() != global::Kerf.Topology.ShapeType.Wire)
        {
            throw new ConstructionErrorException("BRepProj_Projection.cxx: projected shape is neither wire nor edge");
        }

        // if Wire is only an edge, transform it into a Wire
        Wire aWire = new();
        if (Wire.ShapeType() == ShapeType.Edge)
        {
            BrepBuilder BB = new();
            BB.MakeWire(ref aWire);
            BB.Add(aWire, Wire);
        }
        else
        {
            aWire = ShapeCasts.Wire(Wire);
        }

        // compute the "length" of the conical surface to build
        double mdis = DistanceIn(Wire, Shape);

        // Initialize iterator to get first sub-shape of Wire
        Explorer ExpWire = new();
        ExpWire.Init(aWire, ShapeType.Vertex);

        // get the first Point of the first sub-shape os the Wire
        Point3D PC = Tool.Pnt(ShapeCasts.Vertex(ExpWire.Current()));

        // compute the ratio of the scale transformation
        double Scale = PC.Distance(P);
        if (Math.Abs(Scale) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            throw new ConstructionErrorException("Projection");
        }
        Scale = 1.0 + mdis / Scale;

        // move the base of the conical surface by scaling it with ratio Scale
        Transform3D T = new();
        T.SetScale(P, Scale);
        TrsfModification Tsca = new(T);
        Modifier ModifScale = new(aWire, Tsca);
        Shape ShapeGen1 = ModifScale.InternalModifiedShape(aWire);

        Vertex aVertex = new BRepLib_MakeVertex(P).Vertex();
        Edge DegEdge = new();
        BrepBuilder BB2 = new();
        BB2.MakeEdge(ref DegEdge);
        BB2.Add(DegEdge, aVertex.Oriented(Forward));
        BB2.Add(DegEdge, aVertex.Oriented(Reversed));
        BB2.Degenerated(DegEdge, true);

        Wire DegWire = new();
        BB2.MakeWire(ref DegWire);
        BB2.Add(DegWire, DegEdge);
        DegWire.Closed(true);

        // Build the Ruled surface based shape
        BRepFill_Generator RuledSurf = new();
        RuledSurf.AddWire(DegWire);
        RuledSurf.AddWire(ShapeCasts.Wire(ShapeGen1));
        RuledSurf.Perform();
        Shell SurfShell = RuledSurf.InternalShell();

        // Perform section
        BuildSection(Shape, SurfShell);
    }

    //=================================================================================================

    /// <summary>True if the projection gave at least one wire.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>Starts the iteration over the wires of the projection.</summary>
    public void Init()
    {
        myItr = 1;
    }

    /// <summary>True while there is a current wire.</summary>
    public bool More() => myItr > 0 && mySection is not null && myItr <= mySection.Length();

    /// <summary>Moves to the next wire.</summary>
    public void Next()
    {
        myItr++;
    }

    /// <summary>The current wire (frozen).</summary>
    public Wire Current() => ShapeCasts.Wire(mySection!.Value(myItr));

    /// <summary>The wires of the projection as a compound (frozen; null when not done).</summary>
    public Compound Shape() => myShape;
}
