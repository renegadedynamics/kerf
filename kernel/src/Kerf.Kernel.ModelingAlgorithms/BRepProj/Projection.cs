// Port of: src/ModelingAlgorithms/TKBool/BRepProj/BRepProj_Projection.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepProj/BRepProj_Projection.lxx
// Port of: src/ModelingAlgorithms/TKBool/BRepProj/BRepProj_Projection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Sweeping;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Api;

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
public sealed class Projection
{
    private bool _isDone;
    // (OCCT's myLsh, never used, is left out)
    private Compound _shape = new();
    private NCollection_HSequence<Shape>? _section;
    private int _itr;

    //=======================================================================
    // function : DistanceOut
    // purpose  : Compute the minimum distance between input shapes
    //           (using Bounding Boxes of each Shape)
    //=======================================================================
    private static double DistanceOut(in Shape s1, in Shape s2)
    {
        Box bBox1 = new(), bBox2 = new();
        Bounds.Add(s1, ref bBox1);
        Bounds.Add(s2, ref bBox2);
        return bBox1.Distance(bBox2);
    }

    //=======================================================================
    // function : DistanceIn
    // purpose  : Compute the maximum distance between input Shapes
    //           we compute the maximum dimension of each Bounding Box and then
    //           add each other with the minimum distance of shapes.
    //=======================================================================

    private static double DistanceIn(in Shape s1, in Shape s2)
    {
        Box lbBox = new(), sbBox = new();
        Bounds.Add(s1, ref sbBox);
        Bounds.Add(s2, ref lbBox);

        sbBox.Get(out double sXmin, out double sYmin, out double sZmin, out double sXmax, out double sYmax, out double sZmax);
        lbBox.Get(out double lXmin, out double lYmin, out double lZmin, out double lXmax, out double lYmax, out double lZmax);

        // Compute the max distance between input shapes------------//
        Xyz lmin = new(lXmin, lYmin, lZmin), lmax = new(lXmax, lYmax, lZmax);
        Xyz smin = new(sXmin, sYmin, sZmin), smax = new(sXmax, sYmax, sZmax);
        lmax.Subtract(lmin);
        smax.Subtract(smin);
        return lmax.Modulus() + smax.Modulus() + DistanceOut(s1, s2);
    }

    //=======================================================================
    // function : BuildSection
    // purpose  : Cuts theShape by theTool using BRepAlgoAPI_Section and
    //           stores result as set of connected wires and compound
    //=======================================================================

    private void BuildSection(in Shape shape, in Shape tool)
    {
        _isDone = false;
        _section = null;
        _shape = new Compound();
        _itr = 0;

        // if theShape is compound, extract only faces -- section algorithm
        // may refuse to work if e.g. vertex is present
        Shape aShape = new();
        if (shape.Type == ShapeType.Face || shape.Type == ShapeType.Shell
            || shape.Type == ShapeType.Solid || shape.Type == ShapeType.CompSolid)
        {
            aShape = shape;
        }
        else if (shape.Type == ShapeType.Compound)
        {
            Compound c = new();
            BrepBuilder b = new();
            Explorer exp = new(shape, ShapeType.Face);
            for (; exp.More(); exp.Next())
            {
                if (c.IsNull)
                {
                    b.MakeCompound(ref c);
                }
                b.Add(c, exp.Current());
            }
            aShape = c;
        }
        if (aShape.IsNull)
        {
            throw new ConstructionErrorException("BRepProj_Projection.cxx: target shape has no faces");
        }

        // build section computing p-curves on both shapes to get higher precision
        SectionOperation sectionTool = new(aShape, tool, false);
        sectionTool.Approximation(true);
        sectionTool.ComputePCurveOn1(true);
        sectionTool.ComputePCurveOn2(true);
        // Use Oriented Bounding Boxes inside Booleans to speed up calculation of the section
        sectionTool.SetUseOBB(true);
        // Kerf: the faces of a compound target are gathered in a new compound, which isn't locked; the pave filler
        // finds the caller's frozen faces below it and sections non-destructively, as it does a frozen solid or face
        // (decisions 939, 1378). Inside an operation's overlay the section is destructive, as OCCT's.
        sectionTool.Build();

        // check for successful work of the section tool
        if (!sectionTool.IsDone)
        {
            return;
        }

        // get edges of the result
        NCollection_HSequence<Shape> edges = new();
        Explorer exp2 = new(sectionTool.Shape(), ShapeType.Edge);
        for (; exp2.More(); exp2.Next())
        {
            edges.Append(exp2.Current());
        }

        // if no edges are found, this means that this section yields no result
        if (edges.Length() <= 0)
        {
            return;
        }

        // connect edges to wires using ShapeAnalysis functionality
        _section = FreeBounds.ConnectEdgesToWires(edges, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        _isDone = (_section is not null && _section.Length() > 0);

        // collect all resulting wires to compound
        if (_isDone)
        {
            BrepBuilder b = new();
            Compound shapeC = new();
            b.MakeCompound(ref shapeC);
            for (int i = 1; i <= _section!.Length(); i++)
            {
                b.Add(shapeC, _section.Value(i));
            }
            shapeC.TShape()!.FreezeGraph(); // OCCT.NET: handed out as is
            _shape = shapeC;

            // initialize iteration (for compatibility with previous versions)
            _itr = 1;
        }
    }

    //=================================================================================================

    /// <summary>
    /// The projection of <paramref name="wire"/> (a wire or an edge) onto <paramref name="shape"/> along the direction
    /// <paramref name="d"/>.
    /// </summary>
    /// <exception cref="NullObjectException">A null input shape.</exception>
    /// <exception cref="ConstructionErrorException">The projected shape is neither a wire nor an edge, or the target
    /// has no faces.</exception>
    public Projection(in Shape wire, in Shape shape, in Direction3D d)
    {
        _isDone = false;
        _itr = 0;
        // Check the input
        Standard_NullObject_Raise_if((wire.IsNull || shape.IsNull), "BRepProj_Projection.cxx: null input shape");
        if (wire.Type != ShapeType.Edge && wire.Type != global::Kerf.Topology.ShapeType.Wire)
        {
            throw new ConstructionErrorException("BRepProj_Projection.cxx: projected shape is neither wire nor edge");
        }

        // compute the "length" of the cylindrical surface to build
        double mdis = DistanceIn(wire, shape);
        Vector3D vsup = new(d.XYZ() * 2 * mdis);
        Vector3D vinf = new(d.XYZ() * -mdis);

        // move the base of the cylindrical surface by translating it by -mdis
        Transform3D t = new Transform3D().WithTranslation(vinf);
        // Note: it is necessary to create copy of wire to avoid adding new pcurves into it
        TrsfModification trsf = new(t);
        Modifier modif = new(wire, trsf);
        Shape wireBase = modif.InternalModifiedShape(wire);

        // Creation of a cylindrical surface
        Prism cylSurf = new(wireBase, vsup, false);

        // Perform section
        BuildSection(shape, cylSurf.Shape());
    }

    //=================================================================================================

    /// <summary>
    /// The projection of <paramref name="wire"/> (a wire or an edge) onto <paramref name="shape"/> from the point
    /// <paramref name="p"/>.
    /// </summary>
    /// <exception cref="NullObjectException">A null input shape.</exception>
    /// <exception cref="ConstructionErrorException">The projected shape is neither a wire nor an edge, the point is the
    /// wire's first vertex, or the target has no faces.</exception>
    public Projection(in Shape wire, in Shape shape, in Point3D p)
    {
        _isDone = false;
        _itr = 0;
        // Check the input
        Standard_NullObject_Raise_if((wire.IsNull || shape.IsNull), "BRepProj_Projection.cxx: null input shape");
        if (wire.Type != ShapeType.Edge && wire.Type != global::Kerf.Topology.ShapeType.Wire)
        {
            throw new ConstructionErrorException("BRepProj_Projection.cxx: projected shape is neither wire nor edge");
        }

        // if Wire is only an edge, transform it into a Wire
        Wire aWire = new();
        if (wire.Type == ShapeType.Edge)
        {
            BrepBuilder bb = new();
            bb.MakeWire(ref aWire);
            bb.Add(aWire, wire);
        }
        else
        {
            aWire = ShapeCasts.Wire(wire);
        }

        // compute the "length" of the conical surface to build
        double mdis = DistanceIn(wire, shape);

        // Initialize iterator to get first sub-shape of Wire
        Explorer expWire = new();
        expWire.Init(aWire, ShapeType.Vertex);

        // get the first Point of the first sub-shape os the Wire
        Point3D pc = Tool.Pnt(ShapeCasts.Vertex(expWire.Current()));

        // compute the ratio of the scale transformation
        double scale = pc.DistanceTo(p);
        if (Math.Abs(scale) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            throw new ConstructionErrorException("Projection");
        }
        scale = 1.0 + mdis / scale;

        // move the base of the conical surface by scaling it with ratio Scale
        Transform3D t = new Transform3D().WithScale(p, scale);
        TrsfModification tsca = new(t);
        Modifier modifScale = new(aWire, tsca);
        Shape shapeGen1 = modifScale.InternalModifiedShape(aWire);

        Vertex vertex = new LibraryMakeVertex(p).Vertex();
        Edge degEdge = new();
        BrepBuilder bb2 = new();
        bb2.MakeEdge(ref degEdge);
        bb2.Add(degEdge, vertex.Oriented(Forward));
        bb2.Add(degEdge, vertex.Oriented(Reversed));
        bb2.Degenerated(degEdge, true);

        Wire degWire = new();
        bb2.MakeWire(ref degWire);
        bb2.Add(degWire, degEdge);
        degWire.Closed(true);

        // Build the Ruled surface based shape
        Generator ruledSurf = new();
        ruledSurf.AddWire(degWire);
        ruledSurf.AddWire(ShapeCasts.Wire(shapeGen1));
        ruledSurf.Perform();
        Shell surfShell = ruledSurf.InternalShell();

        // Perform section
        BuildSection(shape, surfShell);
    }

    //=================================================================================================

    /// <summary>True if the projection gave at least one wire.</summary>
    public bool IsDone => _isDone;

    /// <summary>Starts the iteration over the wires of the projection.</summary>
    public void Init()
    {
        _itr = 1;
    }

    /// <summary>True while there is a current wire.</summary>
    public bool More() => _itr > 0 && _section is not null && _itr <= _section.Length();

    /// <summary>Moves to the next wire.</summary>
    public void Next()
    {
        _itr++;
    }

    /// <summary>The current wire (frozen).</summary>
    public Wire Current() => ShapeCasts.Wire(_section!.Value(_itr));

    /// <summary>The wires of the projection as a compound (frozen; null when not done).</summary>
    public Compound Shape() => _shape;
}
