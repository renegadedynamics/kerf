// Port of: src/ModelingAlgorithms/TKFillet/BRepFilletAPI/BRepFilletAPI_MakeFillet.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepFilletAPI/BRepFilletAPI_MakeFillet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using static Kerf.Engine.Fillets.FilletShape;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Fillets.Api;

/// <summary>
/// Builds fillets on the broken edges of a shell or solid: initialized with a shape, given contours and radii
/// (constant, linear between two values, by a law or interpolating parameter-radius pairs), built, then consulted.
/// </summary>
/// <remarks>
/// <para>
/// A maker (decisions 21, 414) over a <see cref="FilBuilder"/>. The caller's shape is never written: the
/// builder copies a frozen shape at the entry and gives back the caller's sub-shapes where the fillets left them
/// unchanged (decision 1136). <see cref="MakeShape.Shape"/>, <see cref="MakeShape.Generated"/>,
/// <see cref="MakeShape.Modified"/>, <see cref="NewFaces"/> and <see cref="BadShape"/> hand out frozen
/// shapes. The history answers for the caller's edges, vertices and faces.
/// </para>
/// <para>
/// The faces come from unit 7R's reconstruction on the Boolean engine (decision 992), not from OCCT's
/// TopOpeBRepBuild_HBuilder: the result is native's geometrically, its faces in another order.
/// </para>
/// </remarks>
public class MakeFillet : LocalOperation
{
    private readonly FilBuilder _builder;
    private readonly NCollection_Map<Shape> _map = new(ShapeHasher.Instance);

    //=================================================================================================

    /// <summary>
    /// Initializes the computation of the fillets on <paramref name="s"/>. <paramref name="fShape"/> sets the type of
    /// fillet surface: ChFi3d_Rational (classical NURBS representation of circles, the default),
    /// ChFi3d_QuasiAngular (a NURBS representation whose parametrization matches the circle's) or ChFi3d_Polynomial
    /// (a polynomial representation of circles).
    /// </summary>
    public MakeFillet(in Shape s, FilletShape fShape = FilletShape.Rational)
    {
        _builder = new FilBuilder(s, fShape);
    }

    //=================================================================================================

    /// <summary>Sets the tolerances of the computation (OCCT's defaults: 1e-2, 1e-4, 1e-5, 1e-4, 1e-5, 1e-3).</summary>
    public void SetParams(double tang, double tesp, double t2d, double tApp3d, double tolApp2d, double fleche)
    {
        _builder.SetParams(tang, tesp, t2d, tApp3d, tolApp2d, fleche);
    }

    //=================================================================================================

    /// <summary>
    /// Changes the continuity of the fillet surfaces (C0, C1 or C2; C1 by default) and the G1 tolerance between the
    /// fillets and their support faces.
    /// </summary>
    public void SetContinuity(Continuity internalContinuity, double angularTolerance)
    {
        _builder.SetContinuity(internalContinuity, angularTolerance);
    }

    //=================================================================================================

    /// <summary>Adds a fillet contour (a contour of tangent edges); the radius must be set after.</summary>
    public override void Add(in Edge e)
    {
        _builder.Add(e);
    }

    //=================================================================================================

    /// <summary>Adds a fillet contour with the radius <paramref name="radius"/>.</summary>
    public void Add(double radius, in Edge e)
    {
        // myBuilder.Add(Radius,E);
        _builder.Add(e);
        int ic = _builder.Contains(e, out int iinC);
        if (ic != 0)
        {
            SetRadius(radius, ic, iinC);
        }
    }

    //=================================================================================================

    /// <summary>
    /// Adds a fillet contour whose radius evolves linearly from <paramref name="r1"/> to <paramref name="r2"/>
    /// between the first and last vertices of the spine.
    /// </summary>
    public void Add(double r1, double r2, in Edge e)
    {
        _builder.Add(e);
        int ic = _builder.Contains(e, out int iinC);
        if (ic != 0)
        {
            SetRadius(r1, r2, ic, iinC);
        }
    }

    //=================================================================================================

    /// <summary>Adds a fillet contour whose radius evolves by the law <paramref name="l"/>.</summary>
    public void Add(LawsFunction l, in Edge e)
    {
        // myBuilder.Add(L,E);
        _builder.Add(e);
        int ic = _builder.Contains(e, out int iinC);
        if (ic != 0)
        {
            SetRadius(l, ic, iinC);
        }
    }

    //=================================================================================================

    /// <summary>
    /// Adds a fillet contour whose radius interpolates the pairs <paramref name="uandR"/>: X a relative parameter on
    /// the spine in [0, 1], Y the radius there.
    /// </summary>
    public void Add(NCollection_Array1<Point2D> uandR, in Edge e)
    {
        _builder.Add(e);
        int ic = _builder.Contains(e, out int iinC);
        if (ic != 0)
        {
            SetRadius(uandR, ic, iinC);
        }
    }

    //=================================================================================================

    /// <summary>Sets the radius <paramref name="radius"/> along the contour <paramref name="ic"/>.</summary>
    public void SetRadius(double radius, int ic, int iinC)
    {
        Xy firstUandR = new(0.0, radius), lastUandR = new(1.0, radius);
        _builder.SetRadius(firstUandR, ic, iinC);
        _builder.SetRadius(lastUandR, ic, iinC);
    }

    //=================================================================================================

    /// <summary>
    /// Sets a radius evolving linearly from <paramref name="R1"/> to <paramref name="R2"/> between the first and last
    /// vertices of the contour <paramref name="ic"/>.
    /// </summary>
    public void SetRadius(double R1, double R2, int ic, int iinC)
    {
        double r1, r2;

        if (Math.Abs(R1 - R2) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            r1 = r2 = (R1 + R2) * 0.5;
        }
        else
        {
            r1 = R1;
            r2 = R2;
        }
        Xy firstUandR = new(0.0, r1), lastUandR = new(1.0, r2);
        _builder.SetRadius(firstUandR, ic, iinC);
        _builder.SetRadius(lastUandR, ic, iinC);
    }

    //=================================================================================================

    /// <summary>Sets a radius evolving by the law <paramref name="l"/> along the contour <paramref name="ic"/>.</summary>
    public void SetRadius(LawsFunction l, int ic, int iinC)
    {
        _builder.SetRadius(l, ic, iinC);
    }

    //=================================================================================================

    /// <summary>
    /// Sets a radius interpolating the pairs <paramref name="uandR"/> (X a relative parameter on the contour, Y the
    /// radius) along the contour <paramref name="ic"/>.
    /// </summary>
    public void SetRadius(NCollection_Array1<Point2D> uandR, int ic, int iinC)
    {
        if (uandR.Length() == 1)
        {
            SetRadius(uandR[uandR.Lower()].Y(), ic, iinC);
        }
        else if (uandR.Length() == 2)
        {
            SetRadius(uandR[uandR.Lower()].Y(), uandR[uandR.Upper()].Y(), ic, iinC);
        }
        else
        {
            double uf = uandR[uandR.Lower()].X();
            double ul = uandR[uandR.Upper()].X();
            for (int i = uandR.Lower(); i <= uandR.Upper(); i++)
            {
                double ucur = uandR[i].X();
                ucur = (ucur - uf) / (ul - uf);
                Xy newUandR = new(ucur, uandR[i].Y());
                _builder.SetRadius(newUandR, ic, iinC);
            }
        }
    }

    //=================================================================================================

    /// <summary>Erases the radius of the contour <paramref name="ic"/> (nothing is done if IC is out of bounds).</summary>
    public override void ResetContour(int ic)
    {
        _builder.ResetContour(ic);
    }

    //=================================================================================================

    /// <summary>True if the radius along the contour <paramref name="ic"/> is constant (false if IC is out of bounds).</summary>
    public bool IsConstant(int ic)
    {
        return _builder.IsConstant(ic);
    }

    //=================================================================================================

    /// <summary>The constant radius of the contour <paramref name="ic"/> (-1 if IC is out of bounds).</summary>
    public double Radius(int ic)
    {
        return _builder.Radius(ic);
    }

    //=================================================================================================

    /// <summary>True if the radius along the edge <paramref name="e"/> of the contour <paramref name="ic"/> is constant.</summary>
    public bool IsConstant(int ic, in Edge e)
    {
        return _builder.IsConstant(ic, e);
    }

    //=================================================================================================

    /// <summary>The constant radius along the edge <paramref name="e"/> of the contour <paramref name="ic"/> (-1 if none).</summary>
    public double Radius(int ic, in Edge e)
    {
        return _builder.Radius(ic, e);
    }

    //=================================================================================================

    /// <summary>Assigns the radius <paramref name="radius"/> to the edge <paramref name="e"/> of the contour <paramref name="ic"/>.</summary>
    public void SetRadius(double radius, int ic, in Edge e)
    {
        _builder.SetRadius(radius, ic, e);
    }

    //=================================================================================================

    /// <summary>The bounds of the law on the edge <paramref name="e"/> of the contour <paramref name="ic"/>; false if it has none.</summary>
    public bool GetBounds(int ic, in Edge e, ref double f, ref double l)
    {
        return _builder.GetBounds(ic, e, ref f, ref l);
    }

    //=================================================================================================

    /// <summary>The radius law on the edge <paramref name="e"/> of the contour <paramref name="ic"/> (null if none).</summary>
    public LawsFunction? GetLaw(int ic, in Edge e)
    {
        return _builder.GetLaw(ic, e);
    }

    //=================================================================================================

    /// <summary>Sets the radius law <paramref name="l"/> on the edge <paramref name="e"/> of the contour <paramref name="ic"/>.</summary>
    public void SetLaw(int ic, in Edge e, LawsFunction l)
    {
        _builder.SetLaw(ic, e, l);
    }

    //=================================================================================================

    /// <summary>Assigns the radius <paramref name="radius"/> at the vertex <paramref name="v"/> of the contour <paramref name="ic"/>.</summary>
    public void SetRadius(double radius, int ic, in Vertex v)
    {
        _builder.SetRadius(radius, ic, v);
    }

    //=================================================================================================

    /// <summary>Assigns the type of fillet surface.</summary>
    public void SetFilletShape(FilletShape fShape)
    {
        _builder.SetFilletShape(fShape);
    }

    //=================================================================================================

    /// <summary>The type of fillet surface.</summary>
    public FilletShape GetFilletShape()
    {
        return _builder.GetFilletShape();
    }

    //=================================================================================================

    public override int ContourCount
    {
        get
        {
            return _builder.NbElements();
        }
    }

    //=================================================================================================

    public override int Contour(in Edge e)
    {
        return _builder.Contains(e);
    }

    //=================================================================================================

    public override int NbEdges(int i)
    {
        Spine spine = _builder.Value(i)!;
        int n = spine.EdgeCount;
        return n;
    }

    //=================================================================================================

    /// <summary>The edge <paramref name="j"/> of the contour <paramref name="i"/>: the caller's edge.</summary>
    public override Edge Edge(int i, int j)
    {
        Spine spine = _builder.Value(i)!;
        Edge s = spine.Edges(j);
        return ShapeCasts.Edge(_builder.FromWork(s)); // the caller's (OCCT.NET)
    }

    //=================================================================================================

    public override void Remove(in Edge e)
    {
        _builder.Remove(e);
    }

    //=================================================================================================

    public override double Length(int ic)
    {
        return _builder.Length(ic);
    }

    //=================================================================================================

    public override Vertex FirstVertex(int ic)
    {
        return _builder.FirstVertex(ic);
    }

    //=================================================================================================

    public override Vertex LastVertex(int ic)
    {
        return _builder.LastVertex(ic);
    }

    //=================================================================================================

    public override double Abscissa(int ic, in Vertex v)
    {
        return _builder.Abscissa(ic, v);
    }

    //=================================================================================================

    public override double RelativeAbscissa(int ic, in Vertex v)
    {
        return _builder.RelativeAbscissa(ic, v);
    }

    //=================================================================================================

    public override bool ClosedAndTangent(int ic)
    {
        return _builder.ClosedAndTangent(ic);
    }

    //=================================================================================================

    public override bool Closed(int ic)
    {
        return _builder.Closed(ic);
    }

    //=================================================================================================

    /// <summary>
    /// OCCT's <c>TopOpeBRepBuild_HBuilder</c>, here the reconstruction's seam (decision 1137): internal, its type
    /// being OCCT.NET's. <see cref="SurfaceCount"/> and <see cref="NewFaces"/> give what OCCT's callers read from it.
    /// </summary>
    internal Reconstruction Builder()
    {
        return _builder.Builder();
    }

    /// <summary>The fillet builder (OCCT.NET, for ported callers and tests).</summary>
    internal FilBuilder InternalBuilder() => _builder;

    //=================================================================================================

    /// <summary>Builds the fillets of all the contours; the result is <see cref="MakeShape.Shape"/>.</summary>
    public override void Build(ProgressRange range = default)
    {
        _builder.Compute();
        if (_builder.IsDone)
        {
            Done();
            _shape = _builder.InternalShape();

            // creation of the Map.
            Explorer ex = new();
            for (ex.Init(_shape, ShapeType.Face); ex.More(); ex.Next())
            {
                _map.Add(ex.Current());
            }
        }
    }

    //=================================================================================================

    /// <summary>Resets the fields updated by Build, not the contours.</summary>
    public override void Reset()
    {
        NotDone();
        _builder.Reset();
        _map.Clear();
    }

    //=================================================================================================

    /// <summary>The number of surfaces built (the data structure's).</summary>
    public int SurfaceCount
    {
        get
        {
            return _builder.Builder().DataStructure()!.SurfaceCount;
        }
    }

    //=================================================================================================

    /// <summary>The faces built on the surface <paramref name="i"/> (frozen).</summary>
    public ISchematiqList<Shape> NewFaces(int i)
    {
        BeforePublishingLists();
        return PublishedList(_builder.Builder().NewFaces(i));
    }

    //=================================================================================================

    public override void Simulate(int ic)
    {
        _builder.Simulate(ic);
    }

    //=================================================================================================

    public override int NbSurf(int ic)
    {
        return _builder.NbSurf(ic);
    }

    //=================================================================================================

    public override NCollection_HArray1<CircSection>? Sect(int ic, int IS)
    {
        return _builder.Sect(ic, IS);
    }

    //=================================================================================================

    // The lists hold faces of the result: freeze it with them (decision 414).
    private protected override void BeforePublishingLists()
    {
        if (IsDone)
        {
            _shape.TShape()?.FreezeGraph();
        }
    }

    // Generated(EorV): the faces of the fillets on the edge or vertex EorV.
    internal override NCollection_List<Shape> InternalGenerated(in Shape eorV)
    {
        return _builder.InternalGenerated(eorV);
    }

    //=================================================================================================

    // Modified(F): the pieces of the face F (the union of the states OUT, IN and ON, as OCCT; the reconstruction
    // builds IN only, decision 1110).
    internal override NCollection_List<Shape> InternalModified(in Shape f)
    {
        Shape F = _builder.ToWork(f); // the working copy's (OCCT.NET)
        _generated.Clear();

        if (_builder.Builder().IsSplit(F, Out))
        {
            NCollection_List<Shape>.Iterator it = new(_builder.Builder().Splits(F, Out));
            for (; it.More(); it.Next())
            {
                _generated.Append(it.Value());
            }
        }
        if (_builder.Builder().IsSplit(F, In))
        {
            NCollection_List<Shape>.Iterator it = new(_builder.Builder().Splits(F, In));
            for (; it.More(); it.Next())
            {
                _generated.Append(it.Value());
            }
        }
        if (_builder.Builder().IsSplit(F, On))
        {
            NCollection_List<Shape>.Iterator it = new(_builder.Builder().Splits(F, On));
            for (; it.More(); it.Next())
            {
                _generated.Append(it.Value());
            }
        }
        return _generated;
    }

    //=================================================================================================

    /// <summary>True if the face <paramref name="f"/> is neither in the result nor split.</summary>
    public override bool IsDeleted(in Shape f)
    {
        Shape work = _builder.ToWork(f); // the working copy's (OCCT.NET)
        return !(_map.Contains(f) || _map.Contains(work) || _builder.Builder().IsSplit(work, Out)
                 || _builder.Builder().IsSplit(work, In)
                 || _builder.Builder().IsSplit(work, On));
    }

    //=================================================================================================

    /// <summary>The number of contours that failed.</summary>
    public int FaultyContourCount
    {
        get
        {
            return _builder.FaultyContourCount;
        }
    }

    //=================================================================================================

    /// <summary>The index of the <paramref name="i"/>-th contour that failed.</summary>
    public int FaultyContour(int i)
    {
        return _builder.FaultyContour(i);
    }

    //=================================================================================================

    /// <summary>The number of surfaces computed on the contour <paramref name="ic"/>.</summary>
    public int NbComputedSurfaces(int ic)
    {
        return _builder.NbComputedSurfaces(ic);
    }

    //=================================================================================================

    /// <summary>The surface <paramref name="IS"/> computed on the contour <paramref name="ic"/>.</summary>
    public Surface? ComputedSurface(int ic, int IS)
    {
        return _builder.ComputedSurface(ic, IS);
    }

    //=================================================================================================

    /// <summary>The number of vertices where the corner failed.</summary>
    public int FaultyVertexCount
    {
        get
        {
            return _builder.FaultyVertexCount;
        }
    }

    //=================================================================================================

    /// <summary>The <paramref name="iv"/>-th vertex where the corner failed (the caller's).</summary>
    public Vertex FaultyVertex(int iv)
    {
        return _builder.FaultyVertex(iv);
    }

    //=================================================================================================

    /// <summary>True if a partial result was built (<see cref="BadShape"/>) though a corner failed.</summary>
    public bool HasResult
    {
        get
        {
            return _builder.HasResult;
        }
    }

    //=================================================================================================

    /// <summary>The partial result (frozen).</summary>
    /// <exception cref="NoSuchObjectException">There is none (<see cref="HasResult"/> is false).</exception>
    public Shape BadShape()
    {
        return _builder.BadShape();
    }

    //=================================================================================================

    /// <summary>The status of the contour <paramref name="ic"/>.</summary>
    public ErrorStatus StripeStatus(int ic)
    {
        return _builder.StripeStatus(ic);
    }
}
