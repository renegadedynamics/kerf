// Port of: src/ModelingAlgorithms/TKFillet/BRepFilletAPI/BRepFilletAPI_MakeChamfer.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepFilletAPI/BRepFilletAPI_MakeChamfer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.Data.ChamfMethod;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Fillets.Api;

/// <summary>
/// Builds chamfers on the edges of a shell or solid: by one distance (symmetric), two distances or a distance and an
/// angle, each measured on a face adjacent to the edge.
/// </summary>
/// <remarks>
/// A maker over a <see cref="ChBuilder"/>, as <see cref="MakeFillet"/>: the caller's shape is
/// never written, the result, the history and the lists hand out frozen shapes, the history answers for the caller's
/// sub-shapes, and the faces come from unit 7R's reconstruction (decision 992).
/// </remarks>
internal class MakeChamfer : LocalOperation
{
    private readonly ChBuilder _builder;
    private readonly HashSet<Shape> _map = new(ShapeHasher.Instance);

    //=================================================================================================

    /// <summary>Initializes an algorithm for computing chamfers on the shape <paramref name="s"/>.</summary>
    public MakeChamfer(in Shape s)
    {
        _builder = new ChBuilder(s);
    }

    //=================================================================================================

    /// <summary>Adds the edge <paramref name="e"/> (and the edges tangent to it) as a contour; the distances are set after.</summary>
    public override void Add(in Edge e)
    {
        _builder.Add(e);
    }

    //=================================================================================================

    /// <summary>Adds a symmetric chamfer contour of distance <paramref name="dis"/> on the edge <paramref name="e"/>.</summary>
    public void Add(double dis, in Edge e)
    {
        _builder.Add(dis, e);
    }

    //=================================================================================================

    /// <summary>Sets the distance of the symmetric chamfer of the contour <paramref name="ic"/>.</summary>
    public void SetDist(double dis, int ic, in Face f)
    {
        _builder.SetDist(dis, ic, f);
    }

    //=================================================================================================

    /// <summary>The distance of the symmetric chamfer of the contour <paramref name="ic"/>.</summary>
    public void GetDist(int ic, ref double dis)
    {
        _builder.GetDist(ic, ref dis);
    }

    //=================================================================================================

    /// <summary>
    /// Adds a chamfer contour by two distances on the edge <paramref name="e"/>: <paramref name="dis1"/> measured on the
    /// face <paramref name="f"/>, <paramref name="dis2"/> on the other.
    /// </summary>
    public void Add(double dis1, double dis2, in Edge e, in Face f)
    {
        _builder.Add(dis1, dis2, e, f);
    }

    //=================================================================================================

    /// <summary>Sets the two distances of the contour <paramref name="ic"/>, <paramref name="dis1"/> on the face <paramref name="f"/>.</summary>
    public void SetDists(double dis1, double dis2, int ic, in Face f)
    {
        _builder.SetDists(dis1, dis2, ic, f);
    }

    //=================================================================================================

    /// <summary>The two distances of the contour <paramref name="ic"/>.</summary>
    public void Dists(int ic, ref double dis1, ref double dis2)
    {
        double temp1 = 0.0, temp2 = 0.0;
        _builder.Dists(ic, ref temp1, ref temp2);
        dis1 = temp1;
        dis2 = temp2;
    }

    //=================================================================================================

    /// <summary>
    /// Adds a chamfer contour by a distance <paramref name="dis"/> on the face <paramref name="f"/> and an angle
    /// <paramref name="angle"/> (radians) on the edge <paramref name="e"/>.
    /// </summary>
    public void AddDA(double dis, double angle, in Edge e, in Face f)
    {
        _builder.AddDA(dis, angle, e, f);
    }

    //=================================================================================================

    /// <summary>Sets the distance and the angle of the contour <paramref name="ic"/>, the distance on the face <paramref name="f"/>.</summary>
    public void SetDistAngle(double dis, double angle, int ic, in Face f)
    {
        _builder.SetDistAngle(dis, angle, ic, f);
    }

    //=================================================================================================

    /// <summary>The distance and the angle of the contour <paramref name="ic"/>.</summary>
    public void GetDistAngle(int ic, ref double dis, ref double angle)
    {
        _builder.GetDistAngle(ic, ref dis, ref angle);
    }

    //=================================================================================================

    /// <summary>Sets the mode of the chamfer (classic, or a throat: ChFiDS_ChamfMode).</summary>
    public void SetMode(ChamfMode mode)
    {
        _builder.SetMode(mode);
    }

    //=================================================================================================

    /// <summary>True if the chamfer of the contour <paramref name="ic"/> is symmetric (one distance).</summary>
    public bool IsSymetric(int ic)
    {
        ChamfMethod chamfMeth = _builder.IsChamfer(ic);
        bool ret = false;

        if (chamfMeth == ChamfMethod.Sym)
        {
            ret = true;
        }

        return ret;
    }

    //=================================================================================================

    /// <summary>True if the chamfer of the contour <paramref name="ic"/> is by two distances.</summary>
    public bool IsTwoDistances(int ic)
    {
        ChamfMethod chamfMeth = _builder.IsChamfer(ic);
        bool ret = false;

        if (chamfMeth == ChamfMethod.TwoDist)
        {
            ret = true;
        }

        return ret;
    }

    //=================================================================================================

    /// <summary>True if the chamfer of the contour <paramref name="ic"/> is by a distance and an angle.</summary>
    public bool IsDistanceAngle(int ic)
    {
        ChamfMethod chamfMeth = _builder.IsChamfer(ic);
        bool ret = false;

        if (chamfMeth == ChamfMethod.DistAngle)
        {
            ret = true;
        }

        return ret;
    }

    //=================================================================================================

    public override void ResetContour(int ic)
    {
        _builder.ResetContour(ic);
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
        return ShapeCasts.Edge(s);
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
    /// being Kerf's.
    /// </summary>
    internal Reconstruction Builder()
    {
        return _builder.Builder();
    }

    /// <summary>The chamfer builder (Kerf, for ported callers and tests).</summary>
    internal ChBuilder InternalBuilder() => _builder;

    //=================================================================================================

    /// <summary>Builds the chamfers of all the contours; the result is <see cref="MakeShape.Shape"/>.</summary>
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

    // Generated(EorV): the faces of the chamfers on the edge or vertex EorV.
    internal override NCollection_List<Shape> InternalGenerated(in Shape eorV)
    {
        return _builder.InternalGenerated(eorV);
    }

    //=================================================================================================

    // Modified(F): the pieces of the face F (the union of the states OUT, IN and ON, as OCCT; the reconstruction
    // builds IN only, decision 1110).
    internal override NCollection_List<Shape> InternalModified(in Shape f)
    {
        Shape F = f;
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
        Shape work = f;
        return !(_map.Contains(f) || _map.Contains(work) || _builder.Builder().IsSplit(work, Out)
                 || _builder.Builder().IsSplit(work, In)
                 || _builder.Builder().IsSplit(work, On));
    }
}
