// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_LocationLaw.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_LocationLaw.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Construction.Surfaces.SurfacePipeError;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Construction.Filling;

/// <summary>A location law on a wire: one GeomFill location law per non-degenerated edge of the path.</summary>
/// <remarks>
/// A working object (decisions 21 and 806): evaluation caches the edges' lengths, <see cref="NbHoles"/> caches the
/// discontinuities, and the transformations change the elementary laws, which are the law's own copies of the
/// caller's law (one per edge, as in OCCT). <see cref="Law"/> returns them, not copies. The path and its edges are the
/// caller's. <see cref="D0"/> hands out a frozen transformed copy of the section; <see cref="PerformVertex"/> a frozen
/// new vertex, and <see cref="InternalPerformVertex"/> the same vertex unfrozen for the sweeps that keep building on it
/// (decision 900).
/// </remarks>
public abstract class LocationLaw : Transient
{
    protected Wire _path = new();
    protected double _tol;
    protected NCollection_HArray1<SurfaceLocationLaw>? _laws;
    protected NCollection_HArray1<double>? _length;
    protected NCollection_HArray1<Shape>? _edges;
    protected NCollection_HArray1<int>? _disc;
    // Set by TangentIsMain, NormalIsMain and BiNormalIsMain and never read, as in OCCT.
#pragma warning disable CS0414
    private int _type;
#pragma warning restore CS0414

    //=================================================================================================

    private static double Norm(in Matrix3x3 m)
    {
        double r, norme;
        Xyz coord;
        coord = m.Row(1);
        norme = Math.Abs(coord.X()) + Math.Abs(coord.Y()) + Math.Abs(coord.Z());
        coord = m.Row(2);
        r = Math.Abs(coord.X()) + Math.Abs(coord.Y()) + Math.Abs(coord.Z());
        if (r > norme)
        {
            norme = r;
        }
        coord = m.Row(3);
        r = Math.Abs(coord.X()) + Math.Abs(coord.Y()) + Math.Abs(coord.Z());
        if (r > norme)
        {
            norme = r;
        }

        return norme;
    }

    //=======================================================================
    // function : ToG0
    // purpose  : Calculate transformation T such as T.M2 = M1
    //=======================================================================

    private static void ToG0(in Matrix3x3 m1, in Matrix3x3 m2, out Matrix3x3 t)
    {
        t = m2.Inverted();
        t *= m1;
    }

    //=================================================================================================

    /// <summary>Initialises all the fields; called by the constructors of the derived classes.</summary>
    protected void Init(in Wire path)

    {
        int nbEdge;
        WireExplorer wexp = new();
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool B;

        _path = path;
        _tol = 1.0e-4;

        for (nbEdge = 0, wexp.Init(_path); wexp.More(); wexp.Next())
        {
            //    if (! B.Degenerated(wexp.Current())) NbEdge++;
            if (!Tool.Degenerated(wexp.Current()))
            {
                nbEdge++;
            }
        }

        _laws = new NCollection_HArray1<SurfaceLocationLaw>(1, nbEdge);
        _length = new NCollection_HArray1<double>(1, nbEdge + 1);
        _length.Init(-1.0);
        _length.SetValue(1, 0.0);
        _edges = new NCollection_HArray1<Shape>(1, nbEdge);
        _disc = null;
        TangentIsMain();
    }

    //=================================================================================================

    /// <summary>The first error status of the elementary laws other than GeomFill_PipeOk, else GeomFill_PipeOk.</summary>
    public SurfacePipeError GetStatus()
    {
        int ii, n = _laws!.Length();
        SurfacePipeError status = SurfacePipeError.PipeOk;
        for (ii = 1; ii <= n && (status == SurfacePipeError.PipeOk); ii++)
        {
            status = _laws.Value(ii).ErrorStatus();
        }
        return status;
    }

    //=================================================================================================

    /// <summary>To preserve if possible the tangent in transformations (the default mode).</summary>
    protected void TangentIsMain()
    {
        _type = 1;
    }

    //=================================================================================================

    /// <summary>To preserve if possible the normal in transformations.</summary>
    protected void NormalIsMain()
    {
        _type = 2;
    }

    //=================================================================================================

    /// <summary>To preserve if possible the binormal in transformations.</summary>
    protected void BiNormalIsMain()
    {
        _type = 3;
    }

    //=================================================================================================

    /// <summary>
    /// Applies a linear transformation to each elementary law to reduce the discontinuities of the law to one
    /// rotation.
    /// </summary>
    public virtual void TransformInCompatibleLaw(double tolAngular)
    {

        double angle;
        int ipath;
        Matrix3x3 trsf = new(), m1 = new(), m2 = new();
        Vector3D v = new(), t1 = new(), t2 = new(), n1 = new(), n2 = new();
        Xyz oz = new(0, 0, 1);

        _laws!.Value(1).GetDomain(out double first, out double last);

        for (ipath = 2; ipath <= _laws.Length(); ipath++)
        {
            _laws.Value(ipath - 1).D0(last, ref m1, ref v);
            _laws.Value(ipath).GetDomain(out first, out last);
            _laws.Value(ipath).D0(first, ref m2, ref v);
            t1 = t1.WithXYZ(m1.Column(3));
            t2 = t2.WithXYZ(m2.Column(3));
            n1 = n1.WithXYZ(m1.Column(1));
            n2 = n2.WithXYZ(m2.Column(1));
            if (t1.IsParallel(t2, tolAngular) && !t1.IsOpposite(t2, tolAngular))
            { // Correction G0
                ToG0(m1, m2, out trsf);
            }
            else
            {
                double alpha;
                Vector3D cross = t1;
                cross = cross.Crossed(t2);
                alpha = t2.AngleWithRef(t1, cross);
                Axis3D axe = new(Constants.Origin(), new Direction3D(cross.XYZ()));
                n2 = n2.Rotated(axe, alpha);

                angle = n2.AngleWithRef(n1, t1);
                trsf = trsf.WithRotation(oz, angle);
            }
            _laws.Value(ipath).SetTrsf(trsf);
        }
    }

    //=================================================================================================

    /// <summary>Applies a linear transformation to each elementary law so that the global law is continuous.</summary>
    public virtual void TransformInG0Law()
    {

        int ipath;
        Matrix3x3 m1 = new(), m2 = new(); //,Trsf
        Vector3D v = new();
        _laws!.Value(1).GetDomain(out double first, out double last);
        for (ipath = 2; ipath <= _laws.Length(); ipath++)
        {
            _laws.Value(ipath - 1).D0(last, ref m1, ref v);
            _laws.Value(ipath).GetDomain(out first, out last);
            _laws.Value(ipath).D0(first, ref m2, ref v);
            ToG0(m1, m2, out Matrix3x3 aux);
            _laws.Value(ipath).SetTrsf(aux);
        }

        // Is the law periodical ?
        if (_path.Closed())
        {
            _laws.Value(_laws.Length()).D0(last, ref m1, ref v);
            _laws.Value(1).GetDomain(out first, out last);
            _laws.Value(1).D0(first, ref m2, ref v);
        }
    }

    //=======================================================================
    // function : DeleteTransform
    // purpose  : Remove the setting in continuity of law.
    //=======================================================================

    /// <summary>Removes the transformations set by <see cref="TransformInG0Law"/> or TransformInCompatibleLaw.</summary>
    public void DeleteTransform()
    {
        Matrix3x3 id = new();
        id = id.WithIdentity();
        for (int ii = 1; ii <= _edges!.Length(); ii++)
        {
            _laws!.ChangeValue(ii).SetTrsf(id);
        }
        _disc = null;
    }

    //=======================================================================
    // function : NbHoles
    // purpose  : Find "Holes"
    //=======================================================================

    /// <summary>The number of vertices of the path where the law isn't continuous (G0) within <paramref name="tol"/>.</summary>
    public int NbHoles(double tol = 1.0e-7)
    {
        if (_disc is null)
        {
            NCollection_Sequence<int> seq = new();
            int ii, nbDisc;
            for (ii = 2, nbDisc = -1; ii <= _laws!.Length() + 1; ii++)
            {
                if (IsG1(ii - 1, tol, 1.0e-12) == -1)
                {
                    seq.Append(ii);
                }
            }
            nbDisc = seq.Length();
            if (nbDisc > 0)
            {
                _disc = new NCollection_HArray1<int>(1, nbDisc);
                for (ii = 1; ii <= nbDisc; ii++)
                {
                    _disc.SetValue(ii, seq[ii]);
                }
            }
        }
        if (_disc is null)
        {
            return 0;
        }
        return _disc.Length();
    }

    //=================================================================================================

    /// <summary>Fills <paramref name="disc"/> with the indices found by <see cref="NbHoles"/>.</summary>
    public void Holes(NCollection_Array1<int> disc)
    {
        if (_disc is not null)
        {
            for (int ii = 1; ii <= _disc.Length(); ii++)
            {
                disc[ii] = _disc.Value(ii);
            }
        }
    }

    //=================================================================================================

    /// <summary>The number of elementary laws.</summary>
    public int LawCount => _laws!.Length();

    //=================================================================================================

    /// <summary>The elementary law of rank <paramref name="index"/> (in [1, NbLaw()]): the law's own object.</summary>
    public SurfaceLocationLaw Law(int index) => _laws!.Value(index);

    //=================================================================================================

    /// <summary>The path.</summary>
    public Wire Wire() => _path;

    //=================================================================================================

    /// <summary>The edge of rank <paramref name="index"/> (in [1, NbLaw()]) of the path.</summary>
    public Edge Edge(int index) => ShapeCasts.Edge(_edges!.Value(index));

    //=================================================================================================

    /// <summary>The vertex of rank <paramref name="index"/> (in [1, NbLaw() + 1]) of the path.</summary>
    public Vertex Vertex(int index)
    {
        Edge e;
        Vertex v = new();
        if (index <= _edges!.Length())
        {
            e = ShapeCasts.Edge(_edges.Value(index));
            if (e.Orientation() == Reversed)
            {
                v = Exploration.LastVertex(e);
            }
            else
            {
                v = Exploration.FirstVertex(e);
            }
        }
        else if (index == _edges.Length() + 1)
        {
            e = ShapeCasts.Edge(_edges.Value(index - 1));
            if (e.Orientation() == Reversed)
            {
                v = Exploration.FirstVertex(e);
            }
            else
            {
                v = Exploration.LastVertex(e);
            }
        }
        return v;
    }

    //===================================================================
    // function : PerformVertex
    // purpose  : Calculate a vertex of sweeping from a vertex of section
    //           and the index of the edge in the trajectory
    //===================================================================

    /// <summary>
    /// <paramref name="input"/> placed by the law at the vertex of rank <paramref name="index"/> of the path, as a new
    /// frozen vertex. <paramref name="iLoc"/> chooses the law at a discontinuity: -1 the law before the vertex, 1 the
    /// law after it, 0 the average of both (the tolerance then covers half their gap).
    /// </summary>
    public void PerformVertex(int index, in Vertex input, double tolMin, out Vertex output, int iLoc = 0)
    {
        InternalPerformVertex(index, input, tolMin, out output, iLoc);
        output.TShape()?.FreezeGraph();
    }

    /// <summary><see cref="PerformVertex"/> without freezing the vertex, for the sweeps that build on it.</summary>
    internal void InternalPerformVertex(int index, in Vertex input, double tolMin, out Vertex output,
                                        int iLoc = 0)
    {
        output = new Vertex();
        BrepBuilder b = new();
        bool isBary = (iLoc == 0);
        double first, last;
        Point3D p;
        Vector3D v1 = new(), v2 = new(); //, V;
        Matrix3x3 m1 = new(), m2 = new();

        if (index > 0 && index < _laws!.Length())
        {
            if (iLoc <= 0)
            {
                _laws.Value(index).GetDomain(out first, out last);
                _laws.Value(index).D0(last, ref m1, ref v1);
            }

            if (iLoc >= 0)
            {
                _laws.Value(index + 1).GetDomain(out first, out last);
                if (iLoc == 0)
                {
                    _laws.Value(index + 1).D0(first, ref m2, ref v2);
                }
                else
                {
                    _laws.Value(index + 1).D0(first, ref m1, ref v1);
                }
            }
        }

        if (index == 0 || index == _laws!.Length())
        {
            if (!_path.Closed() || (IsG1(index, tolMin) != 1))
            {
                isBary = false;
                if (index == 0)
                {
                    _laws!.Value(1).GetDomain(out first, out last);
                    _laws.Value(1).D0(first, ref m1, ref v1);
                }
                else
                {
                    _laws!.Value(_laws.Length()).GetDomain(out first, out last);
                    _laws.Value(_laws.Length()).D0(last, ref m1, ref v1);
                }
            }
            else
            {
                if (iLoc <= 0)
                {
                    _laws!.Value(_laws.Length()).GetDomain(out first, out last);
                    _laws.Value(_laws.Length()).D0(last, ref m1, ref v1);
                }

                if (iLoc >= 0)
                {
                    _laws!.Value(1).GetDomain(out first, out last);
                    if (iLoc == 0)
                    {
                        _laws.Value(1).D0(first, ref m2, ref v2);
                    }
                    else
                    {
                        _laws.Value(1).D0(first, ref m1, ref v1);
                    }
                }
            }
        }

        p = Tool.Pnt(input);

        if (isBary)
        {
            Xyz p1 = p.XYZ(), p2 = p.XYZ();
            p1 *= m1;
            p1 += v1.XYZ();
            p2 *= m2;
            p2 += v2.XYZ();

            Xyz coord = default;
            coord.SetLinearForm(0.5, p1, 0.5, p2);
            p = new Point3D(coord);
            p1 -= p2;
            double tol = p1.Modulus() / 2;
            tol += tolMin;
            b.MakeVertex(ref output, p, tol);
        }
        else
        {
            p = new Point3D(p.XYZ() * m1);
            p = new Point3D(p.XYZ() + v1.XYZ());
            b.MakeVertex(ref output, p, tolMin);
        }
    }

    //=================================================================================================

    /// <summary>The curvilinear bounds of the law of rank <paramref name="index"/>.</summary>
    public void CurvilinearBounds(int index, out double first, out double last)
    {
        first = _length!.Value(index);
        last = _length.Value(index + 1);
        if (last < 0)
        { // It is required to carry out the calculation
            int ii, nbE = _edges!.Length();
            double length;

            for (ii = 1, length = 0.0; ii <= nbE; ii++)
            {
                _laws!.Value(ii).GetDomain(out _, out _);
                length += AbscissaPoint.Length(_laws.Value(ii).GetCurve()!, _tol);
                _length.SetValue(ii + 1, length);
            }

            first = _length.Value(index);
            last = _length.Value(index + 1);
        }
    }

    /// <summary>True if the path is closed (by its flag or by its end vertices).</summary>
    public bool IsClosed()
    {
        if (_path.Closed())
        {
            return true;
        }

        Exploration.Vertices(_path, out Vertex v1, out Vertex v2);
        return (v1.IsSame(v2));
    }

    //=======================================================================
    // function : IsG1
    // purpose  : Evaluate the continuity of the law by a vertex
    //=======================================================================

    /// <summary>
    /// The continuity of the law at the vertex of rank <paramref name="index"/> between two edges of the path: -1 not
    /// connected, 0 connected (G0), 1 tangent (G1).
    /// </summary>
    public int IsG1(int index, double spatialTolerance = 1.0e-7, double angularTolerance = 1.0e-4)
    {
        Vector3D v1 = new(), dv1 = new(), v2 = new(), dv2 = new();
        Matrix3x3 m1 = new(), m2 = new(), dm1 = new(), dm2 = new();
        double first, last, epsNul = 1.0e-12;
        double tolEps = spatialTolerance;
        bool Ok_D1 = false;
        Vertex v;
        Edge e = new();
        NCollection_Array1<Point2D> bid1 = new(1, 1);
        NCollection_Array1<Vector2D> bid2 = new(1, 1);

        if (index > 0 && index < _laws!.Length())
        {
            _laws.Value(index).GetDomain(out first, out last);
            Ok_D1 = _laws.Value(index).D1(last, ref m1, ref v1, ref dm1, ref dv1, bid1, bid2);
            if (!Ok_D1)
            {
                _laws.Value(index).D0(last, ref m1, ref v1);
            }

            _laws.Value(index + 1).GetDomain(out first, out last);
            if (Ok_D1)
            {
                Ok_D1 = _laws.Value(index + 1).D1(first, ref m2, ref v2, ref dm2, ref dv2, bid1, bid2);
            }
            if (!Ok_D1)
            {
                _laws.Value(index + 1).D0(first, ref m2, ref v2);
            }

            e = ShapeCasts.Edge(_edges!.Value(index + 1));
        }
        if (index == 0 || index == _laws!.Length())
        {
            if (!_path.Closed())
            {
                return -1;
            }
            _laws!.Value(_laws.Length()).GetDomain(out first, out last);
            Ok_D1 = _laws.Value(_laws.Length()).D1(last, ref m1, ref v1, ref dm1, ref dv1, bid1, bid2);
            if (!Ok_D1)
            {
                _laws.Value(_laws.Length()).D0(last, ref m1, ref v1);
            }

            _laws.Value(1).GetDomain(out first, out last);
            if (Ok_D1)
            {
                // OCCT: the result isn't kept in Ok_D1 here, unlike the branch above
                _laws.Value(1).D1(first, ref m2, ref v2, ref dm2, ref dv2, bid1, bid2);
            }
            if (!Ok_D1)
            {
                _laws.Value(1).D0(first, ref m2, ref v2);
            }

            e = ShapeCasts.Edge(_edges!.Value(1));
        }

        if (e.Orientation() == Reversed)
        {
            v = Exploration.LastVertex(e);
        }
        else
        {
            v = Exploration.FirstVertex(e);
        }

        tolEps += 2 * Tool.Tolerance(v);

        bool isG0 = true;
        bool isG1 = true;

        if ((v1 - v2).Magnitude() > tolEps)
        {
            isG0 = false;
        }
        if (Norm(m1 - m2) > spatialTolerance)
        {
            isG0 = false;
        }

        if (!isG0)
        {
            return -1;
        }
        if (!Ok_D1)
        {
            return 0; // No control of the derivative
        }

        if ((dv1.Magnitude() > epsNul) && (dv2.Magnitude() > epsNul)
            && (dv1.Angle(dv2) > angularTolerance))
        {
            isG1 = false;
        }

        // For the next, the tests are mostly empirical
        double norm1 = Norm(dm1);
        double norm2 = Norm(dm2);
        // It two 2 norms are null, it is good
        if ((norm1 > epsNul) || (norm2 > epsNul))
        {
            // otherwise the normalized matrices are compared
            if ((norm1 > epsNul) && (norm2 > epsNul))
            {
                dm1 /= norm1;
                dm2 /= norm2;
                if (Norm(dm1 - dm2) > angularTolerance)
                {
                    isG1 = false;
                }
            }
            else
            {
                isG1 = false; // 1 Null the other is not
            }
        }

        if (isG1)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    //=================================================================================================

    /// <summary>
    /// The index of the law and the parameter on it of the curvilinear abscissa <paramref name="abcissa"/>
    /// (<paramref name="index"/> 0 when the abscissa is past the path's end; <paramref name="u"/> then unchanged).
    /// </summary>
    public void Parameter(double abcissa, out int index, ref double u)
    {
        int iedge, nbE = _edges!.Length();
        bool trouve = false;

        // Control that the lengths are calculated
        if (_length!.Value(nbE + 1) < 0)
        {
            CurvilinearBounds(nbE, out _, out _);
        }

        // Find the interval
        for (iedge = 1; iedge <= nbE && !trouve;)
        {
            if (_length.Value(iedge + 1) >= abcissa)
            {
                trouve = true;
            }
            else
            {
                iedge++;
            }
        }

        if (trouve)
        {
            SurfaceLocationLaw law = _laws!.Value(iedge);
            law.GetDomain(out double f, out double l);

            if (abcissa == _length.Value(iedge + 1))
            {
                u = l;
            }
            else if (abcissa == _length.Value(iedge))
            {
                u = f;
            }
            else
            {
                AbscissaPoint absC = new(_tol,
                                                _laws.Value(iedge).GetCurve()!,
                                                abcissa - _length.Value(iedge),
                                                f);
                u = absC.Parameter();
            }
            index = iedge;
        }
        else
        {
            index = 0;
        }
    }

    //===================================================================
    // function : D0
    // purpose  : Position of a section, with a given curviline abscissa
    //===================================================================

    /// <summary>
    /// <paramref name="w"/> placed by the law at the curvilinear abscissa <paramref name="abcissa"/>: a frozen
    /// transformed copy (null past the path's end).
    /// </summary>
    public void D0(double abcissa, ref Shape w)
    {
        InternalD0(abcissa, ref w);
        w.TShape()?.FreezeGraph();
    }

    /// <summary><see cref="D0"/> without freezing the copy, for the pipe shell that sweeps it (BRepFill_PipeShell).</summary>
    internal void InternalD0(double abcissa, ref Shape w)
    {
        double u = 0;
        Matrix3x3 m = new();
        Vector3D v = new();

        Parameter(abcissa, out int ind, ref u);
        if (ind != 0)
        {
            // Positionement
            _laws!.Value(ind).D0(u, ref m, ref v);
            Transform3D fila = new();
            fila = fila.WithValues(m[1, 1],
                           m[1, 2],
                           m[1, 3],
                           v.X(),
                           m[2, 1],
                           m[2, 2],
                           m[2, 3],
                           v.Y(),
                           m[3, 1],
                           m[3, 2],
                           m[3, 3],
                           v.Z());
            // TopLoc_Location Loc(fila);
            // W.Location(Loc.Multiplied(W.Location()));
            w = new Transform(w, fila, true).InternalShape(); // copy
            ///////////////////////////////////////////
        }
        else
        {
            w.Nullify();
        }
    }

    //=======================================================================
    // function : Abscissa
    // purpose  : Calculate the abscissa of a point
    //=======================================================================

    /// <summary>The curvilinear abscissa of the point at <paramref name="param"/> on the law of rank <paramref name="index"/>.</summary>
    public double Abscissa(int index, double param)
    {
        double length = _length!.Value(index);
        if (length < 0)
        {
            // OCCT: takes the edge's last bound, not its first, when the lengths aren't computed yet (occt-issues.md)
            CurvilinearBounds(index, out _, out length);
        }

        length += AbscissaPoint.Length(_laws!.Value(index).GetCurve()!,
                                              _laws.Value(index).GetCurve()!.FirstParameter(),
                                              param,
                                              _tol);
        return length;
    }
}
