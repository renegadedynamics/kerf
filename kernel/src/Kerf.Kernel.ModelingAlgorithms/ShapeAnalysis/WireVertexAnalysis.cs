// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_WireVertex.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_WireVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Healing.Analysis;

/// <summary>
/// Analyzes and records the status of the vertices of a wire loaded in a <see cref="ExtendWireData"/>: for each
/// vertex, the end of the preceding edge is compared with the start of the following one, and a status, a position
/// and parameters are recorded, for use by fixing tools. Only reads the wire.
/// </summary>
/// <remarks>
/// A working object (decision 21). Vertex <c>num</c> is the end vertex of edge <c>num</c>: edge <c>num</c> precedes
/// it, edge <c>num+1</c> (edge 1 for the last) follows it.
/// </remarks>
internal sealed class WireVertexAnalysis
{
    private ExtendWireData? _wire;
    private NCollection_HArray1<int>? _stat;
    private NCollection_HArray1<Xyz>? _pos;
    private NCollection_HArray1<double>? _uPre;
    private NCollection_HArray1<double>? _uFol;
    private double _preci;
    private bool _done;

    /// <summary>Empty constructor.</summary>
    public WireVertexAnalysis()
    {
        _done = false;
        _preci = Kerf.Engine.Foundation.Precision.Confusion();
    }

    /// <summary>Loads a wire (the precision argument is unused, as in OCCT).</summary>
    public void Init(in Wire wire, double preci)
    {
        Init(new ExtendWireData(wire), preci);
    }

    /// <summary>Loads a wire data (the precision argument is unused, as in OCCT). An empty wire is ignored.</summary>
    public void Init(ExtendWireData sbwd, double preci)
    {
        _ = preci;
        int nb = sbwd.EdgeCount;
        if (nb == 0)
        {
            return;
        }
        _done = false;
        _wire = sbwd;
        _stat = new NCollection_HArray1<int>(1, nb);
        _stat.Init(0);
        _pos = new NCollection_HArray1<Xyz>(1, nb);
        _uPre = new NCollection_HArray1<double>(1, nb);
        _uPre.Init(0.0);
        _uFol = new NCollection_HArray1<double>(1, nb);
        _uFol.Init(0.0);
    }

    /// <summary>Loads a wire.</summary>
    public void Load(in Wire wire)
    {
        Init(wire, _preci);
    }

    /// <summary>Loads a wire data.</summary>
    public void Load(ExtendWireData sbwd)
    {
        Init(sbwd, _preci);
    }

    /// <summary>Sets the precision for work.</summary>
    public void SetPrecision(double preci)
    {
        _preci = preci;
        _done = false;
    }

    /// <summary>Analyses each vertex: compares the end of the preceding edge with the start of the following one.</summary>
    public void Analyze()
    {
        if (_stat is null)
        {
            return;
        }
        _done = true;
        //  Analyse des vertex qui se suivent
        int i, j, nb = _stat.Length(), stat;
        EdgeAnalysis ea = new();
        for (i = 1; i <= nb; i++)
        {
            stat = -1; // au depart

            j = (i == nb ? 1 : i + 1);
            Vertex v1 = ea.LastVertex(_wire!.Edge(i));
            Vertex v2 = ea.FirstVertex(_wire.Edge(j));
            Point3D pv1 = Tool.Pnt(v1);
            Point3D pv2 = Tool.Pnt(v2);
            double tol1 = Tool.Tolerance(v1);
            double tol2 = Tool.Tolerance(v2);
            ea.Curve3d(_wire.Edge(i), out Curve3D? c1, out double cf, out double upre);
            ea.Curve3d(_wire.Edge(j), out Curve3D? c2, out double ufol, out double cl);
            if (c1 is null || c2 is null)
            {
                continue; // on ne peut rien faire ...
            }
            Point3D p1 = c1.Value(upre);
            Point3D p2 = c2.Value(ufol);

            //   Est-ce que le jeu de vertex convient ? (meme si V1 == V2, on verifie)
            double d1 = pv1.DistanceTo(p1);
            double d2 = pv2.DistanceTo(p2);
            double dd = pv1.DistanceTo(pv2);
            if (d1 <= tol1 && d2 <= tol2 && dd <= (tol1 + tol2))
            {
                stat = 1;
            }
            else if (d1 <= _preci && d2 <= _preci && dd <= _preci)
            {
                stat = 2;
            }
            _stat.SetValue(i, -1); // par defaut
            if (stat > 0)
            {
                if (v1 == v2) // OCCT: == compares orientations; a REVERSED last vertex never equals the FORWARD first one (occt-issues.md)
                {
                    stat = 0;
                }
            }
            if (stat >= 0)
            {
                _stat.SetValue(i, stat);
                continue;
            }
            //    Restent les autres cas !

            //    Une edge se termine sur l autre : il faudra simplement relimiter
            //    Projection calculee sur une demi-edge (pour eviter les pbs de couture)
            double dj1 = new CurveAnalysis().Project(c1, p2, _preci, out Point3D pj1, out double u1, (cf + upre) / 2, upre);
            double dj2 = new CurveAnalysis().Project(c2, p1, _preci, out Point3D pj2, out double u2, ufol, (ufol + cl) / 2);
            if (dj1 <= _preci)
            {
                SetStart(i, pj1.XYZ(), u1);
                continue;
            }
            else if (dj2 <= _preci)
            {
                SetEnd(i, pj2.XYZ(), u2);
                continue;
            }

            //    Restent a verifier les intersections et prolongations !
        }
    }

    /// <summary>Records the status "same vertex" (0) on vertex <paramref name="num"/>.</summary>
    public void SetSameVertex(int num)
    {
        _stat!.SetValue(num, 0);
    }

    /// <summary>Records the status "same coords" (1, at the vertices' tolerances).</summary>
    public void SetSameCoords(int num)
    {
        _stat!.SetValue(num, 1);
    }

    /// <summary>Records the status "close coords" (2, at the precision).</summary>
    public void SetClose(int num)
    {
        _stat!.SetValue(num, 2);
    }

    /// <summary>
    /// Records status 3: vertex <paramref name="num"/> is the end of the preceding edge, and its projection on the
    /// following one, at parameter <paramref name="ufol"/>, lies on it.
    /// </summary>
    public void SetEnd(int num, in Xyz pos, double ufol)
    {
        _stat!.SetValue(num, 3);
        _pos!.SetValue(num, pos);
        _uFol!.SetValue(num, ufol);
    }

    /// <summary>
    /// Records status 4: vertex <paramref name="num"/> is the start of the following edge, and its projection on the
    /// preceding one, at parameter <paramref name="upre"/>, lies on it.
    /// </summary>
    public void SetStart(int num, in Xyz pos, double upre)
    {
        _stat!.SetValue(num, 4);
        _pos!.SetValue(num, pos);
        _uFol!.SetValue(num, upre); // OCCT bug: stores the parameter on the preceding edge as UFollowing (occt-issues.md)
    }

    /// <summary>Records status 5: vertex <paramref name="num"/> is the intersection of both edges.</summary>
    public void SetInters(int num, in Xyz pos, double upre, double ufol)
    {
        _stat!.SetValue(num, 5);
        _pos!.SetValue(num, pos);
        _uPre!.SetValue(num, upre);
        _uFol!.SetValue(num, ufol);
    }

    /// <summary>Records status -1: vertex <paramref name="num"/> can't be said to be the same vertex.</summary>
    public void SetDisjoined(int num)
    {
        _stat!.SetValue(num, -1);
    }

    /// <summary>True if the analysis was performed.</summary>
    public bool IsDone => _done;

    /// <summary>The precision used in the analysis.</summary>
    public double Precision() => _preci;

    /// <summary>The number of edges in the analysed wire (the length of all arrays).</summary>
    public int EdgeCount => _wire!.EdgeCount;

    /// <summary>The analysed wire.</summary>
    public ExtendWireData? WireData() => _wire;

    /// <summary>The recorded status of a vertex (see <see cref="Data"/>).</summary>
    public int Status(int num) => _stat!.Value(num);

    /// <summary>The recorded position of a vertex.</summary>
    public Xyz Position(int num) => _pos!.Value(num);

    /// <summary>The recorded parameter on the preceding edge.</summary>
    public double UPrevious(int num) => _uPre!.Value(num);

    /// <summary>The recorded parameter on the following edge.</summary>
    public double UFollowing(int num) => _uFol!.Value(num);

    /// <summary>
    /// The recorded status of a vertex, with its recorded position and parameters on both edges. Status 0 same, 1 same
    /// coords, 2 close, 3 end (position, following), 4 start (position, preceding), 5 intersection (all), -1 disjoined.
    /// </summary>
    public int Data(int num, out Xyz pos, out double upre, out double ufol)
    {
        pos = _pos!.Value(num);
        upre = _uPre!.Value(num);
        ufol = _uFol!.Value(num);
        return _stat!.Value(num);
    }

    /// <summary>The rank of the next vertex after <paramref name="num"/> with status <paramref name="stat"/>; 0 if none.</summary>
    public int NextStatus(int stat, int num = 0)
    {
        // szv#4:S4163:12Mar99 optimized
        if (_stat is not null)
        {
            int i, nb = _stat.Length();
            for (i = num + 1; i <= nb; i++)
            {
                if (_stat.Value(i) == stat)
                {
                    return i;
                }
            }
        }
        return 0;
    }

    /// <summary>
    /// The rank of the next vertex after <paramref name="num"/> matching a criterion; 0 if none. Criteria: 0 same
    /// vertex (status 0), 1 status &gt; 0 (OCCT's documentation says &gt;= 0), 2 same coords (0, 1, 2), 3 same coords but
    /// not same vertex (1, 2), 4 redefined coords (3, 4, 5), -1 no solution (status &lt; 0).
    /// </summary>
    public int NextCriter(int crit, int num = 0)
    {
        // szv#4:S4163:12Mar99 optimized
        if (_stat is not null)
        {
            int i, nb = _stat.Length();
            for (i = num + 1; i <= nb; i++)
            {
                int stat = _stat.Value(i);
                if ((crit == -1 && stat < 0) || (crit == 0 && stat == 0) || (crit == 1 && stat > 0)
                    || (crit == 2 && (stat >= 0 && stat <= 2)) || (crit == 3 && (stat == 1 || stat == 2))
                    || (crit == 4 && stat > 2))
                {
                    return i;
                }
            }
        }
        return 0;
    }
}
