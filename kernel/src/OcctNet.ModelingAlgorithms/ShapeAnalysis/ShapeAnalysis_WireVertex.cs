// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_WireVertex.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_WireVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Analyzes and records the status of the vertices of a wire loaded in a <see cref="ShapeExtend_WireData"/>: for each
/// vertex, the end of the preceding edge is compared with the start of the following one, and a status, a position
/// and parameters are recorded, for use by fixing tools. Only reads the wire.
/// </summary>
/// <remarks>
/// A working object (decision 21). Vertex <c>num</c> is the end vertex of edge <c>num</c>: edge <c>num</c> precedes
/// it, edge <c>num+1</c> (edge 1 for the last) follows it.
/// </remarks>
public sealed class ShapeAnalysis_WireVertex
{
    private ShapeExtend_WireData? myWire;
    private NCollection_HArray1<int>? myStat;
    private NCollection_HArray1<gp_XYZ>? myPos;
    private NCollection_HArray1<double>? myUPre;
    private NCollection_HArray1<double>? myUFol;
    private double myPreci;
    private bool myDone;

    /// <summary>Empty constructor.</summary>
    public ShapeAnalysis_WireVertex()
    {
        myDone = false;
        myPreci = Occt.Precision.Confusion();
    }

    /// <summary>Loads a wire (the precision argument is unused, as in OCCT).</summary>
    public void Init(in TopoDS_Wire wire, double preci)
    {
        Init(new ShapeExtend_WireData(wire), preci);
    }

    /// <summary>Loads a wire data (the precision argument is unused, as in OCCT). An empty wire is ignored.</summary>
    public void Init(ShapeExtend_WireData sbwd, double preci)
    {
        _ = preci;
        int nb = sbwd.NbEdges();
        if (nb == 0)
        {
            return;
        }
        myDone = false;
        myWire = sbwd;
        myStat = new NCollection_HArray1<int>(1, nb);
        myStat.Init(0);
        myPos = new NCollection_HArray1<gp_XYZ>(1, nb);
        myUPre = new NCollection_HArray1<double>(1, nb);
        myUPre.Init(0.0);
        myUFol = new NCollection_HArray1<double>(1, nb);
        myUFol.Init(0.0);
    }

    /// <summary>Loads a wire.</summary>
    public void Load(in TopoDS_Wire wire)
    {
        Init(wire, myPreci);
    }

    /// <summary>Loads a wire data.</summary>
    public void Load(ShapeExtend_WireData sbwd)
    {
        Init(sbwd, myPreci);
    }

    /// <summary>Sets the precision for work.</summary>
    public void SetPrecision(double preci)
    {
        myPreci = preci;
        myDone = false;
    }

    /// <summary>Analyses each vertex: compares the end of the preceding edge with the start of the following one.</summary>
    public void Analyze()
    {
        if (myStat is null)
        {
            return;
        }
        myDone = true;
        //  Analyse des vertex qui se suivent
        int i, j, nb = myStat.Length(), stat;
        ShapeAnalysis_Edge EA = new();
        for (i = 1; i <= nb; i++)
        {
            stat = -1; // au depart

            j = (i == nb ? 1 : i + 1);
            TopoDS_Vertex V1 = EA.LastVertex(myWire!.Edge(i));
            TopoDS_Vertex V2 = EA.FirstVertex(myWire.Edge(j));
            gp_Pnt PV1 = BRep_Tool.Pnt(V1);
            gp_Pnt PV2 = BRep_Tool.Pnt(V2);
            double tol1 = BRep_Tool.Tolerance(V1);
            double tol2 = BRep_Tool.Tolerance(V2);
            EA.Curve3d(myWire.Edge(i), out Geom_Curve? c1, out double cf, out double upre);
            EA.Curve3d(myWire.Edge(j), out Geom_Curve? c2, out double ufol, out double cl);
            if (c1 is null || c2 is null)
            {
                continue; // on ne peut rien faire ...
            }
            gp_Pnt P1 = c1.Value(upre);
            gp_Pnt P2 = c2.Value(ufol);

            //   Est-ce que le jeu de vertex convient ? (meme si V1 == V2, on verifie)
            double d1 = PV1.Distance(P1);
            double d2 = PV2.Distance(P2);
            double dd = PV1.Distance(PV2);
            if (d1 <= tol1 && d2 <= tol2 && dd <= (tol1 + tol2))
            {
                stat = 1;
            }
            else if (d1 <= myPreci && d2 <= myPreci && dd <= myPreci)
            {
                stat = 2;
            }
            myStat.SetValue(i, -1); // par defaut
            if (stat > 0)
            {
                if (V1 == V2) // OCCT: == compares orientations; a REVERSED last vertex never equals the FORWARD first one (occt-issues.md)
                {
                    stat = 0;
                }
            }
            if (stat >= 0)
            {
                myStat.SetValue(i, stat);
                continue;
            }
            //    Restent les autres cas !

            //    Une edge se termine sur l autre : il faudra simplement relimiter
            //    Projection calculee sur une demi-edge (pour eviter les pbs de couture)
            double dj1 = new ShapeAnalysis_Curve().Project(c1, P2, myPreci, out gp_Pnt PJ1, out double U1, (cf + upre) / 2, upre);
            double dj2 = new ShapeAnalysis_Curve().Project(c2, P1, myPreci, out gp_Pnt PJ2, out double U2, ufol, (ufol + cl) / 2);
            if (dj1 <= myPreci)
            {
                SetStart(i, PJ1.XYZ(), U1);
                continue;
            }
            else if (dj2 <= myPreci)
            {
                SetEnd(i, PJ2.XYZ(), U2);
                continue;
            }

            //    Restent a verifier les intersections et prolongations !
        }
    }

    /// <summary>Records the status "same vertex" (0) on vertex <paramref name="num"/>.</summary>
    public void SetSameVertex(int num)
    {
        myStat!.SetValue(num, 0);
    }

    /// <summary>Records the status "same coords" (1, at the vertices' tolerances).</summary>
    public void SetSameCoords(int num)
    {
        myStat!.SetValue(num, 1);
    }

    /// <summary>Records the status "close coords" (2, at the precision).</summary>
    public void SetClose(int num)
    {
        myStat!.SetValue(num, 2);
    }

    /// <summary>
    /// Records status 3: vertex <paramref name="num"/> is the end of the preceding edge, and its projection on the
    /// following one, at parameter <paramref name="ufol"/>, lies on it.
    /// </summary>
    public void SetEnd(int num, in gp_XYZ pos, double ufol)
    {
        myStat!.SetValue(num, 3);
        myPos!.SetValue(num, pos);
        myUFol!.SetValue(num, ufol);
    }

    /// <summary>
    /// Records status 4: vertex <paramref name="num"/> is the start of the following edge, and its projection on the
    /// preceding one, at parameter <paramref name="upre"/>, lies on it.
    /// </summary>
    public void SetStart(int num, in gp_XYZ pos, double upre)
    {
        myStat!.SetValue(num, 4);
        myPos!.SetValue(num, pos);
        myUFol!.SetValue(num, upre); // OCCT bug: stores the parameter on the preceding edge as UFollowing (occt-issues.md)
    }

    /// <summary>Records status 5: vertex <paramref name="num"/> is the intersection of both edges.</summary>
    public void SetInters(int num, in gp_XYZ pos, double upre, double ufol)
    {
        myStat!.SetValue(num, 5);
        myPos!.SetValue(num, pos);
        myUPre!.SetValue(num, upre);
        myUFol!.SetValue(num, ufol);
    }

    /// <summary>Records status -1: vertex <paramref name="num"/> can't be said to be the same vertex.</summary>
    public void SetDisjoined(int num)
    {
        myStat!.SetValue(num, -1);
    }

    /// <summary>True if the analysis was performed.</summary>
    public bool IsDone() => myDone;

    /// <summary>The precision used in the analysis.</summary>
    public double Precision() => myPreci;

    /// <summary>The number of edges in the analysed wire (the length of all arrays).</summary>
    public int NbEdges() => myWire!.NbEdges();

    /// <summary>The analysed wire.</summary>
    public ShapeExtend_WireData? WireData() => myWire;

    /// <summary>The recorded status of a vertex (see <see cref="Data"/>).</summary>
    public int Status(int num) => myStat!.Value(num);

    /// <summary>The recorded position of a vertex.</summary>
    public gp_XYZ Position(int num) => myPos!.Value(num);

    /// <summary>The recorded parameter on the preceding edge.</summary>
    public double UPrevious(int num) => myUPre!.Value(num);

    /// <summary>The recorded parameter on the following edge.</summary>
    public double UFollowing(int num) => myUFol!.Value(num);

    /// <summary>
    /// The recorded status of a vertex, with its recorded position and parameters on both edges. Status 0 same, 1 same
    /// coords, 2 close, 3 end (position, following), 4 start (position, preceding), 5 intersection (all), -1 disjoined.
    /// </summary>
    public int Data(int num, out gp_XYZ pos, out double upre, out double ufol)
    {
        pos = myPos!.Value(num);
        upre = myUPre!.Value(num);
        ufol = myUFol!.Value(num);
        return myStat!.Value(num);
    }

    /// <summary>The rank of the next vertex after <paramref name="num"/> with status <paramref name="stat"/>; 0 if none.</summary>
    public int NextStatus(int stat, int num = 0)
    {
        // szv#4:S4163:12Mar99 optimized
        if (myStat is not null)
        {
            int i, nb = myStat.Length();
            for (i = num + 1; i <= nb; i++)
            {
                if (myStat.Value(i) == stat)
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
        if (myStat is not null)
        {
            int i, nb = myStat.Length();
            for (i = num + 1; i <= nb; i++)
            {
                int stat = myStat.Value(i);
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
