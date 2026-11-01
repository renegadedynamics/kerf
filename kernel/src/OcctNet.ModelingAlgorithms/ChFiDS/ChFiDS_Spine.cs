// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_Spine.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_Spine.lxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_Spine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Modified by isg, Thu Mar 17 09:21:31 1994

using static Occt.ChFiDS_ChamfMode;
using static Occt.ChFiDS_ErrorStatus;
using static Occt.ChFiDS_State;
using static Occt.ChFiDS_TypeOfConcavity;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// The guideline of a 3D fillet or chamfer: a chain of edges of the solid, tangent to each other, with the nature of
/// its ends (free boundary, section, closed) and its parameterisation by curvilinear abscissa.
/// </summary>
/// <remarks>
/// <para>
/// OCCT notes that the guideline so represented is not C2, although the path claims it; the elementary spines
/// (<see cref="ChFiDS_ElSpine"/>) are the C2 curves the walking follows.
/// </para>
/// <para>
/// A working object of the fillet builders (decision 21). Its edges are the input shape's own (frozen) edges; the
/// builders change the spine's state (status, parameters, elementary spines, error status) through the internal
/// members. Evaluating it by abscissa (<c>Value</c>, <c>D1</c>, <c>Absc</c>, <c>Parameter</c>) moves its current
/// elementary curve, so those members are internal too; the members OCCT declares <c>const</c> are public.
/// </para>
/// </remarks>
public class ChFiDS_Spine : Standard_Transient
{
    protected bool splitdone;
    protected readonly NCollection_List<ChFiDS_ElSpine> elspines = new();
    protected readonly NCollection_List<ChFiDS_ElSpine> offset_elspines = new();
    protected ChFiDS_ChamfMode myMode;

    private readonly BRepAdaptor_Curve myCurve = new();
    // OCCT's myOffsetCurve is never used: left out.
    private int indexofcurve;
    private ChFiDS_TypeOfConcavity myTypeOfConcavity;
    private ChFiDS_State firstState;
    private ChFiDS_State lastState;
    private readonly NCollection_Sequence<TopoDS_Shape> spine = new();
    private readonly NCollection_Sequence<TopoDS_Shape> offsetspine = new();
    private NCollection_HArray1<double>? abscissa;
    // OCCT's offset_abscissa is never used: left out.
    private double tolesp;
    private double firstparam;
    private double lastparam;
    private bool firstprolon;
    private bool lastprolon;
    private bool firstistgt;
    private bool lastistgt;
    private double firsttgtpar;
    private double lasttgtpar;
    private bool hasfirsttgt;
    private bool haslasttgt;
    private gp_Pnt firstori = new();
    private gp_Pnt lastori = new();
    private gp_Vec firsttgt = new();
    private gp_Vec lasttgt = new();
    private double valref;
    private bool hasref;
    private ChFiDS_ErrorStatus errorstate;

    public ChFiDS_Spine()
        : this(Precision.Confusion())
    {
    }

    public ChFiDS_Spine(double Tol)
    {
        splitdone = false;
        myMode = ChFiDS_ClassicChamfer;
        indexofcurve = 0;
        myTypeOfConcavity = ChFiDS_Other;
        firstState = ChFiDS_OnSame;
        lastState = ChFiDS_OnSame;
        tolesp = Tol;
        firstparam = 0.0;
        lastparam = 0.0;
        firstprolon = false;
        lastprolon = false;
        firstistgt = false;
        lastistgt = false;
        firsttgtpar = 0.0;
        lasttgtpar = 0.0;
        hasfirsttgt = false;
        haslasttgt = false;
        valref = 0.0;
        hasref = false;
        errorstate = ChFiDS_Ok;
    }

    /// <summary>Appends an edge to the guideline.</summary>
    internal void SetEdges(in TopoDS_Edge E)
    {
        spine.Append(E);
    }

    /// <summary>Appends an edge to the offset guideline.</summary>
    internal void SetOffsetEdges(in TopoDS_Edge E)
    {
        offsetspine.Append(E);
    }

    /// <summary>Puts the edge first, before all the others.</summary>
    internal void PutInFirst(in TopoDS_Edge E)
    {
        spine.InsertBefore(1, E);
    }

    /// <summary>Puts the offset edge first, before all the others.</summary>
    internal void PutInFirstOffset(in TopoDS_Edge E)
    {
        offsetspine.InsertBefore(1, E);
    }

    public int NbEdges()
    {
        return spine.Length();
    }

    public TopoDS_Edge Edges(int I)
    {
        return TopoDS.Edge(spine.Value(I));
    }

    public TopoDS_Edge OffsetEdges(int I)
    {
        return TopoDS.Edge(offsetspine.Value(I));
    }

    /// <summary>Stores whether the chain starts on a section, a free boundary or is closed.</summary>
    internal void SetFirstStatus(ChFiDS_State S)
    {
        firstState = S;
    }

    /// <summary>Stores whether the chain ends on a section, a free boundary or is closed.</summary>
    internal void SetLastStatus(ChFiDS_State S)
    {
        lastState = S;
    }

    internal virtual void AppendElSpine(ChFiDS_ElSpine Els)
    {
        elspines.Append(Els);
    }

    internal virtual void AppendOffsetElSpine(ChFiDS_ElSpine Els)
    {
        offset_elspines.Append(Els);
    }

    /// <summary>The elementary spine of the edge <paramref name="E"/>.</summary>
    public ChFiDS_ElSpine? ElSpine(in TopoDS_Edge E)
    {
        return ElSpine(Index(E));
    }

    /// <summary>The elementary spine of the edge of index <paramref name="IE"/>.</summary>
    public ChFiDS_ElSpine? ElSpine(int IE)
    {
        double wmil = 0.5 * (FirstParameter(IE) + LastParameter(IE));
        if (IsPeriodic())
        {
            wmil = ElCLib.InPeriod(wmil, FirstParameter(), LastParameter());
        }

        return ElSpine(wmil);
    }

    /// <summary>The elementary spine at the abscissa <paramref name="W"/>, or null.</summary>
    public ChFiDS_ElSpine? ElSpine(double W)
    {
        if (elspines.Extent() == 1)
        {
            return elspines.First();
        }
        else
        {
            NCollection_List<ChFiDS_ElSpine>.Iterator It = new(elspines);
            for (; It.More(); It.Next())
            {
                ChFiDS_ElSpine cur = It.Value();
                double uf = cur.FirstParameter();
                double ul = cur.LastParameter();
                if (uf <= W && W <= ul)
                {
                    return cur;
                }
            }

            return null;
        }
    }

    internal NCollection_List<ChFiDS_ElSpine> ChangeElSpines()
    {
        return elspines;
    }

    internal NCollection_List<ChFiDS_ElSpine> ChangeOffsetElSpines()
    {
        return offset_elspines;
    }

    /// <summary>Forgets the elementary spines; with <paramref name="AllData"/>, also the extensions.</summary>
    internal virtual void Reset(bool AllData = false)
    {
        splitdone = false;
        // if(AllData && !isconstant.IsNull()) isconstant->ChangeArray1().Init(0);
        elspines.Clear();
        if (AllData)
        {
            firstparam = 0.0;
            lastparam = abscissa!.Value(abscissa.Upper());
            firstprolon = lastprolon = false;
        }
    }

    public bool SplitDone()
    {
        return splitdone;
    }

    internal void SplitDone(bool B)
    {
        splitdone = B;
    }

    /// <summary>
    /// Computes the abscissae of the edges, which the evaluation by abscissa needs, and lowers the tolerance to the
    /// curve's parameter range.
    /// </summary>
    internal void Load()
    {
        if (abscissa is not null)
        {
            // OCCT_DEBUG trace left out ("new load of CE").
        }

        int len = spine.Length();
        abscissa = new NCollection_HArray1<double>(1, len);
        double a1 = 0.0;
        for (int i = 1; i <= len; i++)
        {
            myCurve.Initialize(TopoDS.Edge(spine.Value(i)));
            a1 += GCPnts_AbscissaPoint.Length(myCurve);
            abscissa.SetValue(i, a1);
        }

        indexofcurve = 1;
        myCurve.Initialize(TopoDS.Edge(spine.Value(1)));

        // Here, we should update tolesp according to curve parameter range
        // if tolesp candidate less than default initial value.
        double umin = FirstParameter();
        double umax = LastParameter();

        double new_tolesp = 5.0e-5 * (umax - umin);
        if (tolesp > new_tolesp)
        {
            tolesp = new_tolesp;
        }
    }

    /// <summary>Returns <paramref name="R3d"/>.</summary>
    public double Resolution(double R3d)
    {
        return R3d;
    }

    /// <summary>True if the first and last vertices are the same.</summary>
    public bool IsClosed()
    {
        return (FirstVertex().IsSame(LastVertex()));
    }

    public double FirstParameter()
    {
        if (firstprolon)
        {
            return firstparam;
        }

        return 0.0;
    }

    public double LastParameter()
    {
        if (lastprolon)
        {
            return lastparam;
        }

        return abscissa!.Value(abscissa.Upper());
    }

    /// <summary>Extends the guideline at its start to <paramref name="Par"/>.</summary>
    internal void SetFirstParameter(double Par)
    {
        // OCCT_DEBUG traces left out.
        firstprolon = true;
        firstparam = Par;
    }

    /// <summary>Extends the guideline at its end to <paramref name="Par"/>.</summary>
    internal void SetLastParameter(double Par)
    {
        // OCCT_DEBUG traces left out.
        lastprolon = true;
        lastparam = Par;
    }

    /// <summary>The total length of the edges before the edge <paramref name="IndexSpine"/>.</summary>
    public double FirstParameter(int IndexSpine)
    {
        if (IndexSpine == 1)
        {
            return 0.0;
        }

        return abscissa!.Value(IndexSpine - 1);
    }

    /// <summary>The total length of the edges up to the edge <paramref name="IndexSpine"/> (included).</summary>
    public double LastParameter(int IndexSpine)
    {
        return abscissa!.Value(IndexSpine);
    }

    /// <summary>The length of the edge <paramref name="IndexSpine"/>.</summary>
    public double Length(int IndexSpine)
    {
        if (IndexSpine == 1)
        {
            return abscissa!.Value(IndexSpine);
        }

        return abscissa!.Value(IndexSpine) - abscissa.Value(IndexSpine - 1);
    }

    /// <summary>True if the chain is closed (its first status is <see cref="ChFiDS_Closed"/>).</summary>
    public bool IsPeriodic()
    {
        return (firstState == ChFiDS_Closed);
    }

    /// <summary>The length of the closed chain; throws <see cref="Standard_Failure"/> if it is not periodic.</summary>
    public double Period()
    {
        if (!IsPeriodic())
        {
            throw new Standard_Failure("Non-periodic Spine");
        }

        return abscissa!.Value(abscissa.Upper());
    }

    /// <summary>The abscissa of the parameter <paramref name="U"/> on the current edge.</summary>
    internal double Absc(double U)
    {
        return Absc(U, indexofcurve);
    }

    /// <summary>The abscissa of the parameter <paramref name="U"/> on the edge <paramref name="I"/>.</summary>
    internal double Absc(double U, int I)
    {
        if (indexofcurve != I)
        {
            indexofcurve = I;
            myCurve.Initialize(TopoDS.Edge(spine.Value(I)));
        }

        double L = FirstParameter(I);
        if (spine.Value(I).Orientation() == TopAbs_REVERSED)
        {
            L += GCPnts_AbscissaPoint.Length(myCurve, U, myCurve.LastParameter());
        }
        else
        {
            L += GCPnts_AbscissaPoint.Length(myCurve, myCurve.FirstParameter(), U);
        }

        return L;
    }

    /// <summary>The parameter <paramref name="U"/>, on its edge, of the abscissa <paramref name="AbsC"/>.</summary>
    internal void Parameter(double AbsC, out double U, bool Oriented = true)
    {
        int Index;
        for (Index = 1; Index < abscissa!.Length(); Index++)
        {
            if (AbsC < abscissa.Value(Index))
            {
                break;
            }
        }

        Parameter(Index, AbsC, out U, Oriented);
    }

    /// <summary>The parameter <paramref name="U"/>, on the edge <paramref name="Index"/>, of the abscissa
    /// <paramref name="AbsC"/>.</summary>
    internal void Parameter(int Index, double AbsC, out double U, bool Oriented = true)
    {
        if (Index != indexofcurve)
        {
            indexofcurve = Index;
            myCurve.Initialize(TopoDS.Edge(spine.Value(Index)));
        }

        double L;
        TopAbs_Orientation Or = spine.Value(Index).Orientation();
        if (Or == TopAbs_REVERSED)
        {
            L = abscissa!.Value(indexofcurve) - AbsC;
        }
        else if (indexofcurve == 1)
        {
            L = AbsC;
        }
        else
        {
            L = AbsC - abscissa!.Value(indexofcurve - 1);
        }

        double t = L / Length(Index);
        double uapp = (1.0 - t) * myCurve.FirstParameter() + t * myCurve.LastParameter();
        GCPnts_AbscissaPoint GCP = new(myCurve, L, myCurve.FirstParameter(), uapp);
        U = GCP.Parameter();
        if (Or == TopAbs_REVERSED && Oriented)
        {
            U = (myCurve.LastParameter() + myCurve.FirstParameter()) - U;
        }
    }

    internal gp_Pnt Value(double AbsC)
    {
        int Index;
        double L = AbsC;

        Prepare(ref L, out Index);

        if (Index == -1)
        {
            gp_Pnt Pp = firstori;
            gp_Vec Vp = firsttgt;
            Vp.Multiply(L);
            Pp.Translate(Vp);
            return Pp;
        }
        else if (Index == (abscissa!.Length() + 1))
        {
            gp_Pnt Pp = lastori;
            gp_Vec Vp = lasttgt;
            Vp.Multiply(L);
            Pp.Translate(Vp);
            return Pp;
        }

        if (Index != indexofcurve)
        {
            indexofcurve = Index;
            myCurve.Initialize(TopoDS.Edge(spine.Value(Index)));
        }

        double t = L / Length(Index);
        double uapp = (1.0 - t) * myCurve.FirstParameter() + t * myCurve.LastParameter();
        GCPnts_AbscissaPoint GCP = new(myCurve, L, myCurve.FirstParameter(), uapp);
        return myCurve.Value(GCP.Parameter());
    }

    internal void D0(double AbsC, out gp_Pnt P)
    {
        P = Value(AbsC);
    }

    /// <summary>The point and the unit tangent at the abscissa <paramref name="AbsC"/>.</summary>
    internal void D1(double AbsC, out gp_Pnt P, out gp_Vec V1)
    {
        int Index;
        double L = AbsC;

        Prepare(ref L, out Index);

        if (Index == -1)
        {
            P = firstori;
            V1 = firsttgt;
            gp_Vec Vp = firsttgt;
            Vp.Multiply(L);
            P.Translate(Vp);
        }
        else if (Index == (abscissa!.Length() + 1))
        {
            P = lastori;
            V1 = lasttgt;
            gp_Vec Vp = lasttgt;
            Vp.Multiply(L);
            P.Translate(Vp);
        }
        else
        {
            if (Index != indexofcurve)
            {
                indexofcurve = Index;
                myCurve.Initialize(TopoDS.Edge(spine.Value(Index)));
            }

            double t = L / Length(Index);
            double uapp = (1.0 - t) * myCurve.FirstParameter() + t * myCurve.LastParameter();
            GCPnts_AbscissaPoint GCP = new(myCurve, L, myCurve.FirstParameter(), uapp);
            myCurve.D1(GCP.Parameter(), out P, out V1);
            double D1 = 1.0 / V1.Magnitude();
            if (spine.Value(Index).Orientation() == TopAbs_REVERSED)
            {
                D1 = -D1;
            }

            V1.Multiply(D1);
        }
    }

    internal void D2(double AbsC, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2)
    {
        int Index;
        double L = AbsC;

        Prepare(ref L, out Index);

        if (Index == -1)
        {
            P = firstori;
            V1 = firsttgt;
            V2 = new gp_Vec();
            V2.SetCoord(0.0, 0.0, 0.0);
            gp_Vec Vp = firsttgt;
            Vp.Multiply(L);
            P.Translate(Vp);
        }
        else if (Index == (abscissa!.Length() + 1))
        {
            P = lastori;
            V1 = lasttgt;
            V2 = new gp_Vec();
            V2.SetCoord(0.0, 0.0, 0.0);
            gp_Vec Vp = lasttgt;
            Vp.Multiply(L);
            P.Translate(Vp);
        }
        else
        {
            if (Index != indexofcurve)
            {
                indexofcurve = Index;
                myCurve.Initialize(TopoDS.Edge(spine.Value(Index)));
            }

            double t = L / Length(Index);
            double uapp = (1.0 - t) * myCurve.FirstParameter() + t * myCurve.LastParameter();
            GCPnts_AbscissaPoint GCP = new(myCurve, L, myCurve.FirstParameter(), uapp);
            myCurve.D2(GCP.Parameter(), out P, out V1, out V2);
            double N1 = V1.SquareMagnitude();
            double D2 = -(V1.Dot(V2)) * (1.0 / N1) * (1.0 / N1);
            V2.Multiply(1.0 / N1);
            N1 = Math.Sqrt(N1);
            gp_Vec Va = V1.Multiplied(D2);
            V2.Add(Va);
            double D1 = 1.0 / N1;
            if (spine.Value(Index).Orientation() == TopAbs_REVERSED)
            {
                D1 = -D1;
            }

            V1.Multiply(D1);
        }
    }

    internal void SetCurrent(int Index)
    {
        if (Index != indexofcurve)
        {
            indexofcurve = Index;
            myCurve.Initialize(TopoDS.Edge(spine.Value(indexofcurve)));
        }
    }

    /// <summary>Sets the current curve to the edge <paramref name="Index"/> and returns it.</summary>
    internal BRepAdaptor_Curve CurrentElementarySpine(int Index)
    {
        if (Index != indexofcurve)
        {
            indexofcurve = Index;
            myCurve.Initialize(TopoDS.Edge(spine.Value(indexofcurve)));
        }

        return myCurve;
    }

    public int CurrentIndexOfElementarySpine()
    {
        return indexofcurve;
    }

    /// <summary>The type of the current curve.</summary>
    public new GeomAbs_CurveType GetType()
    {
        return myCurve.GetType();
    }

    /// <summary>The current curve as an oriented line starting at the edge's start.</summary>
    public gp_Lin Line()
    {
        gp_Lin LL = myCurve.Line();
        if (spine.Value(indexofcurve).Orientation() == TopAbs_REVERSED)
        {
            LL.Reverse();
            LL.SetLocation(myCurve.Value(myCurve.LastParameter()));
        }
        else
        {
            LL.SetLocation(myCurve.Value(myCurve.FirstParameter()));
        }

        return LL;
    }

    /// <summary>The current curve as an oriented circle starting at the edge's start.</summary>
    public gp_Circ Circle()
    {
        gp_Ax2 Ac = myCurve.Circle().Position();
        gp_Dir Dc = new gp_Dir(new gp_Vec(Ac.Location(), myCurve.Value(myCurve.FirstParameter())));
        gp_Dir ZZ = Ac.Direction();

        if (spine.Value(indexofcurve).Orientation() == TopAbs_REVERSED)
        {
            Dc = new gp_Dir(new gp_Vec(Ac.Location(), myCurve.Value(myCurve.LastParameter())));
            ZZ.Reverse();
        }

        gp_Ax2 A = new gp_Ax2(Ac.Location(), ZZ, Dc);
        return new gp_Circ(A, myCurve.Circle().Radius());
    }

    /// <summary>Whether the chain starts on a free boundary, on a break point, or is closed.</summary>
    public ChFiDS_State FirstStatus()
    {
        return firstState;
    }

    /// <summary>The state at the end of the chain.</summary>
    public ChFiDS_State LastStatus()
    {
        return lastState;
    }

    public ChFiDS_State Status(bool IsFirst)
    {
        if (IsFirst)
        {
            return firstState;
        }
        else
        {
            return lastState;
        }
    }

    /// <summary>The type of concavity of the connection.</summary>
    public ChFiDS_TypeOfConcavity GetTypeOfConcavity()
    {
        return myTypeOfConcavity;
    }

    internal void SetStatus(ChFiDS_State S, bool IsFirst)
    {
        if (IsFirst)
        {
            firstState = S;
        }
        else
        {
            lastState = S;
        }
    }

    internal void SetTypeOfConcavity(ChFiDS_TypeOfConcavity theType)
    {
        myTypeOfConcavity = theType;
    }

    /// <summary>True if the chain starts (or ends) on a tangency point.</summary>
    public bool IsTangencyExtremity(bool IsFirst)
    {
        if (IsFirst)
        {
            return firstistgt;
        }
        else
        {
            return lastistgt;
        }
    }

    internal void SetTangencyExtremity(bool IsTangency, bool IsFirst)
    {
        if (IsFirst)
        {
            firstistgt = IsTangency;
        }
        else
        {
            lastistgt = IsTangency;
        }
    }

    /// <summary>The abscissa of the vertex <paramref name="V"/>, or -1 if it is not on the chain.</summary>
    public double Absc(in TopoDS_Vertex V)
    {
        TopoDS_Vertex d, f;
        TopoDS_Edge E;
        for (int i = 1; i <= spine.Length(); i++)
        {
            E = TopoDS.Edge(spine.Value(i));
            TopExp.Vertices(E, out d, out f);
            if (d.IsSame(V) && E.Orientation() == TopAbs_FORWARD)
            {
                return FirstParameter(i);
            }

            if (d.IsSame(V) && E.Orientation() == TopAbs_REVERSED)
            {
                return LastParameter(i);
            }

            if (f.IsSame(V) && E.Orientation() == TopAbs_FORWARD)
            {
                return LastParameter(i);
            }

            if (f.IsSame(V) && E.Orientation() == TopAbs_REVERSED)
            {
                return FirstParameter(i);
            }
        }

        return -1.0;
    }

    public TopoDS_Vertex FirstVertex()
    {
        TopoDS_Edge E = TopoDS.Edge(spine.First());
        if (E.Orientation() == TopAbs_FORWARD)
        {
            return TopExp.FirstVertex(E);
        }

        return TopExp.LastVertex(E);
    }

    public TopoDS_Vertex LastVertex()
    {
        TopoDS_Edge E = TopoDS.Edge(spine.Last());
        if (E.Orientation() == TopAbs_FORWARD)
        {
            return TopExp.LastVertex(E);
        }

        return TopExp.FirstVertex(E);
    }

    /// <summary>Extends the guideline at its start by its tangent at <paramref name="W"/>.</summary>
    internal void SetFirstTgt(double W)
    {
        if (IsPeriodic())
        {
            throw new Standard_Failure("No extension by tangent on periodic contours");
        }

        // OCCT_DEBUG trace left out.

        // The flag is suspended if is already positioned to avoid
        // stopping d1
        hasfirsttgt = false;
        D1(W, out firstori, out firsttgt);
        // and it is reset.
        hasfirsttgt = true;
        firsttgtpar = W;
    }

    /// <summary>Extends the guideline at its end by its tangent at <paramref name="W"/>.</summary>
    internal void SetLastTgt(double W)
    {
        if (IsPeriodic())
        {
            throw new Standard_Failure("No extension by tangent periodic contours");
        }

        // OCCT_DEBUG trace left out.

        // The flag is suspended if is already positioned to avoid
        // stopping d1
        haslasttgt = false;
        D1(W, out lastori, out lasttgt);
        // and it is reset.
        haslasttgt = true;
        lasttgtpar = W;
    }

    public bool HasFirstTgt()
    {
        return hasfirsttgt;
    }

    public bool HasLastTgt()
    {
        return haslasttgt;
    }

    /// <summary>Sets a parameter reference for the approximation.</summary>
    internal void SetReference(double W)
    {
        hasref = true;
        double lll = abscissa!.Value(abscissa.Upper());
        if (IsPeriodic())
        {
            valref = ElCLib.InPeriod(W, 0.0, lll);
        }
        else
        {
            valref = W;
        }
    }

    /// <summary>Sets a parameter reference for the approximation at the middle of the edge <paramref name="I"/>.</summary>
    internal void SetReference(int I)
    {
        hasref = true;
        if (I == 1)
        {
            valref = abscissa!.Value(1) * 0.5;
        }
        else
        {
            valref = (abscissa!.Value(I) + abscissa.Value(I - 1)) * 0.5;
        }
    }

    /// <summary>The index of the edge at the abscissa <paramref name="W"/>, the next one at a junction when
    /// <paramref name="Forward"/>.</summary>
    public int Index(double W, bool Forward = true)
    {
        int ind, len = abscissa!.Length();
        double par = W, last = abscissa.Value(abscissa.Upper());
        double f = 0.0, l = 0.0, t = Max(tolesp, Precision.Confusion());

        if (IsPeriodic() && Math.Abs(par) >= t && Math.Abs(par - last) >= t)
        {
            par = ElCLib.InPeriod(par, 0.0, last);
        }

        for (ind = 1; ind <= len; ind++)
        {
            f = l;
            l = abscissa.Value(ind);
            if (par < l || ind == len)
            {
                break;
            }
        }

        if (Forward && ind < len && Math.Abs(par - l) < t)
        {
            ind++;
        }
        else if (!Forward && ind > 1 && Math.Abs(par - f) < t)
        {
            ind--;
        }
        else if (Forward && IsPeriodic() && ind == len && Math.Abs(par - l) < t)
        {
            ind = 1;
        }
        else if (!Forward && IsPeriodic() && ind == 1 && Math.Abs(par - f) < t)
        {
            ind = len;
        }

        return ind;
    }

    /// <summary>The index of the edge <paramref name="E"/>, or 0.</summary>
    public int Index(in TopoDS_Edge E)
    {
        for (int IE = 1; IE <= spine.Length(); IE++)
        {
            if (E.IsSame(spine.Value(IE)))
            {
                return IE;
            }
        }

        return 0;
    }

    internal void UnsetReference()
    {
        hasref = false;
    }

    internal void SetErrorStatus(ChFiDS_ErrorStatus state)
    {
        errorstate = state;
    }

    /// <summary>The error status of the spine (renvoie le statut d'erreur concernant la spine).</summary>
    public ChFiDS_ErrorStatus ErrorStatus()
    {
        return errorstate;
    }

    /// <summary>The mode of the chamfers.</summary>
    public ChFiDS_ChamfMode Mode()
    {
        return myMode;
    }

    /// <summary>The tolerance of the spine.</summary>
    public double GetTolesp()
    {
        return tolesp;
    }

    // Moves the abscissa L to its edge Ind (-1 or NbEdges + 1 on the tangent extensions) and makes it local.
    private void Prepare(ref double L, out int Ind)
    {
        double tol = Max(tolesp, Precision.Confusion());
        double last = abscissa!.Value(abscissa.Upper());
        int len = abscissa.Length();
        if (IsPeriodic() && Math.Abs(L) >= tol && Math.Abs(L - last) >= tol)
        {
            L = ElCLib.InPeriod(L, 0.0, last);
        }

        if (hasfirsttgt && (L <= firsttgtpar))
        {
            if (hasref && valref >= L && Math.Abs(L - firsttgtpar) <= tol)
            {
                Ind = Index(L);
            }
            else
            {
                Ind = -1;
                L -= firsttgtpar;
            }
        }
        else if (L <= 0.0)
        {
            Ind = 1;
        }
        else if (haslasttgt && (L >= lasttgtpar))
        {
            if (hasref && valref <= L && Math.Abs(L - lasttgtpar) <= tol)
            {
                Ind = Index(L);
            }
            else
            {
                Ind = len + 1;
                L -= lasttgtpar;
            }
        }
        else if (L >= last)
        {
            Ind = len;
        }
        else
        {
            for (Ind = 1; Ind < len; Ind++)
            {
                if (L < abscissa.Value(Ind))
                {
                    break;
                }
            }

            if (hasref)
            {
                if (L >= valref && Ind != 1)
                {
                    if (Math.Abs(L - abscissa.Value(Ind - 1)) <= Precision.Confusion())
                    {
                        Ind--;
                    }
                }
                else if (L <= valref && Ind != len)
                {
                    if (Math.Abs(L - abscissa.Value(Ind)) <= Precision.Confusion())
                    {
                        Ind++;
                    }
                }
            }
        }

        if (Ind >= 1 && Ind <= len)
        {
            if (spine.Value(Ind).Orientation() == TopAbs_REVERSED)
            {
                L = abscissa.Value(Ind) - L;
            }
            else if (Ind != 1)
            {
                L -= abscissa.Value(Ind - 1);
            }
        }
    }
}
