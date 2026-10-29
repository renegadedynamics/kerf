// Port of: src/ModelingData/TKBRep/BRepAdaptor/BRepAdaptor_CompCurve.hxx
// Port of: src/ModelingData/TKBRep/BRepAdaptor/BRepAdaptor_CompCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// A wire of the BRep topology seen as a 3D curve: its non-degenerated edges in connection order
/// (<see cref="BRepTools_WireExplorer"/>), each over a unit of parameter, or over its length with
/// <c>KnotByCurvilinearAbcissa</c>.
/// </summary>
/// <remarks>
/// C0 and C1 continuity are not assumed between edges. The curve is never periodic, even for a single periodic
/// edge. The wire must be valid: a chain of connected edges. A working object (decision 214).
/// </remarks>
public class BRepAdaptor_CompCurve : Adaptor3d_Curve
{
    private TopoDS_Wire myWire = new();
    private double TFirst;
    private double TLast;
    private double PTol;
    private NCollection_HArray1<BRepAdaptor_Curve>? myCurves;
    private NCollection_HArray1<double>? myKnots;
    private int CurIndex;
    private bool Forward;
    private bool IsbyAC;

    /// <summary>An undefined curve, with no wire loaded.</summary>
    public BRepAdaptor_CompCurve()
    {
        TFirst = 0.0;
        TLast = 0.0;
        PTol = 0.0;
        CurIndex = -1;
        Forward = false;
        IsbyAC = false;
    }

    /// <summary>The wire, its edges parametrised by unit parameters or by length (<paramref name="theIsAC"/>).</summary>
    public BRepAdaptor_CompCurve(in TopoDS_Wire theWire, bool theIsAC = false)
    {
        myWire = theWire;
        TFirst = 0.0;
        TLast = 0.0;
        PTol = 0.0;
        CurIndex = -1;
        Forward = false;
        IsbyAC = theIsAC;
        Initialize(theWire, theIsAC);
    }

    /// <summary>The wire trimmed to [theFirst, theLast].</summary>
    public BRepAdaptor_CompCurve(in TopoDS_Wire theWire, bool theIsAC, double theFirst, double theLast, double theTolerance)
    {
        myWire = theWire;
        TFirst = theFirst;
        TLast = theLast;
        PTol = theTolerance;
        CurIndex = -1;
        Forward = false;
        IsbyAC = theIsAC;
        Initialize(theWire, theIsAC, theFirst, theLast, theTolerance);
    }

    /// <summary>A shallow copy: shallow copies of the edges' adaptors, the knots shared.</summary>
    public override Adaptor3d_Curve ShallowCopy()
    {
        BRepAdaptor_CompCurve aCopy = new BRepAdaptor_CompCurve();

        aCopy.myWire = myWire;
        aCopy.TFirst = TFirst;
        aCopy.TLast = TLast;
        aCopy.PTol = PTol;
        aCopy.myCurves = new NCollection_HArray1<BRepAdaptor_Curve>(1, myCurves!.Length());
        for (int anI = 1; anI <= myCurves.Length(); ++anI)
        {
            Adaptor3d_Curve aCurve = myCurves.Value(anI).ShallowCopy();
            BRepAdaptor_Curve aBrepCurve = (BRepAdaptor_Curve)aCurve;
            aCopy.myCurves.SetValue(anI, aBrepCurve);
        }
        aCopy.myKnots = myKnots;
        aCopy.CurIndex = CurIndex;
        aCopy.Forward = Forward;
        aCopy.IsbyAC = IsbyAC;

        return aCopy;
    }

    /// <summary>Loads the wire; AC parametrises the edges by their lengths.</summary>
    public void Initialize(in TopoDS_Wire W, bool AC)
    {
        int ii, NbEdge;
        var wexp = new BRepTools_WireExplorer();
        TopoDS_Edge E;

        myWire = W;
        PTol = 0.0;
        IsbyAC = AC;

        for (NbEdge = 0, wexp.Init(myWire); wexp.More(); wexp.Next())
        {
            if (!BRep_Tool.Degenerated(wexp.Current()))
            {
                NbEdge++;
            }
        }

        if (NbEdge == 0)
        {
            return;
        }

        CurIndex = (NbEdge + 1) / 2;
        myCurves = new NCollection_HArray1<BRepAdaptor_Curve>(1, NbEdge);
        // OCCT's array holds adaptors by value, default-constructed
        for (int i = 1; i <= NbEdge; i++)
        {
            myCurves.SetValue(i, new BRepAdaptor_Curve());
        }
        myKnots = new NCollection_HArray1<double>(1, NbEdge + 1);
        myKnots.SetValue(1, 0.0);

        for (ii = 0, wexp.Init(myWire); wexp.More(); wexp.Next())
        {
            E = wexp.Current();
            if (!BRep_Tool.Degenerated(E))
            {
                ii++;
                myCurves.ChangeValue(ii).Initialize(E);
                if (AC)
                {
                    myKnots.SetValue(ii + 1, myKnots.Value(ii));
                    myKnots.ChangeValue(ii + 1) += GCPnts_AbscissaPoint.Length(myCurves.ChangeValue(ii));
                }
                else
                {
                    myKnots.SetValue(ii + 1, (double)ii);
                }
            }
        }

        Forward = true; // Default ; The Reverse Edges are parsed.
        if ((NbEdge > 2) || ((NbEdge == 2) && (!myWire.Closed())))
        {
            TopAbs_Orientation Or = myCurves.Value(1).Edge().Orientation();
            TopoDS_Vertex VI = new TopoDS_Vertex(), VL;
            TopExp.CommonVertex(myCurves.Value(1).Edge(), myCurves.Value(2).Edge(), ref VI);
            VL = TopExp.LastVertex(myCurves.Value(1).Edge());
            if (VI.IsSame(VL))
            { // The direction of parsing is always preserved
                if (Or == TopAbs_REVERSED)
                {
                    Forward = false;
                }
            }
            else
            { // The direction of parsing is always reversed
                if (Or != TopAbs_REVERSED)
                {
                    Forward = false;
                }
            }
        }

        TFirst = 0;
        TLast = myKnots.Value(myKnots.Length());
    }

    /// <summary>Loads the wire, trimmed to [First, Last] with the tolerance Tol.</summary>
    public void Initialize(in TopoDS_Wire W, bool AC, double First, double Last, double Tol)
    {
        Initialize(W, AC);
        TFirst = First;
        TLast = Last;
        PTol = Tol;

        // Trim the extremal curves.
        BRepAdaptor_Curve HC;
        int i1, i2;
        double f = TFirst, l = TLast, d;
        i1 = i2 = CurIndex;
        Prepare(ref f, out d, ref i1);
        Prepare(ref l, out d, ref i2);
        CurIndex = (i1 + i2) / 2; // Small optimization
        if (i1 == i2)
        {
            if (l > f)
            {
                HC = (BRepAdaptor_Curve)myCurves!.Value(i1).Trim(f, l, PTol);
            }
            else
            {
                HC = (BRepAdaptor_Curve)myCurves!.Value(i1).Trim(l, f, PTol);
            }
            myCurves.SetValue(i1, HC);
        }
        else
        {
            BRepAdaptor_Curve c1 = myCurves!.Value(i1);
            BRepAdaptor_Curve c2 = myCurves.Value(i2);
            double k;

            k = c1.LastParameter();
            if (k > f)
            {
                HC = (BRepAdaptor_Curve)c1.Trim(f, k, PTol);
            }
            else
            {
                HC = (BRepAdaptor_Curve)c1.Trim(k, f, PTol);
            }
            myCurves.SetValue(i1, HC);

            k = c2.FirstParameter();
            if (k <= l)
            {
                HC = (BRepAdaptor_Curve)c2.Trim(k, l, PTol);
            }
            else
            {
                HC = (BRepAdaptor_Curve)c2.Trim(l, k, PTol);
            }
            myCurves.SetValue(i2, HC);
        }
    }

    /// <summary>The wire.</summary>
    public TopoDS_Wire Wire() => myWire;

    /// <summary>The edge holding the parameter U, and the parameter on it.</summary>
    public void Edge(double U, out TopoDS_Edge E, out double UonE)
    {
        int index = CurIndex;
        UonE = U;
        Prepare(ref UonE, out _, ref index);
        E = myCurves!.Value(index).Edge();
    }

    public override double FirstParameter() => TFirst;

    public override double LastParameter() => TLast;

    public override GeomAbs_Shape Continuity()
    {
        if (myCurves!.Length() > 1)
        {
            return GeomAbs_C0;
        }
        return myCurves.Value(1).Continuity();
    }

    public override int NbIntervals(GeomAbs_Shape S)
    {
        int NbInt, ii;
        for (ii = 1, NbInt = 0; ii <= myCurves!.Length(); ii++)
        {
            NbInt += myCurves.ChangeValue(ii).NbIntervals(S);
        }

        return NbInt;
    }

    public override void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        int ii, jj, kk, n;
        double f, F, delta;

        // First curve (direction of parsing of the edge)
        n = myCurves!.ChangeValue(1).NbIntervals(S);
        NCollection_HArray1<double> Ti = new NCollection_HArray1<double>(1, n + 1);
        myCurves.ChangeValue(1).Intervals(Ti.ChangeArray1(), S);
        InvPrepare(1, out f, out delta);
        F = myKnots!.Value(1);
        if (delta < 0)
        {
            // invert the direction of parsing
            for (kk = 1, jj = Ti.Length(); jj > 0; kk++, jj--)
            {
                T[kk] = F + (Ti.Value(jj) - f) * delta;
            }
        }
        else
        {
            for (kk = 1; kk <= Ti.Length(); kk++)
            {
                T[kk] = F + (Ti.Value(kk) - f) * delta;
            }
        }

        // and the next
        for (ii = 2; ii <= myCurves.Length(); ii++)
        {
            n = myCurves.ChangeValue(ii).NbIntervals(S);
            if (n != Ti.Length() - 1)
            {
                Ti = new NCollection_HArray1<double>(1, n + 1);
            }
            myCurves.ChangeValue(ii).Intervals(Ti.ChangeArray1(), S);
            InvPrepare(ii, out f, out delta);
            F = myKnots.Value(ii);
            if (delta < 0)
            {
                // invert the direction of parcing
                for (jj = Ti.Length() - 1; jj > 0; kk++, jj--)
                {
                    T[kk] = F + (Ti.Value(jj) - f) * delta;
                }
            }
            else
            {
                for (jj = 2; jj <= Ti.Length(); kk++, jj++)
                {
                    T[kk] = F + (Ti.Value(jj) - f) * delta;
                }
            }
        }
    }

    /// <summary>The wire trimmed to [First, Last], a new adaptor.</summary>
    public override Adaptor3d_Curve Trim(double First, double Last, double Tol)
    {
        BRepAdaptor_CompCurve HC = new BRepAdaptor_CompCurve(myWire, IsbyAC, First, Last, Tol);
        return HC;
    }

    public override bool IsClosed() => myWire.Closed();

    public override bool IsPeriodic() => false;

    public override double Period() => (TLast - TFirst);

    public sealed override gp_Pnt EvalD0(double theU)
    {
        double u = theU;
        int index = CurIndex;
        Prepare(ref u, out _, ref index);
        return myCurves!.Value(index).EvalD0(u);
    }

    public sealed override Geom_Curve.ResD1 EvalD1(double theU)
    {
        double u = theU;
        int index = CurIndex;
        Prepare(ref u, out double d, ref index);
        Geom_Curve.ResD1 aRes = myCurves!.Value(index).EvalD1(u);
        aRes.D1 *= d;
        return aRes;
    }

    public sealed override Geom_Curve.ResD2 EvalD2(double theU)
    {
        double u = theU;
        int index = CurIndex;
        Prepare(ref u, out double d, ref index);
        Geom_Curve.ResD2 aRes = myCurves!.Value(index).EvalD2(u);
        aRes.D1 *= d;
        aRes.D2 *= d * d;
        return aRes;
    }

    public sealed override Geom_Curve.ResD3 EvalD3(double theU)
    {
        double u = theU;
        int index = CurIndex;
        Prepare(ref u, out double d, ref index);
        Geom_Curve.ResD3 aRes = myCurves!.Value(index).EvalD3(u);
        aRes.D1 *= d;
        aRes.D2 *= d * d;
        aRes.D3 *= d * d * d;
        return aRes;
    }

    public sealed override gp_Vec EvalDN(double theU, int theN)
    {
        double u = theU;
        int index = CurIndex;
        Prepare(ref u, out double d, ref index);
        return myCurves!.Value(index).EvalDN(u, theN) * Math.Pow(d, theN);
    }

    /// <summary>The smallest parametric resolution of the edges.</summary>
    public override double Resolution(double R3d)
    {
        double Res = 1.0e200, r;
        int ii, L = myCurves!.Length();
        for (ii = 1; ii <= L; ii++)
        {
            r = myCurves.Value(ii).Resolution(R3d);
            if (r < Res)
            {
                Res = r;
            }
        }
        return Res;
    }

    /// <summary>Always GeomAbs_OtherCurve.</summary>
    public override GeomAbs_CurveType GetType()
    {
        return GeomAbs_OtherCurve; // temporary
        //  if ( myCurves->Length() > 1) return GeomAbs_OtherCurve;
        //  return myCurves->Value(1).GetType();
    }

    public override gp_Lin Line() => myCurves!.Value(1).Line();

    public override gp_Circ Circle() => myCurves!.Value(1).Circle();

    public override gp_Elips Ellipse() => myCurves!.Value(1).Ellipse();

    public override gp_Hypr Hyperbola() => myCurves!.Value(1).Hyperbola();

    public override gp_Parab Parabola() => myCurves!.Value(1).Parabola();

    public override int Degree() => myCurves!.Value(1).Degree();

    public override bool IsRational() => myCurves!.Value(1).IsRational();

    public override int NbPoles() => myCurves!.Value(1).NbPoles();

    public override int NbKnots() => myCurves!.Value(1).NbKnots();

    public override Geom_BezierCurve Bezier() => myCurves!.Value(1).Bezier();

    public override Geom_BSplineCurve BSpline() => myCurves!.Value(1).BSpline();

    // When the parameter is close to "node" the rule is determined
    // depending on the sign of tol:
    //   - negative -> Rule preceding to the node.
    //   - positive -> Rule following after the node.
    private void Prepare(ref double W, out double Delta, ref int theCurIndex)
    {
        double f, l, Wtest, Eps;
        int ii;
        if (W - TFirst < TLast - W)
        {
            Eps = PTol;
        }
        else
        {
            Eps = -PTol;
        }

        Wtest = W + Eps; // Offset to discriminate the nodes

        // Find the index
        bool Trouve = false;
        if (myKnots!.Value(theCurIndex) > Wtest)
        {
            for (ii = theCurIndex - 1; ii > 0 && !Trouve; ii--)
            {
                if (myKnots.Value(ii) <= Wtest)
                {
                    theCurIndex = ii;
                    Trouve = true;
                }
            }
            if (!Trouve)
            {
                theCurIndex = 1; // Out of limits...
            }
        }

        else if (myKnots.Value(theCurIndex + 1) <= Wtest)
        {
            for (ii = theCurIndex + 1; ii <= myCurves!.Length() && !Trouve; ii++)
            {
                if (myKnots.Value(ii + 1) > Wtest)
                {
                    theCurIndex = ii;
                    Trouve = true;
                }
            }
            if (!Trouve)
            {
                theCurIndex = myCurves.Length(); // Out of limits...
            }
        }

        // Invert ?
        TopoDS_Edge E = myCurves!.Value(theCurIndex).Edge();
        TopAbs_Orientation Or = E.Orientation();
        bool Reverse;
        Reverse = (Forward && (Or == TopAbs_REVERSED)) || (!Forward && (Or != TopAbs_REVERSED));

        // Calculate the local parameter
        BRep_Tool.Range(E, out f, out l);
        Delta = myKnots.Value(theCurIndex + 1) - myKnots.Value(theCurIndex);
        if (Delta > PTol * 1.0e-9)
        {
            Delta = (l - f) / Delta;
        }

        if (Reverse)
        {
            Delta *= -1;
            W = l + (W - myKnots.Value(theCurIndex)) * Delta;
        }
        else
        {
            W = f + (W - myKnots.Value(theCurIndex)) * Delta;
        }
    }

    private void InvPrepare(int index, out double First, out double Delta)
    {
        // Invert?
        TopoDS_Edge E = myCurves!.Value(index).Edge();
        TopAbs_Orientation Or = E.Orientation();
        bool Reverse;
        Reverse = (Forward && (Or == TopAbs_REVERSED)) || (!Forward && (Or != TopAbs_REVERSED));

        // Calculate the parameters of reparametrisation
        // such as : T = Ti + (t-First)*Delta
        BRep_Tool.Range(E, out double f, out double l);
        Delta = myKnots!.Value(index + 1) - myKnots.Value(index);
        if (l - f > PTol * 1.0e-9)
        {
            Delta /= (l - f);
        }

        if (Reverse)
        {
            Delta *= -1;
            First = l;
        }
        else
        {
            First = f;
        }
    }
}
