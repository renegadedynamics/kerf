// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_Stripe.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_Stripe.lxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_Stripe.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets.Data;

/// <summary>
/// The data of a band of fillet (a stripe): its spine, its sequence of fillet surfaces, the orientations on its two
/// faces, its "quadrant", and at each end the curve, pcurve and points that join it to its neighbours.
/// </summary>
/// <remarks>
/// A working object of the fillet builders (decision 21): they fill it through the internal mutators and the
/// <c>Change*</c> accessors. The public members only read.
/// </remarks>
internal sealed class Stripe : Transient
{
    private double _pardeb1;
    private double _parfin1;
    private double _pardeb2;
    private double _parfin2;
    private Spine? _spine;
    private NCollection_HSequence<SurfData>? _hdata;
    private Curve2D? _pcrv1;
    private Curve2D? _pcrv2;
    private int _choix;
    private int _indexOfSolid;
    private int _indexOfcurve1;
    private int _indexOfcurve2;
    private int _indexfirstPOnS1;
    private int _indexlastPOnS1;
    private int _indexfirstPOnS2;
    private int _indexlastPOnS2;
    private int _begfilled;
    private int _endfilled;
    private Orientation _or1;
    private Orientation _or2;
    private Orientation _orcurv1;
    private Orientation _orcurv2;

    public Stripe()
    {
        _pardeb1 = 0.0;
        _parfin1 = 0.0;
        _pardeb2 = 0.0;
        _parfin2 = 0.0;
        _choix = 0;
        _indexOfSolid = 0;
        _indexOfcurve1 = 0;
        _indexOfcurve2 = 0;
        _indexfirstPOnS1 = 0;
        _indexlastPOnS1 = 0;
        _indexfirstPOnS2 = 0;
        _indexlastPOnS2 = 0;
        _begfilled = /*false*/ 0; // eap, Apr 29 2002, occ293
        _endfilled = /*false*/ 0;
        _or1 = Forward;
        _or2 = Forward;
        _orcurv1 = Forward;
        _orcurv2 = Forward;
    }

    /// <summary>Forgets the fillet surfaces and the end pcurves, and resets the spine.</summary>
    internal void Reset()
    {
        _hdata = null;
        _orcurv1 = _orcurv2 = Forward;
        _pcrv1 = null;
        _pcrv1 = null; // OCCT: pcrv1 twice; pcrv2 is kept (decision 1040)
        _spine!.Reset();
    }

    /// <summary>The fillet surfaces of the stripe, or null.</summary>
    public NCollection_HSequence<SurfData>? SetOfSurfData()
    {
        return _hdata;
    }

    public Spine? Spine()
    {
        return _spine;
    }

    public Orientation OrientationOnFace1()
    {
        return _or1;
    }

    public Orientation OrientationOnFace2()
    {
        return _or2;
    }

    /// <summary>The "quadrant" of the fillet.</summary>
    public int Choix()
    {
        return _choix;
    }

    internal ref NCollection_HSequence<SurfData>? ChangeSetOfSurfData() => ref _hdata;

    internal ref Spine? ChangeSpine() => ref _spine;

    internal void OrientationOnFace1(Orientation or1)
    {
        _or1 = or1;
    }

    internal void OrientationOnFace2(Orientation or2)
    {
        _or2 = or2;
    }

    internal void Choix(int c)
    {
        _choix = c;
    }

    public void FirstParameters(out double pdeb, out double pfin)
    {
        pdeb = _pardeb1;
        pfin = _parfin1;
    }

    public void LastParameters(out double pdeb, out double pfin)
    {
        pdeb = _pardeb2;
        pfin = _parfin2;
    }

    internal void ChangeFirstParameters(double pdeb, double pfin)
    {
        _pardeb1 = pdeb;
        _parfin1 = pfin;
    }

    internal void ChangeLastParameters(double pdeb, double pfin)
    {
        _pardeb2 = pdeb;
        _parfin2 = pfin;
    }

    public int FirstCurve()
    {
        return _indexOfcurve1;
    }

    public int LastCurve()
    {
        return _indexOfcurve2;
    }

    internal void ChangeFirstCurve(int index)
    {
        _indexOfcurve1 = index;
    }

    internal void ChangeLastCurve(int index)
    {
        _indexOfcurve2 = index;
    }

    public Curve2D? FirstPCurve()
    {
        return _pcrv1;
    }

    public Curve2D? LastPCurve()
    {
        return _pcrv2;
    }

    internal ref Curve2D? ChangeFirstPCurve() => ref _pcrv1;

    internal ref Curve2D? ChangeLastPCurve() => ref _pcrv2;

    public Orientation FirstPCurveOrientation()
    {
        return _orcurv1;
    }

    public Orientation LastPCurveOrientation()
    {
        return _orcurv2;
    }

    internal void FirstPCurveOrientation(Orientation o)
    {
        _orcurv1 = o;
    }

    internal void LastPCurveOrientation(Orientation o)
    {
        _orcurv2 = o;
    }

    public int IndexFirstPointOnS1()
    {
        return _indexfirstPOnS1;
    }

    public int IndexFirstPointOnS2()
    {
        return _indexfirstPOnS2;
    }

    public int IndexLastPointOnS1()
    {
        return _indexlastPOnS1;
    }

    public int IndexLastPointOnS2()
    {
        return _indexlastPOnS2;
    }

    internal void ChangeIndexFirstPointOnS1(int index)
    {
        _indexfirstPOnS1 = index;
    }

    internal void ChangeIndexFirstPointOnS2(int index)
    {
        _indexfirstPOnS2 = index;
    }

    internal void ChangeIndexLastPointOnS1(int index)
    {
        _indexlastPOnS1 = index;
    }

    internal void ChangeIndexLastPointOnS2(int index)
    {
        _indexlastPOnS2 = index;
    }

    public void Parameters(bool first, out double pdeb, out double pfin)
    {
        if (first)
        {
            pdeb = _pardeb1;
            pfin = _parfin1;
        }
        else
        {
            pdeb = _pardeb2;
            pfin = _parfin2;
        }
    }

    internal void SetParameters(bool first, double pdeb, double pfin)
    {
        if (first)
        {
            _pardeb1 = pdeb;
            _parfin1 = pfin;
        }
        else
        {
            _pardeb2 = pdeb;
            _parfin2 = pfin;
        }
    }

    public int Curve(bool first)
    {
        if (first)
        {
            return _indexOfcurve1;
        }
        else
        {
            return _indexOfcurve2;
        }
    }

    internal void SetCurve(int index, bool first)
    {
        if (first)
        {
            _indexOfcurve1 = index;
        }
        else
        {
            _indexOfcurve2 = index;
        }
    }

    public Curve2D? PCurve(bool first)
    {
        if (first)
        {
            return _pcrv1;
        }
        else
        {
            return _pcrv2;
        }
    }

    internal ref Curve2D? ChangePCurve(bool first)
    {
        if (first)
        {
            return ref _pcrv1;
        }
        else
        {
            return ref _pcrv2;
        }
    }

    /// <summary>The orientation on face <paramref name="onS"/> (1 or 2).</summary>
    public Orientation Orientation(int onS)
    {
        if (onS == 1)
        {
            return _or1;
        }
        else
        {
            return _or2;
        }
    }

    internal void SetOrientation(Orientation or, int onS)
    {
        if (onS == 1)
        {
            _or1 = or;
        }
        else
        {
            _or2 = or;
        }
    }

    /// <summary>The orientation of the first or last pcurve.</summary>
    public Orientation Orientation(bool first)
    {
        if (first)
        {
            return _orcurv1;
        }
        else
        {
            return _orcurv2;
        }
    }

    internal void SetOrientation(Orientation or, bool first)
    {
        if (first)
        {
            _orcurv1 = or;
        }
        else
        {
            _orcurv2 = or;
        }
    }

    public int IndexPoint(bool first, int onS)
    {
        if (first)
        {
            if (onS == 1)
            {
                return _indexfirstPOnS1;
            }
            else
            {
                return _indexfirstPOnS2;
            }
        }
        else
        {
            if (onS == 1)
            {
                return _indexlastPOnS1;
            }
            else
            {
                return _indexlastPOnS2;
            }
        }
    }

    internal void SetIndexPoint(int index, bool first, int onS)
    {
        if (first)
        {
            if (onS == 1)
            {
                _indexfirstPOnS1 = index;
            }
            else
            {
                _indexfirstPOnS2 = index;
            }
        }
        else
        {
            if (onS == 1)
            {
                _indexlastPOnS1 = index;
            }
            else
            {
                _indexlastPOnS2 = index;
            }
        }
    }

    public int SolidIndex()
    {
        return _indexOfSolid;
    }

    internal void SetSolidIndex(int index)
    {
        _indexOfSolid = index;
    }

    /// <summary>Marks the first or last end as filled into the data structure, <paramref name="nb"/> times.</summary>
    internal void InDS(bool first, int nb = 1) // eap, Apr 29 2002, occ293
    {
        if (first)
        {
            _begfilled = /*true*/ nb;
        }
        else
        {
            _endfilled = /*true*/ nb;
        }
    }

    public int IsInDS(bool first)
    {
        if (first)
        {
            return _begfilled;
        }
        else
        {
            return _endfilled;
        }
    }
}
