// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_Stripe.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_Stripe.lxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_Stripe.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// The data of a band of fillet (a stripe): its spine, its sequence of fillet surfaces, the orientations on its two
/// faces, its "quadrant", and at each end the curve, pcurve and points that join it to its neighbours.
/// </summary>
/// <remarks>
/// A working object of the fillet builders (decision 21): they fill it through the internal mutators and the
/// <c>Change*</c> accessors. The public members only read.
/// </remarks>
public sealed class ChFiDS_Stripe : Standard_Transient
{
    private double pardeb1;
    private double parfin1;
    private double pardeb2;
    private double parfin2;
    private ChFiDS_Spine? mySpine;
    private NCollection_HSequence<ChFiDS_SurfData>? myHdata;
    private Geom2d_Curve? pcrv1;
    private Geom2d_Curve? pcrv2;
    private int myChoix;
    private int indexOfSolid;
    private int indexOfcurve1;
    private int indexOfcurve2;
    private int indexfirstPOnS1;
    private int indexlastPOnS1;
    private int indexfirstPOnS2;
    private int indexlastPOnS2;
    private int begfilled;
    private int endfilled;
    private TopAbs_Orientation myOr1;
    private TopAbs_Orientation myOr2;
    private TopAbs_Orientation orcurv1;
    private TopAbs_Orientation orcurv2;

    public ChFiDS_Stripe()
    {
        pardeb1 = 0.0;
        parfin1 = 0.0;
        pardeb2 = 0.0;
        parfin2 = 0.0;
        myChoix = 0;
        indexOfSolid = 0;
        indexOfcurve1 = 0;
        indexOfcurve2 = 0;
        indexfirstPOnS1 = 0;
        indexlastPOnS1 = 0;
        indexfirstPOnS2 = 0;
        indexlastPOnS2 = 0;
        begfilled = /*false*/ 0; // eap, Apr 29 2002, occ293
        endfilled = /*false*/ 0;
        myOr1 = TopAbs_FORWARD;
        myOr2 = TopAbs_FORWARD;
        orcurv1 = TopAbs_FORWARD;
        orcurv2 = TopAbs_FORWARD;
    }

    /// <summary>Forgets the fillet surfaces and the end pcurves, and resets the spine.</summary>
    internal void Reset()
    {
        myHdata = null;
        orcurv1 = orcurv2 = TopAbs_FORWARD;
        pcrv1 = null;
        pcrv1 = null; // OCCT: pcrv1 twice; pcrv2 is kept (decision 9604)
        mySpine!.Reset();
    }

    /// <summary>The fillet surfaces of the stripe, or null.</summary>
    public NCollection_HSequence<ChFiDS_SurfData>? SetOfSurfData()
    {
        return myHdata;
    }

    public ChFiDS_Spine? Spine()
    {
        return mySpine;
    }

    public TopAbs_Orientation OrientationOnFace1()
    {
        return myOr1;
    }

    public TopAbs_Orientation OrientationOnFace2()
    {
        return myOr2;
    }

    /// <summary>The "quadrant" of the fillet.</summary>
    public int Choix()
    {
        return myChoix;
    }

    internal ref NCollection_HSequence<ChFiDS_SurfData>? ChangeSetOfSurfData() => ref myHdata;

    internal ref ChFiDS_Spine? ChangeSpine() => ref mySpine;

    internal void OrientationOnFace1(TopAbs_Orientation Or1)
    {
        myOr1 = Or1;
    }

    internal void OrientationOnFace2(TopAbs_Orientation Or2)
    {
        myOr2 = Or2;
    }

    internal void Choix(int C)
    {
        myChoix = C;
    }

    public void FirstParameters(out double Pdeb, out double Pfin)
    {
        Pdeb = pardeb1;
        Pfin = parfin1;
    }

    public void LastParameters(out double Pdeb, out double Pfin)
    {
        Pdeb = pardeb2;
        Pfin = parfin2;
    }

    internal void ChangeFirstParameters(double Pdeb, double Pfin)
    {
        pardeb1 = Pdeb;
        parfin1 = Pfin;
    }

    internal void ChangeLastParameters(double Pdeb, double Pfin)
    {
        pardeb2 = Pdeb;
        parfin2 = Pfin;
    }

    public int FirstCurve()
    {
        return indexOfcurve1;
    }

    public int LastCurve()
    {
        return indexOfcurve2;
    }

    internal void ChangeFirstCurve(int Index)
    {
        indexOfcurve1 = Index;
    }

    internal void ChangeLastCurve(int Index)
    {
        indexOfcurve2 = Index;
    }

    public Geom2d_Curve? FirstPCurve()
    {
        return pcrv1;
    }

    public Geom2d_Curve? LastPCurve()
    {
        return pcrv2;
    }

    internal ref Geom2d_Curve? ChangeFirstPCurve() => ref pcrv1;

    internal ref Geom2d_Curve? ChangeLastPCurve() => ref pcrv2;

    public TopAbs_Orientation FirstPCurveOrientation()
    {
        return orcurv1;
    }

    public TopAbs_Orientation LastPCurveOrientation()
    {
        return orcurv2;
    }

    internal void FirstPCurveOrientation(TopAbs_Orientation O)
    {
        orcurv1 = O;
    }

    internal void LastPCurveOrientation(TopAbs_Orientation O)
    {
        orcurv2 = O;
    }

    public int IndexFirstPointOnS1()
    {
        return indexfirstPOnS1;
    }

    public int IndexFirstPointOnS2()
    {
        return indexfirstPOnS2;
    }

    public int IndexLastPointOnS1()
    {
        return indexlastPOnS1;
    }

    public int IndexLastPointOnS2()
    {
        return indexlastPOnS2;
    }

    internal void ChangeIndexFirstPointOnS1(int Index)
    {
        indexfirstPOnS1 = Index;
    }

    internal void ChangeIndexFirstPointOnS2(int Index)
    {
        indexfirstPOnS2 = Index;
    }

    internal void ChangeIndexLastPointOnS1(int Index)
    {
        indexlastPOnS1 = Index;
    }

    internal void ChangeIndexLastPointOnS2(int Index)
    {
        indexlastPOnS2 = Index;
    }

    public void Parameters(bool First, out double Pdeb, out double Pfin)
    {
        if (First)
        {
            Pdeb = pardeb1;
            Pfin = parfin1;
        }
        else
        {
            Pdeb = pardeb2;
            Pfin = parfin2;
        }
    }

    internal void SetParameters(bool First, double Pdeb, double Pfin)
    {
        if (First)
        {
            pardeb1 = Pdeb;
            parfin1 = Pfin;
        }
        else
        {
            pardeb2 = Pdeb;
            parfin2 = Pfin;
        }
    }

    public int Curve(bool First)
    {
        if (First)
        {
            return indexOfcurve1;
        }
        else
        {
            return indexOfcurve2;
        }
    }

    internal void SetCurve(int Index, bool First)
    {
        if (First)
        {
            indexOfcurve1 = Index;
        }
        else
        {
            indexOfcurve2 = Index;
        }
    }

    public Geom2d_Curve? PCurve(bool First)
    {
        if (First)
        {
            return pcrv1;
        }
        else
        {
            return pcrv2;
        }
    }

    internal ref Geom2d_Curve? ChangePCurve(bool First)
    {
        if (First)
        {
            return ref pcrv1;
        }
        else
        {
            return ref pcrv2;
        }
    }

    /// <summary>The orientation on face <paramref name="OnS"/> (1 or 2).</summary>
    public TopAbs_Orientation Orientation(int OnS)
    {
        if (OnS == 1)
        {
            return myOr1;
        }
        else
        {
            return myOr2;
        }
    }

    internal void SetOrientation(TopAbs_Orientation Or, int OnS)
    {
        if (OnS == 1)
        {
            myOr1 = Or;
        }
        else
        {
            myOr2 = Or;
        }
    }

    /// <summary>The orientation of the first or last pcurve.</summary>
    public TopAbs_Orientation Orientation(bool First)
    {
        if (First)
        {
            return orcurv1;
        }
        else
        {
            return orcurv2;
        }
    }

    internal void SetOrientation(TopAbs_Orientation Or, bool First)
    {
        if (First)
        {
            orcurv1 = Or;
        }
        else
        {
            orcurv2 = Or;
        }
    }

    public int IndexPoint(bool First, int OnS)
    {
        if (First)
        {
            if (OnS == 1)
            {
                return indexfirstPOnS1;
            }
            else
            {
                return indexfirstPOnS2;
            }
        }
        else
        {
            if (OnS == 1)
            {
                return indexlastPOnS1;
            }
            else
            {
                return indexlastPOnS2;
            }
        }
    }

    internal void SetIndexPoint(int Index, bool First, int OnS)
    {
        if (First)
        {
            if (OnS == 1)
            {
                indexfirstPOnS1 = Index;
            }
            else
            {
                indexfirstPOnS2 = Index;
            }
        }
        else
        {
            if (OnS == 1)
            {
                indexlastPOnS1 = Index;
            }
            else
            {
                indexlastPOnS2 = Index;
            }
        }
    }

    public int SolidIndex()
    {
        return indexOfSolid;
    }

    internal void SetSolidIndex(int Index)
    {
        indexOfSolid = Index;
    }

    /// <summary>Marks the first or last end as filled into the data structure, <paramref name="Nb"/> times.</summary>
    internal void InDS(bool First, int Nb = 1) // eap, Apr 29 2002, occ293
    {
        if (First)
        {
            begfilled = /*true*/ Nb;
        }
        else
        {
            endfilled = /*true*/ Nb;
        }
    }

    public int IsInDS(bool First)
    {
        if (First)
        {
            return begfilled;
        }
        else
        {
            return endfilled;
        }
    }
}
