// Port of: src/ModelingData/TKBRep/BRep/BRep_GCurve.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_GCurve.lxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_GCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The root of the geometric curve representations: a range, a first and a last parameter.</summary>
public abstract class BRep_GCurve : BRep_CurveRepresentation
{
    private double myFirst;
    private double myLast;

    protected BRep_GCurve(in TopLoc_Location L, double First, double Last) : base(L)
    {
        myFirst = First;
        myLast = Last;
    }

    internal void SetRange(double First, double Last)
    {
        CheckMutable();
        myFirst = First;
        myLast = Last;
        Update();
    }

    /// <summary>The range of the curve.</summary>
    public void Range(out double First, out double Last)
    {
        First = myFirst;
        Last = myLast;
    }

    /// <summary>The first parameter.</summary>
    public double First() => myFirst;

    /// <summary>The last parameter.</summary>
    public double Last() => myLast;

    internal void First(double F)
    {
        CheckMutable();
        myFirst = F;
        Update();
    }

    internal void Last(double L)
    {
        CheckMutable();
        myLast = L;
        Update();
    }

    /// <summary>The point at parameter <paramref name="U"/>.</summary>
    public abstract void D0(double U, out gp_Pnt P);

    /// <summary>Recomputes any derived data after a modification; called when the range changes.</summary>
    internal virtual void Update()
    {
    }
}
