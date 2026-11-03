// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_CutCurve.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_CutCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Properties;
using Kerf.Geometry;

namespace Occt;

/// <summary>Cuts a curve at its extrema of curvature and its inflections, giving a trimmed curve per interval.
/// </summary>
/// <remarks>OCCT declares <c>Perform(C, aSide)</c> and <c>PerformInf(C)</c> but never defines them; they are left
/// out.</remarks>
public sealed class MAT2d_CutCurve
{
    private readonly NCollection_Sequence<Curve2D> theCurves = new();

    public MAT2d_CutCurve()
    {
    }

    public MAT2d_CutCurve(Curve2D C)
    {
        Perform(C);
    }

    /// <summary>Cuts C at the extrema of curvature and at the inflections.</summary>
    public void Perform(Curve2D C)
    {
        theCurves.Clear();

        CurAndInf2d Sommets = new();
        TrimmedCurve2D TrimC;
        double UF, UL, UC;
        Point2D PF, PL, PC;
        double PTol = global::Kerf.Engine.Foundation.Precision.PConfusion() * 10;
        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion() * 10;
        bool YaCut = false;
        Sommets.Perform(C);

        if (Sommets.IsDone && !Sommets.IsEmpty)
        {
            UF = C.FirstParameter();
            UL = C.LastParameter();
            PF = C.Value(UF);
            PL = C.Value(UL);

            for (int i = 1; i <= Sommets.PointCount; i++)
            {
                UC = Sommets.Parameter(i);

                PC = C.Value(UC);
                if (UC - UF > PTol && PC.Distance(PF) > Tol)
                {
                    if (UL - UC < PTol || PL.Distance(PC) < Tol)
                    {
                        break;
                    }
                    TrimC = new TrimmedCurve2D(C, UF, UC);
                    theCurves.Append(TrimC);
                    UF = UC;
                    PF = PC;
                    YaCut = true;
                }
            }
            if (YaCut)
            {
                TrimC = new TrimmedCurve2D(C, UF, UL);
                theCurves.Append(TrimC);
            }
        }
    }

    /// <summary>True if the curve is not cut.</summary>
    public bool UnModified() => theCurves.IsEmpty();

    /// <summary>The number of curves (at least 2); throws Standard_OutOfRange if the curve is not cut.</summary>
    public int NbCurves()
    {
        if (UnModified())
        {
            throw new OutOfRangeException();
        }
        return theCurves.Length();
    }

    /// <summary>The curve of index Index.</summary>
    public TrimmedCurve2D Value(int Index)
    {
        if (UnModified())
        {
            throw new OutOfRangeException();
        }
        if (Index < 1 || Index > theCurves.Length())
        {
            throw new OutOfRangeException();
        }
        return (TrimmedCurve2D)theCurves.Value(Index);
    }
}
