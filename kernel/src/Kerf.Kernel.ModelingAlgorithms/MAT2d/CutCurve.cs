// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_CutCurve.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_CutCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Properties;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Offsets.MedialAxis;

/// <summary>Cuts a curve at its extrema of curvature and its inflections, giving a trimmed curve per interval.
/// </summary>
/// <remarks>OCCT declares <c>Perform(C, aSide)</c> and <c>PerformInf(C)</c> but never defines them; they are left
/// out.</remarks>
internal sealed class CutCurve
{
    private readonly NCollection_Sequence<Curve2D> _curves = new();

    public CutCurve()
    {
    }

    public CutCurve(Curve2D c)
    {
        Perform(c);
    }

    /// <summary>Cuts C at the extrema of curvature and at the inflections.</summary>
    public void Perform(Curve2D c)
    {
        _curves.Clear();

        CurAndInf2d sommets = new();
        TrimmedCurve2D trimC;
        double uf, ul, uc;
        Point2D pf, pl, pc;
        double pTol = global::Kerf.Engine.Foundation.Precision.PConfusion() * 10;
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion() * 10;
        bool yaCut = false;
        sommets.Perform(c);

        if (sommets.IsDone && !sommets.IsEmpty)
        {
            uf = c.FirstParameter;
            ul = c.LastParameter;
            pf = c.Value(uf);
            pl = c.Value(ul);

            for (int i = 1; i <= sommets.PointCount; i++)
            {
                uc = sommets.Parameter(i);

                pc = c.Value(uc);
                if (uc - uf > pTol && pc.DistanceTo(pf) > tol)
                {
                    if (ul - uc < pTol || pl.DistanceTo(pc) < tol)
                    {
                        break;
                    }
                    trimC = new TrimmedCurve2D(c, uf, uc);
                    _curves.Append(trimC);
                    uf = uc;
                    pf = pc;
                    yaCut = true;
                }
            }
            if (yaCut)
            {
                trimC = new TrimmedCurve2D(c, uf, ul);
                _curves.Append(trimC);
            }
        }
    }

    /// <summary>True if the curve is not cut.</summary>
    public bool UnModified() => _curves.IsEmpty();

    /// <summary>The number of curves (at least 2); throws Standard_OutOfRange if the curve is not cut.</summary>
    public int CurveCount
    {
        get
        {
            if (UnModified())
            {
                throw new OutOfRangeException();
            }
            return _curves.Length();
        }
    }

    /// <summary>The curve of index Index.</summary>
    public TrimmedCurve2D Value(int index)
    {
        if (UnModified())
        {
            throw new OutOfRangeException();
        }
        if (index < 1 || index > _curves.Length())
        {
            throw new OutOfRangeException();
        }
        return (TrimmedCurve2D)_curves.Value(index);
    }
}
