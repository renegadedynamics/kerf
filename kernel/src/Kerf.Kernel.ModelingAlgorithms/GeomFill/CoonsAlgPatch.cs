// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_CoonsAlgPatch.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_CoonsAlgPatch.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// The algorithmic Coons patch of four boundaries and two blending functions (linear by default), evaluated
/// directly: its value and its first and mixed derivatives.
/// </summary>
/// <remarks>
/// A working object (decision 806): it keeps the caller's boundaries and laws, as OCCT keeps the handles;
/// <see cref="SetFunc"/> replaces the blending functions. No check is made on the bounds: B1/B3 and B2/B4 must have
/// the same range and be well oriented. The corners are the midpoints of the adjacent boundaries' ends.
/// </remarks>
public class CoonsAlgPatch : Transient
{
    private readonly Boundary[] _bound = new Boundary[4];
    private readonly Point3D[] _c = [new Point3D(), new Point3D(), new Point3D(), new Point3D()];
    private readonly LawsFunction[] _a = new LawsFunction[2];

    public CoonsAlgPatch(Boundary b1, Boundary b2, Boundary b3, Boundary b4)
    {
        _bound[0] = b1;
        _bound[1] = b2;
        _bound[2] = b3;
        _bound[3] = b4;
        double deb0, deb1, fin0, fin1;
        b2.Bounds(out deb1, out fin1);
        Linear law0 = new();
        law0.Set(deb1, 1.0, fin1, 0.0);
        _a[0] = law0;
        b1.Bounds(out deb0, out fin0);
        Linear law1 = new();
        law1.Set(deb0, 0.0, fin0, 1.0);
        _a[1] = law1;

        Xyz temp;
        temp = b4.Value(deb1).XYZ().Added(b1.Value(deb0).XYZ());
        temp.Multiply(0.5);
        _c[0].SetXYZ(temp);
        temp = b1.Value(fin0).XYZ().Added(b2.Value(deb1).XYZ());
        temp.Multiply(0.5);
        _c[1].SetXYZ(temp);
        temp = b2.Value(fin1).XYZ().Added(b3.Value(fin0).XYZ());
        temp.Multiply(0.5);
        _c[2].SetXYZ(temp);
        temp = b3.Value(deb0).XYZ().Added(b4.Value(fin1).XYZ());
        temp.Multiply(0.5);
        _c[3].SetXYZ(temp);
    }

    /// <summary>The blending functions.</summary>
    public void Func(out LawsFunction f1, out LawsFunction f2)
    {
        f1 = _a[0];
        f2 = _a[1];
    }

    /// <summary>Sets the blending functions (kept, not copied).</summary>
    public void SetFunc(LawsFunction f1, LawsFunction f2)
    {
        _a[0] = f1;
        _a[1] = f2;
    }

    /// <summary>The value of the patch at (U, V).</summary>
    public Point3D Value(double u, double v)
    {
        double a0, a1, a2, a3;
        a0 = _a[0].Value(v);
        a1 = _a[1].Value(u);
        a2 = 1.0 - a0;
        a3 = 1.0 - a1;
        Xyz cor, cortemp;

        // OCCT: bounds 0 and 2 (the U boundaries) are evaluated at V, not U (occt-issues.md)
        cor = _bound[0].Value(v).XYZ();
        cor.Multiply(a0);

        cortemp = _bound[1].Value(v).XYZ();
        cortemp.Multiply(a1);
        cor.Add(cortemp);

        cortemp = _bound[2].Value(v).XYZ();
        cortemp.Multiply(a2);
        cor.Add(cortemp);

        cortemp = _bound[3].Value(v).XYZ();
        cortemp.Multiply(a3);
        cor.Add(cortemp);

        cortemp = _c[0].XYZ();
        cortemp.Multiply(-a0 * a3);
        cor.Add(cortemp);

        cortemp = _c[1].XYZ();
        cortemp.Multiply(-a0 * a1);
        cor.Add(cortemp);

        cortemp = _c[2].XYZ();
        cortemp.Multiply(-a1 * a2);
        cor.Add(cortemp);

        cortemp = _c[3].XYZ();
        cortemp.Multiply(-a2 * a3);
        cor.Add(cortemp);

        return new Point3D(cor);
    }

    /// <summary>The partial derivative d/dU at (U, V).</summary>
    public Vector3D D1U(double u, double v)
    {
        double a0, a1, a2, a3, bid;
        a0 = _a[0].Value(v);
        _a[1].D1(u, out bid, out a1);
        a2 = 1 - a0;
        a3 = -a1;
        Xyz cor, cortemp;
        Point3D pbid;
        Vector3D vbid;

        _bound[0].D1(u, out pbid, out vbid);
        cor = vbid.XYZ();
        cor.Multiply(a0);

        cortemp = _bound[1].Value(v).XYZ();
        cortemp.Multiply(a1);
        cor.Add(cortemp);

        _bound[2].D1(u, out pbid, out vbid);
        cortemp = vbid.XYZ();
        cortemp.Multiply(a2);
        cor.Add(cortemp);

        cortemp = _bound[3].Value(v).XYZ();
        cortemp.Multiply(a3);
        cor.Add(cortemp);

        cortemp = _c[0].XYZ();
        cortemp.Multiply(-a0 * a3);
        cor.Add(cortemp);

        cortemp = _c[1].XYZ();
        cortemp.Multiply(-a0 * a1);
        cor.Add(cortemp);

        cortemp = _c[2].XYZ();
        cortemp.Multiply(-a1 * a2);
        cor.Add(cortemp);

        cortemp = _c[3].XYZ();
        cortemp.Multiply(-a2 * a3);
        cor.Add(cortemp);

        vbid.SetXYZ(cor);
        return vbid;
    }

    /// <summary>The partial derivative d/dV at (U, V).</summary>
    public Vector3D D1V(double u, double v)
    {
        double a0, a1, a2, a3, bid;
        _a[0].D1(v, out bid, out a0);
        a1 = _a[1].Value(u);
        a2 = -a0;
        a3 = 1.0 - a1;
        Xyz cor, cortemp;
        Point3D pbid;
        Vector3D vbid;

        cor = _bound[0].Value(u).XYZ();
        cor.Multiply(a0);

        _bound[1].D1(v, out pbid, out vbid);
        cortemp = vbid.XYZ();
        cortemp.Multiply(a1);
        cor.Add(cortemp);

        cortemp = _bound[2].Value(u).XYZ();
        cortemp.Multiply(a2);
        cor.Add(cortemp);

        _bound[3].D1(v, out pbid, out vbid);
        cortemp = vbid.XYZ();
        cortemp.Multiply(a3);
        cor.Add(cortemp);

        cortemp = _c[0].XYZ();
        cortemp.Multiply(-a0 * a3);
        cor.Add(cortemp);

        cortemp = _c[1].XYZ();
        cortemp.Multiply(-a0 * a1);
        cor.Add(cortemp);

        cortemp = _c[2].XYZ();
        cortemp.Multiply(-a1 * a2);
        cor.Add(cortemp);

        cortemp = _c[3].XYZ();
        cortemp.Multiply(-a2 * a3);
        cor.Add(cortemp);

        vbid.SetXYZ(cor);
        return vbid;
    }

    /// <summary>The mixed derivative d2/dUdV at (U, V) (exact for linear blending functions).</summary>
    public Vector3D DUV(double u, double v)
    {
        double a0, a1, a2, a3, bid;
        _a[0].D1(v, out bid, out a0);
        _a[1].D1(u, out bid, out a1);
        a2 = -a0;
        a3 = -a1;

        Xyz cor, cortemp;
        Point3D pbid;
        Vector3D vbid;

        _bound[0].D1(u, out pbid, out vbid);
        cor = vbid.XYZ();
        cor.Multiply(a0);

        _bound[1].D1(v, out pbid, out vbid);
        cortemp = vbid.XYZ();
        cortemp.Multiply(a1);
        cor.Add(cortemp);

        _bound[2].D1(u, out pbid, out vbid);
        cortemp = vbid.XYZ();
        cortemp.Multiply(a2);
        cor.Add(cortemp);

        _bound[3].D1(v, out pbid, out vbid);
        cortemp = vbid.XYZ();
        cortemp.Multiply(a3);
        cor.Add(cortemp);

        cortemp = _c[0].XYZ();
        cortemp.Multiply(-a0 * a3);
        cor.Add(cortemp);

        cortemp = _c[1].XYZ();
        cortemp.Multiply(-a0 * a1);
        cor.Add(cortemp);

        cortemp = _c[2].XYZ();
        cortemp.Multiply(-a1 * a2);
        cor.Add(cortemp);

        cortemp = _c[3].XYZ();
        cortemp.Multiply(-a2 * a3);
        cor.Add(cortemp);

        vbid.SetXYZ(cor);
        return vbid;
    }

    /// <summary>The corner <paramref name="i"/> (from 0).</summary>
    public Point3D Corner(int i) => _c[i];

    /// <summary>The boundary <paramref name="i"/> (from 0).</summary>
    public Boundary Bound(int i) => _bound[i];

    /// <summary>The blending function <paramref name="i"/> (0 or 1).</summary>
    public LawsFunction Func(int i) => _a[i];
}
