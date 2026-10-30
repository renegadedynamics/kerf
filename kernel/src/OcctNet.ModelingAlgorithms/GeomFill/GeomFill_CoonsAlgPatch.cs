// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_CoonsAlgPatch.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_CoonsAlgPatch.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The algorithmic Coons patch of four boundaries and two blending functions (linear by default), evaluated
/// directly: its value and its first and mixed derivatives.
/// </summary>
/// <remarks>
/// A working object (decision 806): it keeps the caller's boundaries and laws, as OCCT keeps the handles;
/// <see cref="SetFunc"/> replaces the blending functions. No check is made on the bounds: B1/B3 and B2/B4 must have
/// the same range and be well oriented. The corners are the midpoints of the adjacent boundaries' ends.
/// </remarks>
public class GeomFill_CoonsAlgPatch : Standard_Transient
{
    private readonly GeomFill_Boundary[] bound = new GeomFill_Boundary[4];
    private readonly gp_Pnt[] c = [new gp_Pnt(), new gp_Pnt(), new gp_Pnt(), new gp_Pnt()];
    private readonly Law_Function[] a = new Law_Function[2];

    public GeomFill_CoonsAlgPatch(GeomFill_Boundary B1, GeomFill_Boundary B2, GeomFill_Boundary B3, GeomFill_Boundary B4)
    {
        bound[0] = B1;
        bound[1] = B2;
        bound[2] = B3;
        bound[3] = B4;
        double deb0, deb1, fin0, fin1;
        B2.Bounds(out deb1, out fin1);
        Law_Linear aLaw0 = new();
        aLaw0.Set(deb1, 1.0, fin1, 0.0);
        a[0] = aLaw0;
        B1.Bounds(out deb0, out fin0);
        Law_Linear aLaw1 = new();
        aLaw1.Set(deb0, 0.0, fin0, 1.0);
        a[1] = aLaw1;

        gp_XYZ temp;
        temp = B4.Value(deb1).XYZ().Added(B1.Value(deb0).XYZ());
        temp.Multiply(0.5);
        c[0].SetXYZ(temp);
        temp = B1.Value(fin0).XYZ().Added(B2.Value(deb1).XYZ());
        temp.Multiply(0.5);
        c[1].SetXYZ(temp);
        temp = B2.Value(fin1).XYZ().Added(B3.Value(fin0).XYZ());
        temp.Multiply(0.5);
        c[2].SetXYZ(temp);
        temp = B3.Value(deb0).XYZ().Added(B4.Value(fin1).XYZ());
        temp.Multiply(0.5);
        c[3].SetXYZ(temp);
    }

    /// <summary>The blending functions.</summary>
    public void Func(out Law_Function f1, out Law_Function f2)
    {
        f1 = a[0];
        f2 = a[1];
    }

    /// <summary>Sets the blending functions (kept, not copied).</summary>
    public void SetFunc(Law_Function f1, Law_Function f2)
    {
        a[0] = f1;
        a[1] = f2;
    }

    /// <summary>The value of the patch at (U, V).</summary>
    public gp_Pnt Value(double U, double V)
    {
        double a0, a1, a2, a3;
        a0 = a[0].Value(V);
        a1 = a[1].Value(U);
        a2 = 1.0 - a0;
        a3 = 1.0 - a1;
        gp_XYZ cor, cortemp;

        // OCCT: bounds 0 and 2 (the U boundaries) are evaluated at V, not U (occt-issues.md)
        cor = bound[0].Value(V).XYZ();
        cor.Multiply(a0);

        cortemp = bound[1].Value(V).XYZ();
        cortemp.Multiply(a1);
        cor.Add(cortemp);

        cortemp = bound[2].Value(V).XYZ();
        cortemp.Multiply(a2);
        cor.Add(cortemp);

        cortemp = bound[3].Value(V).XYZ();
        cortemp.Multiply(a3);
        cor.Add(cortemp);

        cortemp = c[0].XYZ();
        cortemp.Multiply(-a0 * a3);
        cor.Add(cortemp);

        cortemp = c[1].XYZ();
        cortemp.Multiply(-a0 * a1);
        cor.Add(cortemp);

        cortemp = c[2].XYZ();
        cortemp.Multiply(-a1 * a2);
        cor.Add(cortemp);

        cortemp = c[3].XYZ();
        cortemp.Multiply(-a2 * a3);
        cor.Add(cortemp);

        return new gp_Pnt(cor);
    }

    /// <summary>The partial derivative d/dU at (U, V).</summary>
    public gp_Vec D1U(double U, double V)
    {
        double a0, a1, a2, a3, bid;
        a0 = a[0].Value(V);
        a[1].D1(U, out bid, out a1);
        a2 = 1 - a0;
        a3 = -a1;
        gp_XYZ cor, cortemp;
        gp_Pnt pbid;
        gp_Vec vbid;

        bound[0].D1(U, out pbid, out vbid);
        cor = vbid.XYZ();
        cor.Multiply(a0);

        cortemp = bound[1].Value(V).XYZ();
        cortemp.Multiply(a1);
        cor.Add(cortemp);

        bound[2].D1(U, out pbid, out vbid);
        cortemp = vbid.XYZ();
        cortemp.Multiply(a2);
        cor.Add(cortemp);

        cortemp = bound[3].Value(V).XYZ();
        cortemp.Multiply(a3);
        cor.Add(cortemp);

        cortemp = c[0].XYZ();
        cortemp.Multiply(-a0 * a3);
        cor.Add(cortemp);

        cortemp = c[1].XYZ();
        cortemp.Multiply(-a0 * a1);
        cor.Add(cortemp);

        cortemp = c[2].XYZ();
        cortemp.Multiply(-a1 * a2);
        cor.Add(cortemp);

        cortemp = c[3].XYZ();
        cortemp.Multiply(-a2 * a3);
        cor.Add(cortemp);

        vbid.SetXYZ(cor);
        return vbid;
    }

    /// <summary>The partial derivative d/dV at (U, V).</summary>
    public gp_Vec D1V(double U, double V)
    {
        double a0, a1, a2, a3, bid;
        a[0].D1(V, out bid, out a0);
        a1 = a[1].Value(U);
        a2 = -a0;
        a3 = 1.0 - a1;
        gp_XYZ cor, cortemp;
        gp_Pnt pbid;
        gp_Vec vbid;

        cor = bound[0].Value(U).XYZ();
        cor.Multiply(a0);

        bound[1].D1(V, out pbid, out vbid);
        cortemp = vbid.XYZ();
        cortemp.Multiply(a1);
        cor.Add(cortemp);

        cortemp = bound[2].Value(U).XYZ();
        cortemp.Multiply(a2);
        cor.Add(cortemp);

        bound[3].D1(V, out pbid, out vbid);
        cortemp = vbid.XYZ();
        cortemp.Multiply(a3);
        cor.Add(cortemp);

        cortemp = c[0].XYZ();
        cortemp.Multiply(-a0 * a3);
        cor.Add(cortemp);

        cortemp = c[1].XYZ();
        cortemp.Multiply(-a0 * a1);
        cor.Add(cortemp);

        cortemp = c[2].XYZ();
        cortemp.Multiply(-a1 * a2);
        cor.Add(cortemp);

        cortemp = c[3].XYZ();
        cortemp.Multiply(-a2 * a3);
        cor.Add(cortemp);

        vbid.SetXYZ(cor);
        return vbid;
    }

    /// <summary>The mixed derivative d2/dUdV at (U, V) (exact for linear blending functions).</summary>
    public gp_Vec DUV(double U, double V)
    {
        double a0, a1, a2, a3, bid;
        a[0].D1(V, out bid, out a0);
        a[1].D1(U, out bid, out a1);
        a2 = -a0;
        a3 = -a1;

        gp_XYZ cor, cortemp;
        gp_Pnt pbid;
        gp_Vec vbid;

        bound[0].D1(U, out pbid, out vbid);
        cor = vbid.XYZ();
        cor.Multiply(a0);

        bound[1].D1(V, out pbid, out vbid);
        cortemp = vbid.XYZ();
        cortemp.Multiply(a1);
        cor.Add(cortemp);

        bound[2].D1(U, out pbid, out vbid);
        cortemp = vbid.XYZ();
        cortemp.Multiply(a2);
        cor.Add(cortemp);

        bound[3].D1(V, out pbid, out vbid);
        cortemp = vbid.XYZ();
        cortemp.Multiply(a3);
        cor.Add(cortemp);

        cortemp = c[0].XYZ();
        cortemp.Multiply(-a0 * a3);
        cor.Add(cortemp);

        cortemp = c[1].XYZ();
        cortemp.Multiply(-a0 * a1);
        cor.Add(cortemp);

        cortemp = c[2].XYZ();
        cortemp.Multiply(-a1 * a2);
        cor.Add(cortemp);

        cortemp = c[3].XYZ();
        cortemp.Multiply(-a2 * a3);
        cor.Add(cortemp);

        vbid.SetXYZ(cor);
        return vbid;
    }

    /// <summary>The corner <paramref name="I"/> (from 0).</summary>
    public gp_Pnt Corner(int I) => c[I];

    /// <summary>The boundary <paramref name="I"/> (from 0).</summary>
    public GeomFill_Boundary Bound(int I) => bound[I];

    /// <summary>The blending function <paramref name="I"/> (0 or 1).</summary>
    public Law_Function Func(int I) => a[I];
}
