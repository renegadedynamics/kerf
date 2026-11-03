// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_TgtOnCoons.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_TgtOnCoons.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// An algorithmic field of tangents along boundary I of a <see cref="GeomFill_CoonsAlgPatch"/>: the patch's
/// cross-boundary derivative with its component along the boundary's normal removed.
/// </summary>
/// <remarks>A working object (decision 806); it keeps the caller's patch, as OCCT keeps the handle.</remarks>
public class GeomFill_TgtOnCoons : GeomFill_TgtField
{
    private readonly GeomFill_CoonsAlgPatch myK;
    private readonly int ibound;

    /// <summary>The field along boundary <paramref name="I"/> (0 to 3) of <paramref name="K"/>.</summary>
    public GeomFill_TgtOnCoons(GeomFill_CoonsAlgPatch K, int I)
    {
        myK = K;
        ibound = I;
    }

    public override Vector3D Value(double W)
    {
        double U = 0.0, V = 0.0, bid = 0.0;
        switch (ibound)
        {
            case 0:
                myK.Bound(1).Bounds(out V, out bid);
                break;
            case 1:
                myK.Bound(0).Bounds(out bid, out U);
                break;
            case 2:
                myK.Bound(1).Bounds(out bid, out V);
                break;
            case 3:
                myK.Bound(0).Bounds(out U, out bid);
                break;
        }

        Vector3D tgk = new();

        switch (ibound)
        {
            case 0:
            case 2:
                U = W;
                tgk = myK.D1V(U, V);
                break;
            case 1:
            case 3:
                V = W;
                tgk = myK.D1U(U, V);
                break;
        }

        Vector3D n = myK.Bound(ibound).Norm(W);
        double scal = tgk.Dot(n);
        n.Multiply(-scal);
        tgk.Add(n);
        return tgk;
    }

    public override Vector3D D1(double W)
    {
        double U = 0.0, V = 0.0, bid = 0.0;
        switch (ibound)
        {
            case 0:
                myK.Bound(1).Bounds(out V, out bid);
                break;
            case 1:
                myK.Bound(0).Bounds(out bid, out U);
                break;
            case 2:
                myK.Bound(1).Bounds(out bid, out V);
                break;
            case 3:
                myK.Bound(0).Bounds(out U, out bid);
                break;
        }

        Vector3D tgsc = new(), dtgsc;

        switch (ibound)
        {
            case 0:
            case 2:
                U = W;
                tgsc = myK.D1V(U, V);
                break;
            case 1:
            case 3:
                V = W;
                tgsc = myK.D1U(U, V);
                break;
        }
        dtgsc = myK.DUV(U, V);

        Vector3D n, dn;
        myK.Bound(ibound).D1Norm(W, out n, out dn);

        double scal = tgsc.Dot(n);
        Vector3D scaln = n.Multiplied(-scal);
        tgsc.Add(scaln);

        Vector3D scaldn = dn.Multiplied(-scal);

        double scal2 = -dtgsc.Dot(n) - tgsc.Dot(dn);
        Vector3D temp = n.Multiplied(scal2);

        temp.Add(scaldn);
        Vector3D dtpur = dtgsc.Added(temp);

        return dtpur;
    }

    public override void D1(double W, out Vector3D T, out Vector3D DT)
    {
        double U = 0.0, V = 0.0, bid = 0.0;
        switch (ibound)
        {
            case 0:
                myK.Bound(1).Bounds(out V, out bid);
                break;
            case 1:
                myK.Bound(0).Bounds(out bid, out U);
                break;
            case 2:
                myK.Bound(1).Bounds(out bid, out V);
                break;
            case 3:
                myK.Bound(0).Bounds(out U, out bid);
                break;
        }

        Vector3D tgsc = new(), dtgsc;

        switch (ibound)
        {
            case 0:
            case 2:
                U = W;
                tgsc = myK.D1V(U, V);
                break;
            case 1:
            case 3:
                V = W;
                tgsc = myK.D1U(U, V);
                break;
        }
        dtgsc = myK.DUV(U, V);

        Vector3D n, dn;
        myK.Bound(ibound).D1Norm(W, out n, out dn);

        double scal = tgsc.Dot(n);
        Vector3D scaln = n.Multiplied(-scal);
        T = tgsc.Added(scaln);

        Vector3D scaldn = dn.Multiplied(-scal);

        double scal2 = -dtgsc.Dot(n) - tgsc.Dot(dn);
        Vector3D temp = n.Multiplied(scal2);

        temp.Add(scaldn);
        DT = dtgsc.Added(temp);
    }
}
