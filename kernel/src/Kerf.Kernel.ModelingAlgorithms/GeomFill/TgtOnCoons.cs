// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_TgtOnCoons.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_TgtOnCoons.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// An algorithmic field of tangents along boundary I of a <see cref="CoonsAlgPatch"/>: the patch's
/// cross-boundary derivative with its component along the boundary's normal removed.
/// </summary>
/// <remarks>A working object (decision 806); it keeps the caller's patch, as OCCT keeps the handle.</remarks>
internal class TgtOnCoons : TgtField
{
    private readonly CoonsAlgPatch _k;
    private readonly int _ibound;

    /// <summary>The field along boundary <paramref name="i"/> (0 to 3) of <paramref name="k"/>.</summary>
    public TgtOnCoons(CoonsAlgPatch k, int i)
    {
        _k = k;
        _ibound = i;
    }

    public override Vector3D Value(double w)
    {
        double u = 0.0, v = 0.0, bid = 0.0;
        switch (_ibound)
        {
            case 0:
                _k.Bound(1).Bounds(out v, out bid);
                break;
            case 1:
                _k.Bound(0).Bounds(out bid, out u);
                break;
            case 2:
                _k.Bound(1).Bounds(out bid, out v);
                break;
            case 3:
                _k.Bound(0).Bounds(out u, out bid);
                break;
        }

        Vector3D tgk = new();

        switch (_ibound)
        {
            case 0:
            case 2:
                u = w;
                tgk = _k.D1V(u, v);
                break;
            case 1:
            case 3:
                v = w;
                tgk = _k.D1U(u, v);
                break;
        }

        Vector3D n = _k.Bound(_ibound).Norm(w);
        double scal = tgk.Dot(n);
        n = n.Multiplied(-scal);
        tgk = tgk.Added(n);
        return tgk;
    }

    public override Vector3D D1(double w)
    {
        double u = 0.0, v = 0.0, bid = 0.0;
        switch (_ibound)
        {
            case 0:
                _k.Bound(1).Bounds(out v, out bid);
                break;
            case 1:
                _k.Bound(0).Bounds(out bid, out u);
                break;
            case 2:
                _k.Bound(1).Bounds(out bid, out v);
                break;
            case 3:
                _k.Bound(0).Bounds(out u, out bid);
                break;
        }

        Vector3D tgsc = new(), dtgsc;

        switch (_ibound)
        {
            case 0:
            case 2:
                u = w;
                tgsc = _k.D1V(u, v);
                break;
            case 1:
            case 3:
                v = w;
                tgsc = _k.D1U(u, v);
                break;
        }
        dtgsc = _k.DUV(u, v);

        Vector3D n, dn;
        _k.Bound(_ibound).D1Norm(w, out n, out dn);

        double scal = tgsc.Dot(n);
        Vector3D scaln = n.Multiplied(-scal);
        tgsc = tgsc.Added(scaln);

        Vector3D scaldn = dn.Multiplied(-scal);

        double scal2 = -dtgsc.Dot(n) - tgsc.Dot(dn);
        Vector3D temp = n.Multiplied(scal2);

        temp = temp.Added(scaldn);
        Vector3D dtpur = dtgsc.Added(temp);

        return dtpur;
    }

    public override void D1(double w, out Vector3D t, out Vector3D dt)
    {
        double u = 0.0, v = 0.0, bid = 0.0;
        switch (_ibound)
        {
            case 0:
                _k.Bound(1).Bounds(out v, out bid);
                break;
            case 1:
                _k.Bound(0).Bounds(out bid, out u);
                break;
            case 2:
                _k.Bound(1).Bounds(out bid, out v);
                break;
            case 3:
                _k.Bound(0).Bounds(out u, out bid);
                break;
        }

        Vector3D tgsc = new(), dtgsc;

        switch (_ibound)
        {
            case 0:
            case 2:
                u = w;
                tgsc = _k.D1V(u, v);
                break;
            case 1:
            case 3:
                v = w;
                tgsc = _k.D1U(u, v);
                break;
        }
        dtgsc = _k.DUV(u, v);

        Vector3D n, dn;
        _k.Bound(_ibound).D1Norm(w, out n, out dn);

        double scal = tgsc.Dot(n);
        Vector3D scaln = n.Multiplied(-scal);
        t = tgsc.Added(scaln);

        Vector3D scaldn = dn.Multiplied(-scal);

        double scal2 = -dtgsc.Dot(n) - tgsc.Dot(dn);
        Vector3D temp = n.Multiplied(scal2);

        temp = temp.Added(scaldn);
        dt = dtgsc.Added(temp);
    }
}
