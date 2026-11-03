// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Framework.hxx
// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Framework.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;
using static Kerf.Engine.Geometry.IsoType;

namespace Kerf.Engine.Approximation.Advanced;

/// <summary>
/// The constraints of <see cref="ApproxAFunc2Var"/>: the nodes of the grid, and its iso-curves in strips
/// (OCCT's "strips" are sequences of <see cref="Iso"/>; AdvApp2Var has no Strip class).
/// </summary>
/// <remarks>
/// A working object of the approximation, with OCCT's public mutators (decision 194). The constructor copies the
/// sequences and their strips, as OCCT's does; the nodes and isos are shared.
/// </remarks>
public class Framework
{
    private readonly NCollection_Sequence<Node> _nodeConstraints;
    private readonly NCollection_Sequence<NCollection_Sequence<Iso>> _uConstraints;
    private readonly NCollection_Sequence<NCollection_Sequence<Iso>> _vConstraints;

    private static int FindStripByBounds(NCollection_Sequence<NCollection_Sequence<Iso>> constraints,
                                         double first,
                                         double last)
    {
        int stripIndex = 1;
        while (stripIndex < constraints.Length()
               && (constraints.Value(stripIndex).First().T0() != first
                   || constraints.Value(stripIndex).First().T1() != last))
        {
            ++stripIndex;
        }
        return stripIndex;
    }

    private static int FindIsoByConstant(NCollection_Sequence<Iso> strip, int nbIso, double theConst)
    {
        int isoIndex = 1;
        while (isoIndex <= nbIso && strip.Value(isoIndex).Constante() != theConst)
        {
            ++isoIndex;
        }
        return isoIndex;
    }

    private static NCollection_Sequence<NCollection_Sequence<Iso>> CopyStrips(
        NCollection_Sequence<NCollection_Sequence<Iso>> strips)
    {
        NCollection_Sequence<NCollection_Sequence<Iso>> copy = new();
        foreach (NCollection_Sequence<Iso> strip in strips)
        {
            copy.Append(new NCollection_Sequence<Iso>(strip));
        }
        return copy;
    }

    public Framework()
    {
        _nodeConstraints = new NCollection_Sequence<Node>();
        _uConstraints = new NCollection_Sequence<NCollection_Sequence<Iso>>();
        _vConstraints = new NCollection_Sequence<NCollection_Sequence<Iso>>();
    }

    public Framework(NCollection_Sequence<Node> frame,
                                NCollection_Sequence<NCollection_Sequence<Iso>> uFrontier,
                                NCollection_Sequence<NCollection_Sequence<Iso>> vFrontier)
    {
        _nodeConstraints = new NCollection_Sequence<Node>(frame);
        _uConstraints = CopyStrips(uFrontier);
        _vConstraints = CopyStrips(vFrontier);
    }

    /// <summary>
    /// The first iso not approximated, with its indices in <paramref name="indexIso"/> and
    /// <paramref name="indexStrip"/>; null (indices unchanged) if every iso is.
    /// </summary>
    public Iso? FirstNotApprox(ref int indexIso, ref int indexStrip)
    {
        for (int uvIter = 0; uvIter < 2; ++uvIter)
        {
            NCollection_Sequence<NCollection_Sequence<Iso>> seq =
                uvIter == 0 ? _uConstraints : _vConstraints;
            int i = 1;
            foreach (NCollection_Sequence<Iso> s in seq)
            {
                int j = 1;
                foreach (Iso iso in s)
                {
                    if (!iso.IsApproximated)
                    {
                        indexIso = j;
                        indexStrip = i;
                        return iso;
                    }
                    j++;
                }
                i++;
            }
        }
        return null;
    }

    public int FirstNode(IsoType type, int indexIso, int indexStrip)
    {
        int nbIsoInV = _uConstraints.Length() + 1;
        if (type == global::Kerf.Engine.Geometry.IsoType.IsoU)
        {
            return nbIsoInV * (indexStrip - 1) + indexIso;
        }
        return nbIsoInV * (indexIso - 1) + indexStrip;
    }

    public int LastNode(IsoType type, int indexIso, int indexStrip)
    {
        int nbIsoInV = _uConstraints.Length() + 1;
        if (type == global::Kerf.Engine.Geometry.IsoType.IsoU)
        {
            return nbIsoInV * indexStrip + indexIso;
        }
        return nbIsoInV * (indexIso - 1) + indexStrip + 1;
    }

    public void ChangeIso(int indexIso, int indexStrip, Iso iso)
    {
        NCollection_Sequence<Iso> strip = iso.Type() == global::Kerf.Engine.Geometry.IsoType.IsoV
                                                          ? _uConstraints.ChangeValue(indexStrip)
                                                          : _vConstraints.ChangeValue(indexStrip);
        strip.SetValue(indexIso, iso);
    }

    public Node Node(int indexNode) => _nodeConstraints.Value(indexNode);

    /// <summary>The node at (<paramref name="u"/>, <paramref name="v"/>), or the last node if there is none.</summary>
    public Node Node(double u, double v)
    {
        foreach (Node node in _nodeConstraints)
        {
            if (node.Coord().X() == u && node.Coord().Y() == v)
            {
                return node;
            }
        }
        return _nodeConstraints.Last();
    }

    public Iso IsoU(double u, double v0, double v1)
    {
        int stripIndex = FindStripByBounds(_vConstraints, v0, v1);
        NCollection_Sequence<Iso> strip = _vConstraints.Value(stripIndex);
        int isoIndex = FindIsoByConstant(strip, _uConstraints.Length(), u);
        return strip.Value(isoIndex);
    }

    public Iso IsoV(double u0, double u1, double v)
    {
        int stripIndex = FindStripByBounds(_uConstraints, u0, u1);
        NCollection_Sequence<Iso> strip = _uConstraints.Value(stripIndex);
        int isoIndex = FindIsoByConstant(strip, _vConstraints.Length(), v);
        return strip.Value(isoIndex);
    }

    public void UpdateInU(double cuttingValue)
    {
        int uStripIndex = 1;
        foreach (NCollection_Sequence<Iso> uConst in _uConstraints)
        {
            if (uConst.First().U0() <= cuttingValue && uConst.First().U1() >= cuttingValue)
            {
                break;
            }
            ++uStripIndex;
        }

        {
            NCollection_Sequence<Iso> strip = _uConstraints.Value(uStripIndex);
            double uFirst = strip.First().U0();
            double uLast = strip.First().U1();

            // Modify the V isos of the U strip at aUStripIndex.
            foreach (Iso iso in strip)
            {
                iso.ChangeDomain(uFirst, cuttingValue);
                iso.ResetApprox();
            }

            // Insert a new U strip after aUStripIndex.
            NCollection_Sequence<Iso> newStrip = new();
            foreach (Iso iso in strip)
            {
                Iso newIso = new(iso.Type(),
                                             iso.Constante(),
                                             cuttingValue,
                                             uLast,
                                             iso.V0(),
                                             iso.V1(),
                                             0,
                                             iso.UOrder(),
                                             iso.VOrder());
                newIso.ResetApprox();
                newStrip.Append(newIso);
            }
            _uConstraints.InsertAfter(uStripIndex, newStrip);
        }

        //  Insert a new Iso U=U* in each V strip after aUStripIndex
        //  and restrict the domains of the adjacent Isos
        for (int vStripIndex = 1; vStripIndex <= _vConstraints.Length(); vStripIndex++)
        {
            NCollection_Sequence<Iso> strip = _vConstraints.ChangeValue(vStripIndex);
            Iso iso = strip.Value(uStripIndex);
            iso.ChangeDomain(iso.U0(), cuttingValue, iso.V0(), iso.V1());

            Iso newIso = new(iso.Type(),
                                         cuttingValue,
                                         iso.U0(),
                                         cuttingValue,
                                         iso.V0(),
                                         iso.V1(),
                                         0,
                                         iso.UOrder(),
                                         iso.VOrder());
            newIso.ResetApprox();
            strip.InsertAfter(uStripIndex, newIso);

            iso = strip.Value(uStripIndex + 2);
            iso.ChangeDomain(cuttingValue, iso.U1(), iso.V0(), iso.V1());
        }

        //  Insert the new nodes (U*,Vj)
        Node next;
        Node prev = _nodeConstraints.First();
        for (int nodeIndex = 1; nodeIndex < _nodeConstraints.Length(); nodeIndex++)
        {
            next = _nodeConstraints.Value(nodeIndex + 1);
            if (prev.Coord().X() < cuttingValue && next.Coord().X() > cuttingValue
                && prev.Coord().Y() == next.Coord().Y())
            {
                Xy newUV = new(cuttingValue, prev.Coord().Y());
                Node newNode = new(newUV, prev.UOrder(), prev.VOrder());
                _nodeConstraints.InsertAfter(nodeIndex, newNode);
            }
            prev = next;
        }
    }

    public void UpdateInV(double cuttingValue)
    {
        int vStripIndex = 1;
        while (_vConstraints.Value(vStripIndex).First().V0() > cuttingValue
               || _vConstraints.Value(vStripIndex).First().V1() < cuttingValue)
        {
            vStripIndex++;
        }

        {
            NCollection_Sequence<Iso> strip = _vConstraints.ChangeValue(vStripIndex);
            double vFirst = strip.First().V0();
            double vLast = strip.First().V1();

            // Modify the U isos of the V strip at aVStripIndex.
            foreach (Iso iso in strip)
            {
                iso.ChangeDomain(vFirst, cuttingValue);
                iso.ResetApprox();
            }

            // Insert a new V strip after aVStripIndex.
            NCollection_Sequence<Iso> newStrip = new();
            foreach (Iso iso in strip)
            {
                Iso newIso = new(iso.Type(),
                                             iso.Constante(),
                                             iso.U0(),
                                             iso.U1(),
                                             cuttingValue,
                                             vLast,
                                             0,
                                             iso.UOrder(),
                                             iso.VOrder());
                newIso.ResetApprox();
                newStrip.Append(newIso);
            }
            _vConstraints.InsertAfter(vStripIndex, newStrip);
        }

        // Insert a new Iso V=V* in each U strip after aVStripIndex
        // and restrict the domains of the adjacent Isos
        foreach (NCollection_Sequence<Iso> strip in _uConstraints)
        {
            Iso iso = strip.Value(vStripIndex);
            iso.ChangeDomain(iso.U0(), iso.U1(), iso.V0(), cuttingValue);

            Iso newIso = new(iso.Type(),
                                         cuttingValue,
                                         iso.U0(),
                                         iso.U1(),
                                         iso.V0(),
                                         cuttingValue,
                                         0,
                                         iso.UOrder(),
                                         iso.VOrder());
            newIso.ResetApprox();
            strip.InsertAfter(vStripIndex, newIso);

            iso = strip.Value(vStripIndex + 2);
            iso.ChangeDomain(iso.U0(), iso.U1(), cuttingValue, iso.V1());
        }

        //  Insert the new nodes (Ui,V*)
        int nodeStartIndex = 1;
        while (nodeStartIndex <= _nodeConstraints.Length()
               && _nodeConstraints.Value(nodeStartIndex).Coord().Y() < cuttingValue)
        {
            nodeStartIndex += _uConstraints.Length() + 1;
        }
        for (int isoIndex = 1; isoIndex <= _uConstraints.Length() + 1; isoIndex++)
        {
            Node jNode = _nodeConstraints.Value(isoIndex);
            Xy newUV = new(jNode.Coord().X(), cuttingValue);
            Node newNode = new(newUV, jNode.UOrder(), jNode.VOrder());
            _nodeConstraints.InsertAfter(nodeStartIndex + isoIndex - 2, newNode);
        }
    }

    /// <summary>The approximation of the U iso <paramref name="indexIso"/> of strip <paramref name="indexStrip"/>.</summary>
    public NCollection_HArray1<double>? UEquation(int indexIso, int indexStrip) =>
        _vConstraints.Value(indexStrip).Value(indexIso).Polynom();

    /// <summary>The approximation of the V iso <paramref name="indexIso"/> of strip <paramref name="indexStrip"/>.</summary>
    public NCollection_HArray1<double>? VEquation(int indexIso, int indexStrip) =>
        _uConstraints.Value(indexStrip).Value(indexIso).Polynom();
}
