// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_BoxBndTree.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_BoxBndTree.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace Kerf.Engine.Healing.Analysis;

/// <summary>
/// The selector of the tree of wire end boxes used by <see cref="FreeBounds"/>'s ConnectWiresToWires:
/// accepts the wire whose end is shared with (or closest to) the current wire's ends.
/// </summary>
/// <remarks>
/// A working object for ported code (decision 21). <c>Accept</c> reads the wires of the sequence it is given; it
/// writes nothing into shapes.
/// </remarks>
internal sealed class BoxBndTreeSelectorAnalysis : SpatialTree<int, Box>.Selector
{
    private Box _fBox;
    private Box _lBox;
    private readonly NCollection_HArray1<Shape> _seq;
    private readonly bool _shared;
    private int _nb;
    private Vertex _fVertex;
    private Vertex _lVertex;
    private Point3D _fPnt;
    private Point3D _lPnt;
    private readonly HashSet<int> _list = new();
    private double _tol;
    private double _min3d;
    private readonly NCollection_Array1<int> _arrIndices;
    private int _status;

    /// <summary>A selector over the wires of <paramref name="seq"/>, by shared vertices or by distance.</summary>
    public BoxBndTreeSelectorAnalysis(NCollection_HArray1<Shape> seq, bool shared)
    {
        _seq = seq;
        _shared = shared;
        _nb = 0;
        _tol = 1e-7;
        _min3d = 1e-7;
        _arrIndices = new NCollection_Array1<int>(1, 2);
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        _fBox = new Box();
        _lBox = new Box();
        _fVertex = new Vertex();
        _lVertex = new Vertex();
        _fPnt = new Point3D();
        _lPnt = new Point3D();
        _arrIndices.Init(0);
    }

    /// <summary>Sets the boxes of the current wire's first and last vertices.</summary>
    public void DefineBoxes(in Box fBox, in Box lBox)
    {
        _fBox = fBox;
        _lBox = lBox;
        _arrIndices.Init(0);
    }

    /// <summary>Sets the current wire's first and last vertices.</summary>
    public void DefineVertexes(Vertex vf, Vertex vl)
    {
        _fVertex = vf;
        _lVertex = vl;
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
    }

    /// <summary>Sets the current wire's first and last points.</summary>
    public void DefinePnt(in Point3D fPnt, in Point3D lPnt)
    {
        _fPnt = fPnt;
        _lPnt = lPnt;
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
    }

    /// <summary>The index of the wire accepted.</summary>
    public int GetNb() => _nb;

    /// <summary>Sets the index of the wire accepted.</summary>
    public void SetNb(int nb) => _nb = nb;

    /// <summary>Excludes the wire of the given index from the selection.</summary>
    public void LoadList(int elem) => _list.Add(elem);

    /// <summary>Clears the stop flag.</summary>
    public void SetStop() => _stop = false;

    /// <summary>Sets the tolerance of the distance mode.</summary>
    public void SetTolerance(double tol)
    {
        _tol = tol;
        _min3d = tol;
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
    }

    /// <summary>True if the wire of the given index is excluded.</summary>
    public bool ContWire(int nbWire) => _list.Contains(nbWire);

    /// <summary>
    /// The status of the last acceptance: DONE1 (DONE2) the current wire's last vertex is the wire's first (last);
    /// DONE3 (DONE4) its first vertex is the wire's last (first); FAIL2 none.
    /// </summary>
    public bool LastCheckStatus(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_status, status);

    /// <summary>True if the box is out of both end boxes.</summary>
    public override bool Reject(in Box bnd)
    {
        bool fch = _fBox.IsOut(bnd);
        bool lch = _lBox.IsOut(bnd);
        return fch && lch;
    }

    private const int First = 1;
    private const int Last = 2;

    /// <summary>Accepts the wire of the given index if it connects to the current wire.</summary>
    public override bool Accept(in int obj)
    {
        if (obj < 1 || obj > _seq.Length())
        {
            throw new NoSuchObjectException("ShapeAnalysis_BoxBndTreeSelector::Accept : no such object for current index");
        }
        bool isAccept = false;
        if (_list.Contains(obj))
        {
            return false;
        }

        Wire w = ShapeCasts.Wire(_seq.Value(obj));
        ShapeAnalysisAnalysis.FindBounds(w, out Vertex v1, out Vertex v2);
        if (_shared)
        {
            if (_lVertex.IsSame(v1))
            {
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                isAccept = true;
                _arrIndices[Last] = obj;
            }
            else
            {
                if (_lVertex.IsSame(v2))
                {
                    _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                    isAccept = true;
                    _arrIndices[Last] = obj;
                }
                else
                {
                    if (_fVertex.IsSame(v2))
                    {
                        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
                        isAccept = true;
                        _arrIndices[First] = obj;
                    }
                    else
                    {
                        if (_fVertex.IsSame(v1))
                        {
                            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE4);
                            isAccept = true;
                            _arrIndices[First] = obj;
                        }
                        else
                        {
                            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                        }
                    }
                }
            }

            if (isAccept)
            {
                SetNb(obj);
                if (_arrIndices[Last] != 0)
                {
                    _stop = true;
                }
                return true;
            }
            else
            {
                _stop = false;
            }
        }

        else
        {
            Point3D p1 = Tool.Pnt(v1);
            Point3D p2 = Tool.Pnt(v2);

            double tailhead, tailtail, headhead, headtail;
            tailhead = p1.DistanceTo(_lPnt);
            tailtail = p2.DistanceTo(_lPnt);
            headhead = p1.DistanceTo(_fPnt);
            headtail = p2.DistanceTo(_fPnt);
            double dm1 = tailhead, dm2 = headtail;
            int res1 = 0, res2 = 0;
            if (tailhead > tailtail)
            {
                res1 = 1;
                dm1 = tailtail;
            }
            if (headtail > headhead)
            {
                res2 = 1;
                dm2 = headhead;
            }
            int result = res1;
            double min3d;
            min3d = Min(dm1, dm2);
            if (min3d > _min3d)
            {
                return false;
            }

            int minInd = (dm1 > dm2 ? First : Last);
            int maxInd = (dm1 > dm2 ? Last : First);
            _arrIndices[minInd] = obj;
            if ((min3d - _min3d) > RealSmall())
            {
                _arrIndices[maxInd] = 0;
            }

            _min3d = min3d;
            if (min3d > _tol)
            {
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                return false;
            }

            int anObj = (_arrIndices[Last] != 0 ? _arrIndices[Last] : _arrIndices[First]);
            SetNb(anObj);

            if (min3d == 0 && minInd == Last)
            {
                _stop = true;
            }

            if (dm1 > dm2)
            {
                dm1 = dm2;
                result = res2 + 2;
            }
            if (anObj == obj)
            {
                switch (result)
                {
                    case 0:
                        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                        break;
                    case 1:
                        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                        break;
                    case 2:
                        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
                        break;
                    case 3:
                        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE4);
                        break;
                }
            }
            return true;
        }

        return false;
    }
}
