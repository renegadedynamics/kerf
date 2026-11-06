// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_TransferParameters.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_TransferParameters.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;
using Kerf.Engine.Healing.Build;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Healing.Analysis;

/// <summary>
/// Transfers parameters between an edge's 3D curve and its pcurve on a face, by the linear map between their ranges
/// (the edge is taken as SameParameter). Subclasses (ShapeAnalysis_TransferParametersProj) compute it by projection.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="TransferRange"/> writes the ranges of a new edge in place, so it is
/// internal: ported code calls it on edges it is building.
/// </remarks>
internal class TransferParametersAnalysis : Transient
{
    protected double _first;
    protected double _last;
    protected Edge _edge = new Edge();
    // OCCT leaves it uninitialised until SetMaxTolerance; 0 here
    protected double _maxTolerance;

    private double _shift;
    private double _scale;
    private double _first2d;
    private double _last2d;

    /// <summary>An empty tool: the identity map.</summary>
    public TransferParametersAnalysis()
    {
        _scale = 1.0;
        _shift = 0.0;
    }

    /// <summary>A tool for the edge on the face (see <see cref="Init"/>).</summary>
    public TransferParametersAnalysis(in Edge e, in Face f)
    {
        Init(e, f);
    }

    /// <summary>
    /// Loads the edge and the face, and computes the linear map from the 3D curve's range to the pcurve's (the
    /// identity if either is missing; a null face is allowed, for free edges).
    /// </summary>
    public virtual void Init(in Edge e, in Face F)
    {
        _scale = 1.0;
        _shift = 0.0;
        double l, f, l2d = 0.0, f2d = 0.0;
        _edge = e;
        var sae = new EdgeAnalysis();
        sae.Curve3d(e, out Curve3D? curve3d, out f, out l, false);
        _first = f;
        _last = l;
        Curve2D? curve2d = null;
        if (!F.IsNull)
        { // process free edges
            sae.PCurve(e, F, out curve2d, out f2d, out l2d, false);
        }

        _first2d = f2d;
        _last2d = l2d;
        if (curve3d is null || curve2d is null)
        {
            return;
        }

        double ln2d = l2d - f2d;
        double ln3d = l - f;
        _scale = (ln3d <= Constants.Resolution() ? 1.0 : ln2d / ln3d);
        _shift = f2d - f * _scale;
    }

    /// <summary>Sets the maximal tolerance to use in the computations.</summary>
    public void SetMaxTolerance(double maxtol)
    {
        _maxTolerance = maxtol;
    }

    /// <summary>Transfers the parameters (to the pcurve if <paramref name="to2d"/>, else to the 3D curve).</summary>
    public virtual NCollection_HSequence<double> Perform(NCollection_HSequence<double> Params, bool to2d)
    {
        var res = new NCollection_HSequence<double>();
        for (int i = 1; i <= Params.Length(); i++)
        {
            res.Append(Perform(Params.Value(i), to2d));
        }

        return res;
    }

    /// <summary>Transfers one parameter (to the pcurve if <paramref name="to2d"/>, else to the 3D curve).</summary>
    public virtual double Perform(double param, bool to2d)
    {
        double newParam;
        if (to2d)
        {
            newParam = _shift + param * _scale;
        }
        else
        {
            newParam = -_shift / _scale + param * 1.0 / _scale;
        }

        return newParam;
    }

    /// <summary>
    /// Sets on <paramref name="newEdge"/> (a part of the loaded edge) the ranges of the loaded edge's curves between
    /// the parameters <paramref name="prevPar"/> and <paramref name="currPar"/>, of the pcurve if
    /// <paramref name="is2d"/>, else of the 3D curve (ShapeBuild_Edge::CopyRanges).
    /// </summary>
    /// <remarks>Internal: it writes into the edge, which throws on a frozen one.</remarks>
    internal virtual void TransferRange(in Edge newEdge, double prevPar, double currPar, bool is2d)
    {
        var sbe = new BuildEdgeBuilder();
        if (is2d)
        {
            double span2d = _last2d - _first2d;
            double tmp1, tmp2;
            if (prevPar > currPar)
            {
                tmp1 = currPar;
                tmp2 = prevPar;
            }
            else
            {
                tmp1 = prevPar;
                tmp2 = currPar;
            }

            double alpha = (tmp1 - _first2d) / span2d;
            double beta = (tmp2 - _first2d) / span2d;
            sbe.CopyRanges(newEdge, _edge, alpha, beta);
        }
        else
        {
            double alpha = (prevPar - _first) / (_last - _first);
            double beta = (currPar - _first) / (_last - _first);
            sbe.CopyRanges(newEdge, _edge, alpha, beta);
        }
    }

    /// <summary>Tells if the map is the identity (no shift, scale 1).</summary>
    public virtual bool IsSameRange
    {
        get
        {
            return _shift == 0.0 && _scale == 1.0;
        }
    }
}
