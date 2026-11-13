// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateCurveBoundedSurface.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateCurveBoundedSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Healing;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;

using Kerf.Geometry;

using Kerf.Topology;

namespace Kerf.Engine.Exchange.Step.Import;

/// <summary>Translates a curve_bounded_surface into a TopoDS_Face.</summary>
/// <remarks>
/// Needs <see cref="ShapeAlgo.Init"/>. <see cref="Value"/> freezes the result; the translators use
/// <c>InternalValue</c> (decision 1247).
/// </remarks>
internal sealed class TranslateCurveBoundedSurface : ImportRoot
{
    private Face _face = new Face();

    public TranslateCurveBoundedSurface()
    {
    }

    public TranslateCurveBoundedSurface(StepCurveBoundedSurface cbs, TransientProcess tp, in Factors localFactors)
    {
        Init(cbs, tp, localFactors);
    }

    public TranslateCurveBoundedSurface(StepCurveBoundedSurface cbs, TransientProcess tp)
        : this(cbs, tp, new Factors())
    {
    }

    public bool Init(StepCurveBoundedSurface cbs, TransientProcess tp, in Factors localFactors)
    {
        _face.Nullify();
        if (cbs is null)
        {
            return false;
        }

        // translate basis surface
        StepSurface s = cbs.BasisSurface();
        Surface surf = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeSurface(s, localFactors);
        if (surf is null)
        {
            tp.AddFail(cbs, "Basis surface not translated");
            return false;
        }

        // abv 30.06.00: trj4_k1_geo-tu.stp #108: do as in TranslateFace
        // pdn to force bsplsurf to be periodic
        StepBSplineSurface sgbss = s as StepBSplineSurface;
        if (sgbss is not null)
        {
            Surface periodicSurf = global::Kerf.Engine.Healing.ShapeAlgo.AlgoContainer().ConvertToPeriodic(surf);
            if (periodicSurf is not null)
            {
                tp.AddWarning(s, "Surface forced to be periodic");
                surf = periodicSurf;
            }
        }

        // create face
        var b = new BrepBuilder();
        b.MakeFace(ref _face, surf, Kerf.Engine.Foundation.Precision.Confusion());

        // add natural bound if implicit
        if (cbs.ImplicitOuter())
        {
            if (surf is BoundedSurface)
            {
                var mf = new MakeFace(surf, Kerf.Engine.Foundation.Precision.Confusion());
                _face = mf.InternalFace();
            }
            else
            {
                tp.AddWarning(cbs, "Cannot make natural bounds on infinite surface");
            }
        }

        // translate boundaries
        NCollection_HArray1<StepSurfaceBoundary> bnd = cbs.Boundaries();
        int nb = bnd.Length();
        for (int i = 1; i <= nb; i++)
        {
            StepCompositeCurve cc = bnd.Value(i)?.BoundaryCurve();
            if (cc is null)
            {
                continue;
            }

            var trCC = new TranslateCompositeCurve(cc, tp, s, surf, localFactors);
            if (!trCC.IsDone)
            {
                tp.AddWarning(cbs, "Boundary not translated");
                continue;
            }

            Wire wire = trCC.InternalValue();
            if (cc is StepOuterBoundaryCurve && nb == 1 && !cbs.ImplicitOuter() && IsWoundAsHole(_face, wire, surf))
            {
                // #1652 (decision 35002): the file says this wire is the outer boundary. Wound clockwise on a sphere or a
                // torus, the healing would take it for a hole and add the natural bounds, giving the rest of the surface
                // for the patch; so it's wound as an outer boundary here, where that is still known.
                var sewd = new ExtendWireData(wire);
                sewd.Reverse();
                wire = sewd.InternalWire();
            }

            b.Add(_face, wire);
        }

        _done = !_face.IsNull;
        return _done;
    }

    // Whether the wire, alone on the face's surface, would bound a hole rather than a patch: only where the healing's
    // natural bounds could follow (a sphere, or a surface periodic both ways), for a wire with no seam or degenerated edge
    // that doesn't run round a period, as the healing's own test has it (ShapeFix_Face::isNeedAddNaturalBound).
    private static bool IsWoundAsHole(in Face face, Wire wire, Surface surf)
    {
        bool sphere = surf is SphericalSurface;
        if (!sphere && !(surf.IsUPeriodic && surf.IsVPeriodic))
        {
            return false;
        }

        Face alone = ShapeCasts.Face(face.EmptyCopied());
        alone.Orient(Orientation.Forward);
        new BrepBuilder().Add(alone, wire);
        for (var it = new Explorer(alone, ShapeType.Edge); it.More(); it.Next())
        {
            Edge edge = ShapeCasts.Edge(it.Current());
            if (Tool.Degenerated(edge) || Tool.IsClosed(edge, alone))
            {
                return false;
            }
        }

        ShapeAnalysisAnalysis.GetFaceUVBounds(alone, out double uMin, out double uMax, out double vMin, out double vMax);
        if (surf.IsUPeriodic && uMax - uMin >= surf.UPeriod * (1 - 1e-6)
            || surf.IsVPeriodic && vMax - vMin >= surf.VPeriod * (1 - 1e-6))
        {
            return false;
        }

        return !ShapeAnalysisAnalysis.IsOuterBound(alone);
    }

    public bool Init(StepCurveBoundedSurface cbs, TransientProcess tp) => Init(cbs, tp, new Factors());

    /// <summary>The result of the last translation, frozen; a null face if it failed.</summary>
    public Face Value()
    {
        Face face = _face;
        face.TShape()?.FreezeGraph();
        return face;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal Face InternalValue() => _face;
}
