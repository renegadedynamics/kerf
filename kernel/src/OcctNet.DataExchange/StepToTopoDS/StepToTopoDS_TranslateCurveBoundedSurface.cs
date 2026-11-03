// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateCurveBoundedSurface.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateCurveBoundedSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Topology.Brep;

using Kerf.Geometry;

using Kerf.Topology;

namespace Occt;

/// <summary>Translates a curve_bounded_surface into a TopoDS_Face.</summary>
/// <remarks>
/// Needs <see cref="ShapeAlgo.Init"/>. <see cref="Value"/> freezes the result; the translators use
/// <c>InternalValue</c> (decision 1247).
/// </remarks>
public sealed class StepToTopoDS_TranslateCurveBoundedSurface : StepToTopoDS_Root
{
    private Face myFace = new Face();

    public StepToTopoDS_TranslateCurveBoundedSurface()
    {
    }

    public StepToTopoDS_TranslateCurveBoundedSurface(StepGeom_CurveBoundedSurface CBS, Transfer_TransientProcess TP, in StepData_Factors theLocalFactors)
    {
        Init(CBS, TP, theLocalFactors);
    }

    public StepToTopoDS_TranslateCurveBoundedSurface(StepGeom_CurveBoundedSurface CBS, Transfer_TransientProcess TP)
        : this(CBS, TP, new StepData_Factors())
    {
    }

    public bool Init(StepGeom_CurveBoundedSurface CBS, Transfer_TransientProcess TP, in StepData_Factors theLocalFactors)
    {
        myFace.Nullify();
        if (CBS is null)
        {
            return false;
        }

        // translate basis surface
        StepGeom_Surface S = CBS.BasisSurface();
        Surface Surf = StepToGeom.MakeSurface(S, theLocalFactors);
        if (Surf is null)
        {
            TP.AddFail(CBS, "Basis surface not translated");
            return false;
        }

        // abv 30.06.00: trj4_k1_geo-tu.stp #108: do as in TranslateFace
        // pdn to force bsplsurf to be periodic
        StepGeom_BSplineSurface sgbss = S as StepGeom_BSplineSurface;
        if (sgbss is not null)
        {
            Surface periodicSurf = ShapeAlgo.AlgoContainer().ConvertToPeriodic(Surf);
            if (periodicSurf is not null)
            {
                TP.AddWarning(S, "Surface forced to be periodic");
                Surf = periodicSurf;
            }
        }

        // create face
        var B = new BrepBuilder();
        B.MakeFace(ref myFace, Surf, Kerf.Engine.Foundation.Precision.Confusion());

        // add natural bound if implicit
        if (CBS.ImplicitOuter())
        {
            if (Surf is BoundedSurface)
            {
                var mf = new BRepBuilderAPI_MakeFace(Surf, Kerf.Engine.Foundation.Precision.Confusion());
                myFace = mf.InternalFace();
            }
            else
            {
                TP.AddWarning(CBS, "Cannot make natural bounds on infinite surface");
            }
        }

        // translate boundaries
        NCollection_HArray1<StepGeom_SurfaceBoundary> bnd = CBS.Boundaries();
        int nb = bnd.Length();
        for (int i = 1; i <= nb; i++)
        {
            StepGeom_CompositeCurve cc = bnd.Value(i)?.BoundaryCurve();
            if (cc is null)
            {
                continue;
            }

            var TrCC = new StepToTopoDS_TranslateCompositeCurve(cc, TP, S, Surf, theLocalFactors);
            if (!TrCC.IsDone())
            {
                TP.AddWarning(CBS, "Boundary not translated");
                continue;
            }

            B.Add(myFace, TrCC.InternalValue());
        }

        done = !myFace.IsNull;
        return done;
    }

    public bool Init(StepGeom_CurveBoundedSurface CBS, Transfer_TransientProcess TP) => Init(CBS, TP, new StepData_Factors());

    /// <summary>The result of the last translation, frozen; a null face if it failed.</summary>
    public Face Value()
    {
        Face aFace = myFace;
        aFace.TShape()?.FreezeGraph();
        return aFace;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal Face InternalValue() => myFace;
}
