// Port of: src/ModelingAlgorithms/TKFillet/FilletSurf/FilletSurf_Builder.hxx
// Port of: src/ModelingAlgorithms/TKFillet/FilletSurf/FilletSurf_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Fillets.Surfaces;

/// <summary>
/// The geometry of a fillet of constant radius along a chain of edges, without a shape: its NURBS surfaces and, for
/// each, the two support faces, the 3D curves and pcurves on them, the pcurves on the fillet, the status of its start
/// and end sections, and the parameters of the fillet on the first and last edges.
/// </summary>
/// <remarks>
/// A working object (decision 21) over <see cref="InternalBuilder"/>. The caller's shape is never written
/// (the fillet builder works on a copy) and the support faces are the caller's. The surfaces and curves handed out are
/// the computation's own and frozen: they are published (decision 1138).
/// </remarks>
internal class SurfaceBuilder
{
    private readonly InternalBuilder _intBuild;
    private StatusDone _myisdone;
    private ErrorTypeStatus _myerrorstatus;

    //=================================================================================================

    /// <summary>
    /// Initializes the computation of the fillet of radius <paramref name="r"/> on the shape <paramref name="s"/>
    /// along the edges <paramref name="e"/>. <paramref name="ta"/> is the angular tolerance, <paramref name="tapp3d"/>
    /// and <paramref name="tapp2d"/> the 3D and 2D approximation tolerances.
    /// </summary>
    public SurfaceBuilder(in Shape s, NCollection_List<Shape> e, double r, double ta = 1.0e-2,
                              double tapp3d = 1.0e-4, double tapp2d = 1.0e-5)
    {
        _intBuild = new InternalBuilder(s, global::Kerf.Fillets.FilletShape.Polynomial, ta, tapp3d, tapp2d);
        _myisdone = StatusDone.IsOk;
        _myerrorstatus = ErrorTypeStatus.EmptyList;
        int add = _intBuild.Add(e, r);
        if (add != 0)
        {
            _myisdone = StatusDone.IsNotOk;
            if (add == 1)
            {
                _myerrorstatus = ErrorTypeStatus.EmptyList;
            }
            else if (add == 2)
            {
                _myerrorstatus = ErrorTypeStatus.EdgeNotG1;
            }
            else if (add == 3)
            {
                _myerrorstatus = ErrorTypeStatus.FacesNotG1;
            }
            else if (add == 4)
            {
                _myerrorstatus = ErrorTypeStatus.EdgeNotOnShape;
            }
            else if (add == 5)
            {
                _myerrorstatus = ErrorTypeStatus.NotSharpEdge;
            }
        }
    }

    //=================================================================================================

    /// <summary>Computes the fillet (a list of NURBS surfaces).</summary>
    public void Perform()
    {
        if (_myisdone == StatusDone.IsOk)
        {
            _intBuild.Perform();
            if (_intBuild.Done())
            {
                _myisdone = StatusDone.IsOk;
            }
            else if (_intBuild.SurfaceCount != 0)
            {
                _myisdone = StatusDone.IsPartial;
                _myerrorstatus = ErrorTypeStatus.PbFilletCompute;
            }
            else
            {
                _myisdone = StatusDone.IsNotOk;
                _myerrorstatus = ErrorTypeStatus.PbFilletCompute;
            }
        }
    }

    //=================================================================================================

    /// <summary>The status of the computation: IsOk, IsNotOk (no result) or IsPartial.</summary>
    public StatusDone IsDone
    {
        get
        {
            return _myisdone;
        }
    }

    //=================================================================================================

    /// <summary>
    /// The error when <see cref="IsDone"/> is IsNotOk: EdgeNotG1, FacesNotG1, EdgeNotOnShape, NotSharpEdge or
    /// PbFilletCompute.
    /// </summary>
    public ErrorTypeStatus StatusError()
    {
        return _myerrorstatus;
    }

    //=================================================================================================

    /// <summary>The number of NURBS surfaces of the fillet.</summary>
    /// <exception cref="NotDoneException">The computation gave no result.</exception>
    public int NbSurface()
    {
        if (IsDone != StatusDone.IsNotOk)
        {
            return _intBuild.SurfaceCount;
        }
        throw new NotDoneException("FilletSurf_Builder::NbSurface");
    }

    private void CheckIndex(int index, string what)
    {
        if ((index < 1) || (index > NbSurface()))
        {
            throw new OutOfRangeException("FilletSurf_Builder::" + what);
        }
    }

    private static T Published<T>(T theObject) where T : Transient?
    {
        theObject?.Freeze();
        return theObject;
    }

    //=================================================================================================

    /// <summary>The NURBS surface <paramref name="index"/> (frozen).</summary>
    public Surface SurfaceFillet(int index)
    {
        CheckIndex(index, "SurfaceFillet");
        return Published(_intBuild.SurfaceFillet(index))!;
    }

    //=================================================================================================

    /// <summary>The 3D tolerance reached by the approximation of the surface <paramref name="index"/>.</summary>
    public double TolApp3d(int index)
    {
        CheckIndex(index, "TolApp3d");
        return _intBuild.TolApp3d(index);
    }

    //=================================================================================================

    /// <summary>The first support face of the surface <paramref name="index"/>.</summary>
    public Face SupportFace1(int index)
    {
        CheckIndex(index, "SupportFace1");
        return _intBuild.SupportFace1(index);
    }

    //=================================================================================================

    /// <summary>The second support face of the surface <paramref name="index"/>.</summary>
    public Face SupportFace2(int index)
    {
        CheckIndex(index, "SupportFace2");
        return _intBuild.SupportFace2(index);
    }

    //=================================================================================================

    /// <summary>The 3D curve of the surface <paramref name="index"/> on its first support face (frozen).</summary>
    public Curve3D CurveOnFace1(int index)
    {
        CheckIndex(index, "CurveOnFace1");
        return Published(_intBuild.CurveOnFace1(index))!;
    }

    //=================================================================================================

    /// <summary>The 3D curve of the surface <paramref name="index"/> on its second support face (frozen).</summary>
    public Curve3D CurveOnFace2(int index)
    {
        CheckIndex(index, "CurveOnFace2");
        return Published(_intBuild.CurveOnFace2(index))!;
    }

    //=================================================================================================

    /// <summary>The pcurve of <see cref="CurveOnFace1"/> on the first support face (frozen).</summary>
    public Curve2D PCurveOnFace1(int index)
    {
        CheckIndex(index, "PCurveOnFace1");
        return Published(_intBuild.PCurveOnFace1(index))!;
    }

    //=================================================================================================

    /// <summary>The pcurve of <see cref="CurveOnFace1"/> on the fillet (frozen).</summary>
    public Curve2D PCurve1OnFillet(int index)
    {
        CheckIndex(index, "PCurve1OnFillet");
        return Published(_intBuild.PCurve1OnFillet(index))!;
    }

    //=================================================================================================

    /// <summary>The pcurve of <see cref="CurveOnFace2"/> on the second support face (frozen).</summary>
    public Curve2D PCurveOnFace2(int index)
    {
        CheckIndex(index, "PCurveOnFace2");
        return Published(_intBuild.PCurveOnFace2(index))!;
    }

    //=================================================================================================

    /// <summary>The pcurve of <see cref="CurveOnFace2"/> on the fillet (frozen).</summary>
    public Curve2D PCurve2OnFillet(int index)
    {
        CheckIndex(index, "PCurve2OnFillet");
        return Published(_intBuild.PCurve2OnFillet(index))!;
    }

    //=================================================================================================

    /// <summary>The parameter of the fillet on the first edge.</summary>
    public double FirstParameter()
    {
        if (IsDone == StatusDone.IsNotOk)
        {
            throw new NotDoneException("FilletSurf_Builder::FirstParameter");
        }
        return _intBuild.FirstParameter();
    }

    //=================================================================================================

    /// <summary>The parameter of the fillet on the last edge.</summary>
    public double LastParameter()
    {
        if (IsDone == StatusDone.IsNotOk)
        {
            throw new NotDoneException("FilletSurf_Builder::LastParameter");
        }
        return _intBuild.LastParameter();
    }

    //=================================================================================================

    /// <summary>
    /// The start section: both of its ends on an edge of their support faces (TwoExtremityOnEdge), one, or none.
    /// </summary>
    public SurfaceStatusType StartSectionStatus()
    {
        if (IsDone == StatusDone.IsNotOk)
        {
            throw new NotDoneException("FilletSurf_Builder::StartSectionStatus");
        }
        return _intBuild.StartSectionStatus();
    }

    //=================================================================================================

    /// <summary>The end section: both of its ends on an edge of their support faces, one, or none.</summary>
    public SurfaceStatusType EndSectionStatus()
    {
        if (IsDone == StatusDone.IsNotOk)
        {
            throw new NotDoneException("FilletSurf_Builder::StartSectionStatus"); // OCCT's message
        }
        return _intBuild.EndSectionStatus();
    }

    //=================================================================================================

    /// <summary>Computes only the sections used in the computation of the fillet.</summary>
    public void Simulate()
    {
        if (_myisdone == StatusDone.IsOk)
        {
            _intBuild.Simulate();

            if (_intBuild.Done())
            {
                _myisdone = StatusDone.IsOk;
            }
            else
            {
                _myisdone = StatusDone.IsNotOk;
                _myerrorstatus = ErrorTypeStatus.PbFilletCompute;
            }
        }
    }

    //=================================================================================================

    /// <summary>The number of sections of the surface <paramref name="indexSurf"/> (after <see cref="Simulate"/>).</summary>
    public int NbSection(int indexSurf)
    {
        if (IsDone == StatusDone.IsNotOk)
        {
            throw new NotDoneException("FilletSurf_Builder::NbSection)");
        }
        else if ((indexSurf < 1) || (indexSurf > NbSurface()))
        {
            throw new OutOfRangeException("FilletSurf_Builder::NbSection");
        }
        return _intBuild.NbSection(indexSurf);
    }

    //=================================================================================================

    /// <summary>
    /// The arc of the section <paramref name="indexSec"/> of the surface <paramref name="indexSurf"/>: a trimmed
    /// Geom_Circle (frozen).
    /// </summary>
    public TrimmedCurve3D Section(int indexSurf, int indexSec)
    {
        if ((indexSurf < 1) || (indexSurf > NbSurface()))
        {
            throw new OutOfRangeException("FilletSurf_Builder::Section NbSurface");
        }
        else if ((indexSec < 1) || (indexSec > NbSection(indexSurf)))
        {
            throw new OutOfRangeException("FilletSurf_Builder::Section NbSection");
        }

        return Published(_intBuild.Section(indexSurf, indexSec));
    }
}
