// Port of: src/ModelingAlgorithms/TKFillet/FilletSurf/FilletSurf_Builder.hxx
// Port of: src/ModelingAlgorithms/TKFillet/FilletSurf/FilletSurf_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.FilletSurf_ErrorTypeStatus;
using static Occt.FilletSurf_StatusDone;

namespace Occt;

/// <summary>
/// The geometry of a fillet of constant radius along a chain of edges, without a shape: its NURBS surfaces and, for
/// each, the two support faces, the 3D curves and pcurves on them, the pcurves on the fillet, the status of its start
/// and end sections, and the parameters of the fillet on the first and last edges.
/// </summary>
/// <remarks>
/// A working object (decision 21) over <see cref="FilletSurf_InternalBuilder"/>. The caller's shape is never written
/// (the fillet builder works on a copy) and the support faces are the caller's. The surfaces and curves handed out are
/// the computation's own and frozen: they are published (decision 1138).
/// </remarks>
public class FilletSurf_Builder
{
    private readonly FilletSurf_InternalBuilder myIntBuild;
    private FilletSurf_StatusDone myisdone;
    private FilletSurf_ErrorTypeStatus myerrorstatus;

    //=================================================================================================

    /// <summary>
    /// Initializes the computation of the fillet of radius <paramref name="R"/> on the shape <paramref name="S"/>
    /// along the edges <paramref name="E"/>. <paramref name="Ta"/> is the angular tolerance, <paramref name="Tapp3d"/>
    /// and <paramref name="Tapp2d"/> the 3D and 2D approximation tolerances.
    /// </summary>
    public FilletSurf_Builder(in Shape S, NCollection_List<Shape> E, double R, double Ta = 1.0e-2,
                              double Tapp3d = 1.0e-4, double Tapp2d = 1.0e-5)
    {
        myIntBuild = new FilletSurf_InternalBuilder(S, ChFi3d_FilletShape.ChFi3d_Polynomial, Ta, Tapp3d, Tapp2d);
        myisdone = FilletSurf_IsOk;
        myerrorstatus = FilletSurf_EmptyList;
        int add = myIntBuild.Add(E, R);
        if (add != 0)
        {
            myisdone = FilletSurf_IsNotOk;
            if (add == 1)
            {
                myerrorstatus = FilletSurf_EmptyList;
            }
            else if (add == 2)
            {
                myerrorstatus = FilletSurf_EdgeNotG1;
            }
            else if (add == 3)
            {
                myerrorstatus = FilletSurf_FacesNotG1;
            }
            else if (add == 4)
            {
                myerrorstatus = FilletSurf_EdgeNotOnShape;
            }
            else if (add == 5)
            {
                myerrorstatus = FilletSurf_NotSharpEdge;
            }
        }
    }

    //=================================================================================================

    /// <summary>Computes the fillet (a list of NURBS surfaces).</summary>
    public void Perform()
    {
        if (myisdone == FilletSurf_IsOk)
        {
            myIntBuild.Perform();
            if (myIntBuild.Done())
            {
                myisdone = FilletSurf_IsOk;
            }
            else if (myIntBuild.NbSurface() != 0)
            {
                myisdone = FilletSurf_IsPartial;
                myerrorstatus = FilletSurf_PbFilletCompute;
            }
            else
            {
                myisdone = FilletSurf_IsNotOk;
                myerrorstatus = FilletSurf_PbFilletCompute;
            }
        }
    }

    //=================================================================================================

    /// <summary>The status of the computation: IsOk, IsNotOk (no result) or IsPartial.</summary>
    public FilletSurf_StatusDone IsDone()
    {
        return myisdone;
    }

    //=================================================================================================

    /// <summary>
    /// The error when <see cref="IsDone"/> is IsNotOk: EdgeNotG1, FacesNotG1, EdgeNotOnShape, NotSharpEdge or
    /// PbFilletCompute.
    /// </summary>
    public FilletSurf_ErrorTypeStatus StatusError()
    {
        return myerrorstatus;
    }

    //=================================================================================================

    /// <summary>The number of NURBS surfaces of the fillet.</summary>
    /// <exception cref="NotDoneException">The computation gave no result.</exception>
    public int NbSurface()
    {
        if (IsDone() != FilletSurf_IsNotOk)
        {
            return myIntBuild.NbSurface();
        }
        throw new NotDoneException("FilletSurf_Builder::NbSurface");
    }

    private void checkIndex(int Index, string theWhat)
    {
        if ((Index < 1) || (Index > NbSurface()))
        {
            throw new OutOfRangeException("FilletSurf_Builder::" + theWhat);
        }
    }

    private static T Published<T>(T theObject) where T : Transient?
    {
        theObject?.Freeze();
        return theObject;
    }

    //=================================================================================================

    /// <summary>The NURBS surface <paramref name="Index"/> (frozen).</summary>
    public Surface SurfaceFillet(int Index)
    {
        checkIndex(Index, "SurfaceFillet");
        return Published(myIntBuild.SurfaceFillet(Index))!;
    }

    //=================================================================================================

    /// <summary>The 3D tolerance reached by the approximation of the surface <paramref name="Index"/>.</summary>
    public double TolApp3d(int Index)
    {
        checkIndex(Index, "TolApp3d");
        return myIntBuild.TolApp3d(Index);
    }

    //=================================================================================================

    /// <summary>The first support face of the surface <paramref name="Index"/>.</summary>
    public Face SupportFace1(int Index)
    {
        checkIndex(Index, "SupportFace1");
        return myIntBuild.SupportFace1(Index);
    }

    //=================================================================================================

    /// <summary>The second support face of the surface <paramref name="Index"/>.</summary>
    public Face SupportFace2(int Index)
    {
        checkIndex(Index, "SupportFace2");
        return myIntBuild.SupportFace2(Index);
    }

    //=================================================================================================

    /// <summary>The 3D curve of the surface <paramref name="Index"/> on its first support face (frozen).</summary>
    public Curve3D CurveOnFace1(int Index)
    {
        checkIndex(Index, "CurveOnFace1");
        return Published(myIntBuild.CurveOnFace1(Index))!;
    }

    //=================================================================================================

    /// <summary>The 3D curve of the surface <paramref name="Index"/> on its second support face (frozen).</summary>
    public Curve3D CurveOnFace2(int Index)
    {
        checkIndex(Index, "CurveOnFace2");
        return Published(myIntBuild.CurveOnFace2(Index))!;
    }

    //=================================================================================================

    /// <summary>The pcurve of <see cref="CurveOnFace1"/> on the first support face (frozen).</summary>
    public Curve2D PCurveOnFace1(int Index)
    {
        checkIndex(Index, "PCurveOnFace1");
        return Published(myIntBuild.PCurveOnFace1(Index))!;
    }

    //=================================================================================================

    /// <summary>The pcurve of <see cref="CurveOnFace1"/> on the fillet (frozen).</summary>
    public Curve2D PCurve1OnFillet(int Index)
    {
        checkIndex(Index, "PCurve1OnFillet");
        return Published(myIntBuild.PCurve1OnFillet(Index))!;
    }

    //=================================================================================================

    /// <summary>The pcurve of <see cref="CurveOnFace2"/> on the second support face (frozen).</summary>
    public Curve2D PCurveOnFace2(int Index)
    {
        checkIndex(Index, "PCurveOnFace2");
        return Published(myIntBuild.PCurveOnFace2(Index))!;
    }

    //=================================================================================================

    /// <summary>The pcurve of <see cref="CurveOnFace2"/> on the fillet (frozen).</summary>
    public Curve2D PCurve2OnFillet(int Index)
    {
        checkIndex(Index, "PCurve2OnFillet");
        return Published(myIntBuild.PCurve2OnFillet(Index))!;
    }

    //=================================================================================================

    /// <summary>The parameter of the fillet on the first edge.</summary>
    public double FirstParameter()
    {
        if (IsDone() == FilletSurf_IsNotOk)
        {
            throw new NotDoneException("FilletSurf_Builder::FirstParameter");
        }
        return myIntBuild.FirstParameter();
    }

    //=================================================================================================

    /// <summary>The parameter of the fillet on the last edge.</summary>
    public double LastParameter()
    {
        if (IsDone() == FilletSurf_IsNotOk)
        {
            throw new NotDoneException("FilletSurf_Builder::LastParameter");
        }
        return myIntBuild.LastParameter();
    }

    //=================================================================================================

    /// <summary>
    /// The start section: both of its ends on an edge of their support faces (TwoExtremityOnEdge), one, or none.
    /// </summary>
    public FilletSurf_StatusType StartSectionStatus()
    {
        if (IsDone() == FilletSurf_IsNotOk)
        {
            throw new NotDoneException("FilletSurf_Builder::StartSectionStatus");
        }
        return myIntBuild.StartSectionStatus();
    }

    //=================================================================================================

    /// <summary>The end section: both of its ends on an edge of their support faces, one, or none.</summary>
    public FilletSurf_StatusType EndSectionStatus()
    {
        if (IsDone() == FilletSurf_IsNotOk)
        {
            throw new NotDoneException("FilletSurf_Builder::StartSectionStatus"); // OCCT's message
        }
        return myIntBuild.EndSectionStatus();
    }

    //=================================================================================================

    /// <summary>Computes only the sections used in the computation of the fillet.</summary>
    public void Simulate()
    {
        if (myisdone == FilletSurf_IsOk)
        {
            myIntBuild.Simulate();

            if (myIntBuild.Done())
            {
                myisdone = FilletSurf_IsOk;
            }
            else
            {
                myisdone = FilletSurf_IsNotOk;
                myerrorstatus = FilletSurf_PbFilletCompute;
            }
        }
    }

    //=================================================================================================

    /// <summary>The number of sections of the surface <paramref name="IndexSurf"/> (after <see cref="Simulate"/>).</summary>
    public int NbSection(int IndexSurf)
    {
        if (IsDone() == FilletSurf_IsNotOk)
        {
            throw new NotDoneException("FilletSurf_Builder::NbSection)");
        }
        else if ((IndexSurf < 1) || (IndexSurf > NbSurface()))
        {
            throw new OutOfRangeException("FilletSurf_Builder::NbSection");
        }
        return myIntBuild.NbSection(IndexSurf);
    }

    //=================================================================================================

    /// <summary>
    /// The arc of the section <paramref name="IndexSec"/> of the surface <paramref name="IndexSurf"/>: a trimmed
    /// Geom_Circle (frozen).
    /// </summary>
    public TrimmedCurve3D Section(int IndexSurf, int IndexSec)
    {
        if ((IndexSurf < 1) || (IndexSurf > NbSurface()))
        {
            throw new OutOfRangeException("FilletSurf_Builder::Section NbSurface");
        }
        else if ((IndexSec < 1) || (IndexSec > NbSection(IndexSurf)))
        {
            throw new OutOfRangeException("FilletSurf_Builder::Section NbSection");
        }

        return Published(myIntBuild.Section(IndexSurf, IndexSec));
    }
}
