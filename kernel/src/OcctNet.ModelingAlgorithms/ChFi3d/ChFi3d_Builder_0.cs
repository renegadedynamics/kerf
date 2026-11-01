// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.hxx (ChFi3d_Coefficient)
// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.cxx (ChFi3d_Coefficient)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The fillet builders' free functions (<c>ChFi3d_*</c>), as a static class named after the file.</summary>
/// <remarks>
/// Only <c>ChFi3d_Coefficient</c> is ported, for <see cref="ChFi3d.ConcaveSide"/>, which the offset's analysis reaches
/// (unit 7E, plan 1.3). The fillet units add the rest of the file here.
/// </remarks>
internal static partial class ChFi3d_Builder_0
{
    /// <summary>
    /// The components (<paramref name="DU"/>, <paramref name="DV"/>) in the parametric space of the vector
    /// <paramref name="V3d"/> on the tangent plane spanned by <paramref name="D1u"/> and <paramref name="D1v"/>
    /// (least squares).
    /// </summary>
    internal static void ChFi3d_Coefficient(in gp_Vec V3d, in gp_Vec D1u, in gp_Vec D1v, out double DU, out double DV)
    {
        double AA = D1u.SquareMagnitude();
        double BB = D1u.Dot(D1v);
        double CC = D1v.SquareMagnitude();
        double DD = D1u.Dot(V3d);
        double EE = D1v.Dot(V3d);
        double Delta = AA * CC - BB * BB;
        DU = (DD * CC - EE * BB) / Delta;
        DV = (AA * EE - BB * DD) / Delta;
    }
}
